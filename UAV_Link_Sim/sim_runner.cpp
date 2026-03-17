#include "sim_runner.h"

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <ctime>

#include "transmitter.h"
#include "receiver.h"
#include "channel.h"

//#define WITH_UHD
#ifdef WITH_UHD
#include "usrp_driver.h"
#endif

static std::string bits_to_string(const VecInt& bits, size_t n = 64)
{
    std::string s;
    size_t m = std::min(n, bits.size());
    for (size_t i = 0; i < m; ++i)
        s.push_back(bits[i] ? '1' : '0');
    return s;
}

static double compute_ber(const VecInt& tx, const VecInt& rx, size_t& bit_errors)
{
    size_t n = std::min(tx.size(), rx.size());
    bit_errors = 0;

    for (size_t i = 0; i < n; ++i)
        if ((tx[i] & 1) != (rx[i] & 1))
            bit_errors++;

    return n ? (double)bit_errors / (double)n : 0.0;
}

std::string mode_to_string(RunMode mode)
{
    switch (mode)
    {
    case RunMode::LOOPBACK: return "LOOPBACK";
    case RunMode::AWGN: return "AWGN";
    case RunMode::USRP: return "USRP";
    default: return "UNKNOWN";
    }
}

std::string modulation_to_string(ModulationType m)
{
    switch (m)
    {
    case ModulationType::BPSK: return "BPSK";
    case ModulationType::QPSK: return "QPSK";
    case ModulationType::QAM: return "QAM";
    case ModulationType::OOK: return "OOK";
    case ModulationType::FSK: return "FSK";
    case ModulationType::FM: return "FM";
    case ModulationType::MSK: return "MSK";
    default: return "UNKNOWN";
    }
}

ModulationType parse_modulation(const std::string& s)
{
    std::string t = s;
    std::transform(t.begin(), t.end(), t.begin(), ::tolower);

    if (t == "bpsk") return ModulationType::BPSK;
    if (t == "qpsk") return ModulationType::QPSK;
    if (t == "qam") return ModulationType::QAM;
    if (t == "ook") return ModulationType::OOK;
    if (t == "fsk") return ModulationType::FSK;
    if (t == "fm") return ModulationType::FM;
    if (t == "msk") return ModulationType::MSK;

    throw std::runtime_error("Unknown modulation");
}

RunMode parse_mode(const std::string& s)
{
    std::string t = s;
    std::transform(t.begin(), t.end(), t.begin(), ::tolower);

    if (t == "loopback") return RunMode::LOOPBACK;
    if (t == "awgn") return RunMode::AWGN;
    if (t == "usrp") return RunMode::USRP;

    throw std::runtime_error("Unknown mode");
}

TestResult run_one_test(
    RunMode mode,
    double awgn_snr_db,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation
)
{
    std::ostringstream log;

    TransmitterConfig cfg;

    cfg.function =
        (modulation == ModulationType::MSK) ?
        FunctionType::RemoteControl :
        FunctionType::Telemetry;

    cfg.modulation = modulation;
    cfg.n = 10;
    cfg.frame_bit = 75;
    cfg.samp = 8;
    cfg.zp_sym = 33;
    cfg.Rb = 50000;
    cfg.connect = (mode == RunMode::USRP);

    Transmitter tx(cfg);

    VecComplex one_frame_sig = tx.generateTransmitSignal();
    VecInt one_frame_bits = tx.getLastSourceBits();

    cfg.fs = tx.getFS();

    Receiver rx(cfg);

    log << "[MODE] " << mode_to_string(mode) << "\n";
    log << "[MOD] " << modulation_to_string(modulation) << "\n";

    VecComplex tx_burst;

    for (int i = 0; i < tx_repeat_frames; ++i)
        tx_burst.insert(tx_burst.end(), one_frame_sig.begin(), one_frame_sig.end());

    VecComplex rx_sig;

    if (mode == RunMode::LOOPBACK)
    {
        rx_sig = tx_burst;
    }
    else if (mode == RunMode::AWGN)
    {
        rx_sig = Channel::awgn(tx_burst, awgn_snr_db);
    }

#ifdef WITH_UHD
    else if (mode == RunMode::USRP)
    {
        USRPDriver::Config uc;

        uc.device_args = "type=b200";
        uc.sample_rate = tx.getFS();
        uc.center_freq = center_freq_hz;

        USRPDriver usrp(uc);
        usrp.init();

        usrp.start_rx_worker(tx_burst.size());

        usrp.send_burst(tx_burst);

        usrp.wait_rx_worker();
        rx_sig = usrp.fetch_rx_buffer();
    }
#endif

    VecInt rx_bits = rx.receive(rx_sig);

    const size_t bits_per_frame = one_frame_bits.size();

    const size_t decoded_frames = rx_bits.size() / bits_per_frame;

    size_t total_compared_bits = 0;
    size_t total_bit_errors = 0;

    for (size_t i = 0; i < decoded_frames; ++i)
    {
        VecInt rx_frame(
            rx_bits.begin() + i * bits_per_frame,
            rx_bits.begin() + (i + 1) * bits_per_frame
        );

        size_t frame_errors = 0;

        compute_ber(one_frame_bits, rx_frame, frame_errors);

        total_bit_errors += frame_errors;
        total_compared_bits += bits_per_frame;
    }

    TestResult tr;

    tr.total_bit_errors = total_bit_errors;
    tr.total_compared_bits = total_compared_bits;
    tr.decoded_frames = decoded_frames;

    tr.total_ber =
        total_compared_bits ?
        (double)total_bit_errors / total_compared_bits :
        0.0;

    log << "BER = " << tr.total_ber << "\n";

    tr.log_text = log.str();

    return tr;
}

SweepResult run_awgn_sweep(
    double snr_start,
    double snr_end,
    double snr_step,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation
)
{
    SweepResult sr;
    std::ostringstream log;

    if (snr_step <= 0.0) {
        throw std::runtime_error("snr_step must be > 0");
    }

    if (snr_start > snr_end) {
        throw std::runtime_error("snr_start must be <= snr_end");
    }

    log << "[SWEEP] Mode = AWGN\n";
    log << "[SWEEP] Modulation = " << modulation_to_string(modulation) << "\n";
    log << "[SWEEP] Range = " << snr_start << " dB -> "
        << snr_end << " dB, step = " << snr_step << " dB\n";

    for (double snr = snr_start; snr <= snr_end + 1e-9; snr += snr_step)
    {
        TestResult tr = run_one_test(
            RunMode::AWGN,
            snr,
            tx_repeat_frames,
            center_freq_hz,
            modulation
        );

        SweepPoint pt;
        pt.snr_db = snr;
        pt.ber = tr.total_ber;
        pt.decoded_frames = tr.decoded_frames;
        sr.points.push_back(pt);

        log << "SNR = " << snr
            << " dB, BER = " << tr.total_ber
            << ", decoded_frames = " << tr.decoded_frames
            << "\n";
    }

    sr.log_text = log.str();
    return sr;
}