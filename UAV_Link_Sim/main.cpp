#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <algorithm>
#include <exception>
#include <thread>
#include <chrono>
#include <sstream>
#include <ctime>

#include "common_defs.h"
#include "transmitter.h"
#include "receiver.h"
#include "channel.h"

#define WITH_UHD
#ifdef WITH_UHD
#include "usrp_driver.h"
#endif

static std::string bits_to_string(const VecInt& bits, size_t n = 64) {
    std::string s;
    size_t m = std::min(n, bits.size());
    s.reserve(m);
    for (size_t i = 0; i < m; ++i) s.push_back(bits[i] ? '1' : '0');
    return s;
}

static double compute_ber(const VecInt& tx, const VecInt& rx, size_t& bit_errors) {
    size_t n = std::min(tx.size(), rx.size());
    bit_errors = 0;
    for (size_t i = 0; i < n; ++i) {
        if ((tx[i] & 1) != (rx[i] & 1)) bit_errors++;
    }
    return (n == 0) ? 0.0 : (double)bit_errors / (double)n;
}

enum class RunMode {
    LOOPBACK,
    AWGN,
    USRP
};

static std::string mode_to_string(RunMode mode) {
    switch (mode) {
    case RunMode::LOOPBACK: return "LOOPBACK";
    case RunMode::AWGN:     return "AWGN";
    case RunMode::USRP:     return "USRP";
    default:                return "UNKNOWN";
    }
}

static std::string modulation_to_string(ModulationType m) {
    switch (m) {
    case ModulationType::BPSK: return "BPSK";
    case ModulationType::QPSK: return "QPSK";
    case ModulationType::QAM:  return "QAM";
    case ModulationType::OOK:  return "OOK";
    case ModulationType::FSK:  return "FSK";
    case ModulationType::FM:   return "FM";
    case ModulationType::MSK:  return "MSK";
    default:                   return "UNKNOWN";
    }
}

static ModulationType parse_modulation(const std::string& s) {
    std::string t = s;
    std::transform(t.begin(), t.end(), t.begin(), ::tolower);

    if (t == "bpsk") return ModulationType::BPSK;
    if (t == "qpsk") return ModulationType::QPSK;
    if (t == "qam")  return ModulationType::QAM;
    if (t == "ook")  return ModulationType::OOK;
    if (t == "fsk")  return ModulationType::FSK;
    if (t == "fm")   return ModulationType::FM;
    if (t == "msk")  return ModulationType::MSK;

    throw std::runtime_error("Unknown modulation: " + s);
}

static RunMode parse_mode(const std::string& s) {
    std::string t = s;
    std::transform(t.begin(), t.end(), t.begin(), ::tolower);

    if (t == "loopback") return RunMode::LOOPBACK;
    if (t == "awgn")     return RunMode::AWGN;
    if (t == "usrp")     return RunMode::USRP;

    throw std::runtime_error("Unknown mode: " + s);
}

struct TestResult {
    size_t total_compared_bits = 0;
    size_t total_bit_errors = 0;
    size_t decoded_frames = 0;
    double total_ber = 0.0;
};

static TestResult run_one_test(
    RunMode mode,
    double awgn_snr_db,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation
) {
    TransmitterConfig cfg;

    if (modulation == ModulationType::MSK) {
        cfg.function = FunctionType::RemoteControl;
    }
    else {
        cfg.function = FunctionType::Telemetry;
    }

    cfg.modulation = modulation;
    cfg.n = 10;
    cfg.frame_bit = 75;
    cfg.samp = 8;
    cfg.zp_sym = 33;
    cfg.Rb = 50000;
    cfg.connect = (mode == RunMode::USRP);

    std::time_t now = std::time(nullptr);

    std::cout << "========================================\n";
    char time_buf[64] = {};
    ctime_s(time_buf, sizeof(time_buf), &now);
    std::cout << "[TEST] Time: " << time_buf;
    std::cout << "[TEST] Mode: " << mode_to_string(mode) << "\n";
    std::cout << "[TEST] Modulation: " << modulation_to_string(cfg.modulation) << "\n";
    std::cout << "[TEST] Frames: " << tx_repeat_frames << "\n";
    std::cout << "[TEST] Frequency: " << center_freq_hz / 1e9 << "GHz\n";
    if (mode == RunMode::AWGN) {
        std::cout << "[TEST] SNR: " << awgn_snr_db << " dB\n";
    }
    std::cout << "========================================\n";

    Transmitter tx(cfg);
    VecComplex one_frame_sig = tx.generateTransmitSignal();
    VecInt one_frame_bits = tx.getLastSourceBits();

    cfg.fs = tx.getFS();
    Receiver rx(cfg);

    std::cout << "One-frame bits: " << one_frame_bits.size()
        << ", samples: " << one_frame_sig.size()
        << ", fs: " << tx.getFS() << " Hz\n";

    const size_t pre_zeros = 200000;
    const size_t post_zeros = 5000;

    VecComplex tx_burst;
    tx_burst.reserve(
        pre_zeros +
        (size_t)tx_repeat_frames * one_frame_sig.size() +
        post_zeros
    );

    tx_burst.insert(tx_burst.end(), pre_zeros, Complex(0.0, 0.0));
    for (int i = 0; i < tx_repeat_frames; ++i) {
        tx_burst.insert(tx_burst.end(), one_frame_sig.begin(), one_frame_sig.end());
    }
    tx_burst.insert(tx_burst.end(), post_zeros, Complex(0.0, 0.0));

    std::cout << "[TX CFG] tx_repeat_frames = " << tx_repeat_frames
        << ", pre_zeros = " << pre_zeros
        << ", post_zeros = " << post_zeros << "\n";

    VecComplex rx_sig;

    if (mode == RunMode::LOOPBACK) {
        std::cout << "[MODE] LOOPBACK\n";
        rx_sig = tx_burst;
        std::cout << "TX burst samples: " << tx_burst.size() << "\n";
        std::cout << "RX captured samples: " << rx_sig.size() << "\n";
    }
    else if (mode == RunMode::AWGN) {
        std::cout << "[MODE] AWGN, SNR=" << awgn_snr_db << " dB\n";
        rx_sig = Channel::awgn(tx_burst, awgn_snr_db);
        std::cout << "TX burst samples: " << tx_burst.size() << "\n";
        std::cout << "RX captured samples: " << rx_sig.size() << "\n";
    }
    else if (mode == RunMode::USRP) {
#ifdef WITH_UHD
        std::cout << "[MODE] USRP\n";

        USRPDriver::Config uc;
        uc.device_args = "type=b200";
        uc.sample_rate = tx.getFS();
        uc.center_freq = center_freq_hz;
        uc.tx_gain = 35;
        uc.rx_gain = 40;
        uc.tx_antenna = "TX/RX";
        uc.rx_antenna = "RX2";

        USRPDriver usrp(uc);
        usrp.init();

        std::cout << "[USRP CFG] sample_rate = " << uc.sample_rate
            << ", fc = " << uc.center_freq
            << ", tx_gain = " << uc.tx_gain
            << ", rx_gain = " << uc.rx_gain << "\n";

        const int rx_extra_frames = 10;
        const size_t rx_need =
            tx_burst.size() + (size_t)rx_extra_frames * one_frame_sig.size();

        std::cout << "[RX CFG] rx_extra_frames = " << rx_extra_frames
            << ", rx_need = " << rx_need << "\n";

        usrp.start_rx_worker(rx_need);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        size_t tx_sent = usrp.send_burst(tx_burst);

        usrp.wait_rx_worker();
        usrp.stop_rx_worker();

        rx_sig = usrp.fetch_rx_buffer();

        std::cout << "TX burst samples: " << tx_burst.size()
            << ", actually sent: " << tx_sent << "\n";
        std::cout << "RX captured samples: " << rx_sig.size() << "\n";
#else
        throw std::runtime_error("UHD disabled");
#endif
    }
    else {
        throw std::runtime_error("Unknown mode");
    }

    double avg_power = 0.0;
    double max_abs = 0.0;
    for (const auto& s : rx_sig) {
        avg_power += std::norm(s);
        max_abs = std::max(max_abs, std::abs(s));
    }
    avg_power /= std::max<size_t>(1, rx_sig.size());

    std::cout << "[DBG][RX] avg_power = " << avg_power
        << ", max_abs = " << max_abs << "\n";

    VecInt rx_bits = rx.receive(rx_sig);

    std::cout << "TX bits (first 64): " << bits_to_string(one_frame_bits, 64) << "\n";
    std::cout << "RX bits (first 64): " << bits_to_string(rx_bits, 64) << "\n";
    std::cout << "RX bits total: " << rx_bits.size() << "\n";

    const size_t bits_per_frame = one_frame_bits.size();
    const size_t decoded_frames =
        (bits_per_frame > 0) ? (rx_bits.size() / bits_per_frame) : 0;

    size_t total_compared_bits = 0;
    size_t total_bit_errors = 0;

    for (size_t i = 0; i < decoded_frames; ++i) {
        VecInt rx_frame(
            rx_bits.begin() + i * bits_per_frame,
            rx_bits.begin() + (i + 1) * bits_per_frame
        );

        size_t frame_errors = 0;
        compute_ber(one_frame_bits, rx_frame, frame_errors);

        total_bit_errors += frame_errors;
        total_compared_bits += bits_per_frame;

        std::cout << "[BER] Frame " << (i + 1)
            << ": errors = " << frame_errors
            << " / " << bits_per_frame
            << ", BER = " << std::setprecision(6)
            << ((double)frame_errors / (double)bits_per_frame)
            << "\n";
    }

    double total_ber = (total_compared_bits > 0)
        ? (double)total_bit_errors / (double)total_compared_bits
        : 0.0;

    std::cout << "Compared bits: " << total_compared_bits << "\n";
    std::cout << "Bit errors: " << total_bit_errors << "\n";
    std::cout << "BER = " << std::setprecision(6) << total_ber << "\n";

    TestResult tr;
    tr.total_compared_bits = total_compared_bits;
    tr.total_bit_errors = total_bit_errors;
    tr.decoded_frames = decoded_frames;
    tr.total_ber = total_ber;
    return tr;
}

int main(int argc, char* argv[]) {
    try {
        RunMode mode = RunMode::AWGN;
        double snr_db = 0.0;
        int frames = 200;
        double fc = 2.45e9;
        ModulationType mod = ModulationType::BPSK;

        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];

            if (arg == "--mode" && i + 1 < argc) {
                mode = parse_mode(argv[++i]);
            }
            else if (arg == "--snr" && i + 1 < argc) {
                snr_db = std::stod(argv[++i]);
            }
            else if (arg == "--frames" && i + 1 < argc) {
                frames = std::stoi(argv[++i]);
            }
            else if (arg == "--fc" && i + 1 < argc) {
                fc = std::stod(argv[++i]);
            }
            else if (arg == "--mod" && i + 1 < argc) {
                mod = parse_modulation(argv[++i]);
            }
            else {
                throw std::runtime_error("Unknown argument: " + arg);
            }
        }

        TestResult tr = run_one_test(mode, snr_db, frames, fc, mod);

        std::cout << "[TEST SUMMARY] decoded_frames = " << tr.decoded_frames
            << ", compared_bits = " << tr.total_compared_bits
            << ", bit_errors = " << tr.total_bit_errors
            << ", BER = " << std::setprecision(6) << tr.total_ber
            << "\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "[EXCEPTION] " << e.what() << std::endl;
        return -1;
    }
    catch (...) {
        std::cerr << "[EXCEPTION] unknown exception\n";
        return -1;
    }
}