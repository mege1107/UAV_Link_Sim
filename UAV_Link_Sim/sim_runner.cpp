#include "sim_runner.h"
#include "ofdm_link.h"
#include <random>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <ctime>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <cmath>

#include "transmitter.h"
#include "receiver.h"
#include "channel.h"

//#define WITH_UHD
#ifdef WITH_UHD
#include "usrp_driver.h"
#endif

static constexpr bool SHOW_PURE_MSK_ONLY = true;

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

static std::vector<unsigned char> bits_to_bytes(const VecInt& bits)
{
    std::vector<unsigned char> bytes;

    if (bits.empty()) return bytes;

    const size_t nbytes = bits.size() / 8;
    bytes.reserve(nbytes);

    for (size_t i = 0; i < nbytes; ++i)
    {
        unsigned char b = 0;

        for (size_t j = 0; j < 8; ++j)
        {
            const int raw = bits[i * 8 + j];
            const int bit = raw & 1;
            b |= static_cast<unsigned char>(bit << (7 - j));
        }

        bytes.push_back(b);
    }

    return bytes;
}

static unsigned short read_u16_be(const std::vector<unsigned char>& data, size_t pos)
{
    return static_cast<unsigned short>(
        (static_cast<unsigned short>(data[pos]) << 8) |
        static_cast<unsigned short>(data[pos + 1])
        );
}

static unsigned long long read_u64_be(const std::vector<unsigned char>& data, size_t pos)
{
    unsigned long long v = 0;
    for (int i = 0; i < 8; ++i)
    {
        v = (v << 8) | static_cast<unsigned long long>(data[pos + i]);
    }
    return v;
}

static bool recover_file_from_bits(
    const VecInt& rx_bits,
    const std::string& output_file_path,
    std::string& recovered_filename
)
{
    std::vector<unsigned char> bytes = bits_to_bytes(rx_bits);

    std::cout << "[FILE][RX] total bits = " << rx_bits.size() << "\n";
    std::cout << "[FILE][RX] total bytes = " << bytes.size() << "\n";

    std::cout << "[FILE][RX] first bytes = ";
    for (size_t i = 0; i < std::min<size_t>(32, bytes.size()); ++i) {
        std::cout << (int)bytes[i] << " ";
    }
    std::cout << "\n";

    if (bytes.size() < 16) {
        std::cout << "[FILE][RX] FAIL: bytes.size() < 16\n";
        return false;
    }

    std::cout << "[FILE][RX] magic chars = "
        << (char)bytes[0]
        << (char)bytes[1]
        << (char)bytes[2]
        << (char)bytes[3] << "\n";

    if (!(bytes[0] == 'U' && bytes[1] == 'A' && bytes[2] == 'V' && bytes[3] == 'F')) {
        std::cout << "[FILE][RX] FAIL: magic mismatch\n";
        return false;
    }

    const unsigned char version = bytes[4];
    const unsigned char file_type = bytes[5];
    const unsigned short filename_len = read_u16_be(bytes, 6);
    const unsigned long long file_size = read_u64_be(bytes, 8);

    std::cout << "[FILE][RX] version = " << (int)version << "\n";
    std::cout << "[FILE][RX] file_type = " << (int)file_type << "\n";
    std::cout << "[FILE][RX] filename_len = " << filename_len << "\n";
    std::cout << "[FILE][RX] file_size = " << file_size << "\n";

    if (version != 1) {
        std::cout << "[FILE][RX] FAIL: version != 1\n";
        return false;
    }

    const size_t header_size = 16ull + static_cast<size_t>(filename_len);

    std::cout << "[FILE][RX] header_size = " << header_size << "\n";

    if (bytes.size() < header_size) {
        std::cout << "[FILE][RX] FAIL: bytes.size() < header_size\n";
        return false;
    }

    if (bytes.size() < header_size + static_cast<size_t>(file_size)) {
        std::cout << "[FILE][RX] FAIL: bytes.size() < header_size + file_size\n";
        return false;
    }

    recovered_filename.assign(
        reinterpret_cast<const char*>(&bytes[16]),
        reinterpret_cast<const char*>(&bytes[16 + filename_len])
    );

    std::cout << "[FILE][RX] recovered filename = " << recovered_filename << "\n";

    std::ofstream ofs(output_file_path, std::ios::binary);
    if (!ofs) {
        std::cout << "[FILE][RX] FAIL: cannot open output path = " << output_file_path << "\n";
        throw std::runtime_error("Cannot open output file for writing: " + output_file_path);
    }

    if (file_size > 0) {
        ofs.write(
            reinterpret_cast<const char*>(&bytes[header_size]),
            static_cast<std::streamsize>(file_size)
        );
    }

    std::cout << "[FILE][RX] file saved OK: " << output_file_path << "\n";
    return true;
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
    case ModulationType::QAM:  return "QAM";
    case ModulationType::OOK:  return "OOK";
    case ModulationType::FSK:  return "FSK";
    case ModulationType::FM:   return "FM";
    case ModulationType::MSK:  return "MSK";
    default: return "UNKNOWN";
    }
}

ModulationType parse_modulation(const std::string& s)
{
    std::string t = s;
    std::transform(t.begin(), t.end(), t.begin(), ::tolower);

    if (t == "bpsk") return ModulationType::BPSK;
    if (t == "qpsk") return ModulationType::QPSK;
    if (t == "qam")  return ModulationType::QAM;
    if (t == "ook")  return ModulationType::OOK;
    if (t == "fsk")  return ModulationType::FSK;
    if (t == "fm")   return ModulationType::FM;
    if (t == "msk")  return ModulationType::MSK;

    throw std::runtime_error("Unknown modulation");
}

RunMode parse_mode(const std::string& s)
{
    std::string t = s;
    std::transform(t.begin(), t.end(), t.begin(), ::tolower);

    if (t == "loopback") return RunMode::LOOPBACK;
    if (t == "awgn")     return RunMode::AWGN;
    if (t == "usrp")     return RunMode::USRP;

    throw std::runtime_error("Unknown mode");
}

static std::vector<std::complex<double>> compute_dft(
    const std::vector<std::complex<double>>& x)
{
    const size_t N = x.size();
    std::vector<std::complex<double>> X(N);

    for (size_t k = 0; k < N; ++k) {
        std::complex<double> sum = 0.0;
        for (size_t n = 0; n < N; ++n) {
            double angle = -2.0 * M_PI * static_cast<double>(k) * static_cast<double>(n) / static_cast<double>(N);
            sum += x[n] * std::complex<double>(std::cos(angle), std::sin(angle));
        }
        X[k] = sum;
    }
    return X;
}

static size_t get_center_start(size_t total_len, size_t want_len)
{
    if (want_len >= total_len) {
        return 0;
    }
    return (total_len - want_len) / 2;
}

static VecComplex extract_center_segment(const VecComplex& sig, size_t want_len)
{
    if (sig.empty()) {
        return {};
    }

    const size_t takeN = std::min(want_len, sig.size());
    const size_t start = get_center_start(sig.size(), takeN);

    return VecComplex(sig.begin() + static_cast<long long>(start),
        sig.begin() + static_cast<long long>(start + takeN));
}

static void build_waveform_from_signal(
    const VecComplex& sig_mid,
    ModulationType modulation,
    std::vector<double>& out_waveform,
    WaveformType& out_type)
{
    out_waveform.clear();

    if (sig_mid.empty()) {
        out_type = WaveformType::REAL;
        return;
    }

    bool useEnvelope = 0;
    //bool useEnvelope = (modulation == ModulationType::FM || modulation == ModulationType::MSK);
    out_type = useEnvelope ? WaveformType::ENVELOPE : WaveformType::REAL;

    out_waveform.reserve(sig_mid.size());

    if (useEnvelope)
    {
        std::vector<double> env;
        env.reserve(sig_mid.size());

        for (size_t i = 0; i < sig_mid.size(); ++i) {
            env.push_back(std::abs(sig_mid[i]));
        }

        const size_t win = 64;

        for (size_t i = 0; i < env.size(); ++i) {
            const size_t l = (i > win / 2) ? (i - win / 2) : 0;
            const size_t r = std::min(env.size(), i + win / 2 + 1);

            double sum = 0.0;
            for (size_t k = l; k < r; ++k) {
                sum += env[k];
            }

            out_waveform.push_back(sum / static_cast<double>(r - l));
        }
    }
    else
    {
        for (size_t i = 0; i < sig_mid.size(); ++i) {
            out_waveform.push_back(sig_mid[i].real());
        }
    }
}

static void build_spectrum_from_signal(
    const VecComplex& sig_mid,
    double fs,
    std::vector<double>& out_freq,
    std::vector<double>& out_mag)
{
    out_freq.clear();
    out_mag.clear();

    if (sig_mid.empty() || fs <= 0.0) {
        return;
    }

    const size_t N = sig_mid.size();
    if (N < 2) return;

    std::vector<std::complex<double>> x(N);

    for (size_t n = 0; n < N; ++n) {
        double w = 0.5 - 0.5 * std::cos(2.0 * M_PI * n / (N - 1));
        x[n] = sig_mid[n] * w;
    }

    std::vector<std::complex<double>> X(N);

    for (size_t k = 0; k < N; ++k) {
        std::complex<double> sum = 0.0;
        for (size_t n = 0; n < N; ++n) {
            double angle = -2.0 * M_PI * k * n / N;
            sum += x[n] * std::complex<double>(std::cos(angle), std::sin(angle));
        }
        X[k] = sum;
    }

    out_freq.reserve(N);
    out_mag.reserve(N);

    for (size_t k = 0; k < N; ++k) {
        size_t idx = (k + N / 2) % N;
        double f = (static_cast<double>(k) - static_cast<double>(N) / 2.0)
            * fs / static_cast<double>(N);

        double mag = std::abs(X[idx]) / static_cast<double>(N);
        double mag_db = 20.0 * std::log10(mag + 1e-12);

        out_freq.push_back(f);
        out_mag.push_back(mag_db);
    }
}

static void build_spectrogram(
    const VecComplex& sig_mid,
    double fs,
    TestResult& tr)
{
    tr.spectrogram_data.clear();
    tr.spectrogram_width = 0;
    tr.spectrogram_height = 0;
    tr.spectrogram_time_span = 0.0;
    tr.spectrogram_freq_min = -fs / 2.0;
    tr.spectrogram_freq_max = fs / 2.0;

    if (sig_mid.empty() || fs <= 0.0) {
        return;
    }

    const int fftSize = 256;
    const int hopSize = 64;
    const int windowSize = 256;

    if (sig_mid.size() < static_cast<size_t>(windowSize)) {
        return;
    }

    const int frameCount =
        1 + static_cast<int>((sig_mid.size() - static_cast<size_t>(windowSize)) / static_cast<size_t>(hopSize));
    const int freqBins = fftSize;

    tr.spectrogram_width = frameCount;
    tr.spectrogram_height = freqBins;
    tr.spectrogram_time_span = static_cast<double>(sig_mid.size()) / fs;
    tr.spectrogram_freq_min = -fs / 2.0;
    tr.spectrogram_freq_max = fs / 2.0;
    tr.spectrogram_data.assign(static_cast<size_t>(frameCount * freqBins), -120.0);

    double globalMaxDb = -1e100;

    for (int frame = 0; frame < frameCount; ++frame)
    {
        const int start = frame * hopSize;

        std::vector<std::complex<double>> x(fftSize, 0.0);

        for (int n = 0; n < windowSize; ++n) {
            double w = 0.5 - 0.5 * std::cos(2.0 * M_PI * n / (windowSize - 1));
            x[n] = sig_mid[static_cast<size_t>(start + n)] * w;
        }

        auto X = compute_dft(x);

        for (int k = 0; k < freqBins; ++k)
        {
            const int idx = (k + fftSize / 2) % fftSize;
            const double mag = std::abs(X[static_cast<size_t>(idx)]) / static_cast<double>(fftSize);
            const double db = 20.0 * std::log10(mag + 1e-12);

            tr.spectrogram_data[static_cast<size_t>(frame * freqBins + k)] = db;
            if (db > globalMaxDb) {
                globalMaxDb = db;
            }
        }
    }

    const double floorDb = globalMaxDb - 60.0;

    for (double& v : tr.spectrogram_data) {
        if (v < floorDb) v = floorDb;
        if (v > globalMaxDb) v = globalMaxDb;

        double norm = (v - floorDb) / (globalMaxDb - floorDb + 1e-12);
        v = std::clamp(norm, 0.0, 1.0);
    }
}

static void build_constellation_result(
    const VecComplex& pts,
    TestResult& tr)
{
    tr.constellation_i.clear();
    tr.constellation_q.clear();

    const size_t N = std::min<size_t>(pts.size(), 600);

    tr.constellation_i.reserve(N);
    tr.constellation_q.reserve(N);

    for (size_t i = 0; i < N; ++i) {
        tr.constellation_i.push_back(pts[i].real());
        tr.constellation_q.push_back(pts[i].imag());
    }
}

TestResult run_one_test(
    RunMode mode,
    double awgn_snr_db,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern
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
    cfg.Rb = info_rate_bps;
    cfg.hop_pattern = hop_pattern;
    cfg.connect = (mode == RunMode::USRP);

    cfg.source_mode = SourceMode::RandomBits;
    cfg.input_file_path.clear();
    cfg.file_bits.clear();
    cfg.file_bit_offset = 0;

    Transmitter tx(cfg);

    VecComplex one_frame_sig = tx.generateTransmitSignal();
    VecInt one_frame_bits = tx.getLastSourceBits();

    cfg.fs = tx.getFS();

    Receiver rx(cfg);

    log << "[MODE] " << mode_to_string(mode) << "\n";
    log << "[MOD] " << modulation_to_string(modulation) << "\n";
    log << "[SRC] RANDOM_BITS\n";
    log << "[CENTER FREQ] " << center_freq_hz << " Hz\n";
    log << "[INFO RATE] " << info_rate_bps << " bps\n";
    log << "[HOP PATTERN] " << hop_pattern << "\n";

    VecComplex tx_burst;

    for (int i = 0; i < tx_repeat_frames; ++i) {
        tx_burst.insert(tx_burst.end(), one_frame_sig.begin(), one_frame_sig.end());
    }

    VecComplex rx_sig;

    if (mode == RunMode::LOOPBACK)
    {
        rx_sig = tx_burst;
    }
    else if (mode == RunMode::AWGN)
    {
        rx_sig = Channel::awgn(tx_burst, awgn_snr_db);
    }
    else if (mode == RunMode::USRP)
    {
#ifdef WITH_UHD
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
#else
        throw std::runtime_error("USRP mode requested, but WITH_UHD is not enabled.");
#endif
    }
    else
    {
        throw std::runtime_error("Unknown run mode.");
    }

    if (rx_sig.empty()) {
        throw std::runtime_error("rx_sig is empty before receiver processing.");
    }

    VecInt rx_bits = rx.receive(rx_sig);

    TestResult tr;
    build_constellation_result(rx.getLastConstellationPoints(), tr);

    const size_t bits_per_frame = one_frame_bits.size();
    const size_t decoded_frames = bits_per_frame > 0 ? (rx_bits.size() / bits_per_frame) : 0;

    size_t total_compared_bits = 0;
    size_t total_bit_errors = 0;

    for (size_t i = 0; i < decoded_frames; ++i)
    {
        VecInt rx_frame(
            rx_bits.begin() + static_cast<long long>(i * bits_per_frame),
            rx_bits.begin() + static_cast<long long>((i + 1) * bits_per_frame)
        );

        size_t frame_errors = 0;
        compute_ber(one_frame_bits, rx_frame, frame_errors);

        total_bit_errors += frame_errors;
        total_compared_bits += bits_per_frame;
    }

    tr.total_bit_errors = total_bit_errors;
    tr.total_compared_bits = total_compared_bits;
    tr.decoded_frames = decoded_frames;
    tr.total_ber =
        total_compared_bits ?
        static_cast<double>(total_bit_errors) / static_cast<double>(total_compared_bits) :
        0.0;

    {
        const VecComplex& tx_sig_full = tx.getLastPureModulatedSignal();

        if (tx_sig_full.empty()) {
            throw std::runtime_error("Pure modulated signal is empty");
        }

        const VecComplex tx_sig_wave_mid = extract_center_segment(tx_sig_full, 4000);
        const VecComplex tx_sig_spec_mid = extract_center_segment(tx_sig_full, 1024);
        const VecComplex tx_sig_tf_mid = extract_center_segment(tx_sig_full, 4096);

        build_waveform_from_signal(
            tx_sig_wave_mid,
            modulation,
            tr.waveform,
            tr.waveform_type
        );

        build_spectrum_from_signal(
            tx_sig_spec_mid,
            cfg.fs,
            tr.spectrum_freq,
            tr.spectrum_mag
        );

        build_spectrogram(tx_sig_tf_mid, cfg.fs, tr);

        log << "[TX MID SEGMENT] waveform = " << tx_sig_wave_mid.size()
            << ", spectrum = " << tx_sig_spec_mid.size()
            << ", spectrogram = " << tx_sig_tf_mid.size() << "\n";
        log << "[SPECTROGRAM] width = " << tr.spectrogram_width
            << ", height = " << tr.spectrogram_height << "\n";
    }

    {
        const VecComplex rx_sig_wave_mid = extract_center_segment(rx_sig, 4000);
        const VecComplex rx_sig_spec_mid = extract_center_segment(rx_sig, 1024);

        build_waveform_from_signal(
            rx_sig_wave_mid,
            modulation,
            tr.rx_waveform,
            tr.rx_waveform_type
        );

        build_spectrum_from_signal(
            rx_sig_spec_mid,
            cfg.fs,
            tr.rx_spectrum_freq,
            tr.rx_spectrum_mag
        );

        log << "[RX MID SEGMENT] waveform = " << rx_sig_wave_mid.size()
            << ", spectrum = " << rx_sig_spec_mid.size() << "\n";
    }

    log << "[CONSTELLATION POINTS] " << tr.constellation_i.size() << "\n";
    log << "[RX BITS] " << rx_bits.size() << "\n";
    log << "[DECODED FRAMES] " << decoded_frames << "\n";
    log << "BER = " << tr.total_ber << "\n";

    tr.log_text = log.str();
    return tr;
}

TestResult run_file_transfer_test(
    RunMode mode,
    double awgn_snr_db,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern,
    const std::string& input_file_path,
    const std::string& output_file_path
)
{
    std::ostringstream log;

    if (input_file_path.empty()) {
        throw std::runtime_error("input_file_path is empty");
    }

    if (output_file_path.empty()) {
        throw std::runtime_error("output_file_path is empty");
    }

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
    cfg.Rb = info_rate_bps;
    cfg.hop_pattern = hop_pattern;
    cfg.connect = (mode == RunMode::USRP);

    cfg.source_mode = SourceMode::FileBits;
    cfg.input_file_path = input_file_path;
    cfg.file_bits.clear();
    cfg.file_bit_offset = 0;

    Transmitter tx(cfg);

    VecComplex first_frame_sig = tx.generateTransmitSignal();
    VecInt first_frame_bits = tx.getLastSourceBits();

    cfg.fs = tx.getFS();

    const TransmitterConfig& tx_cfg = tx.getConfig();
    const size_t total_file_bits = tx_cfg.file_bits.size();
    const size_t bits_per_frame = static_cast<size_t>(cfg.frame_bit * cfg.n);
    const size_t total_frames =
        bits_per_frame > 0 ?
        (total_file_bits + bits_per_frame - 1) / bits_per_frame :
        0;

    Receiver rx(cfg);

    log << "[MODE] " << mode_to_string(mode) << "\n";
    log << "[MOD] " << modulation_to_string(modulation) << "\n";
    log << "[SRC] FILE_BITS\n";
    log << "[CENTER FREQ] " << center_freq_hz << " Hz\n";
    log << "[INFO RATE] " << info_rate_bps << " bps\n";
    log << "[HOP PATTERN] " << hop_pattern << "\n";
    log << "[INPUT FILE] " << input_file_path << "\n";
    log << "[OUTPUT FILE] " << output_file_path << "\n";
    log << "[PACKED FILE BITS] " << total_file_bits << "\n";
    log << "[BITS PER FRAME] " << bits_per_frame << "\n";
    log << "[TOTAL FRAMES] " << total_frames << "\n";

    VecComplex tx_burst;
    VecInt tx_all_bits;

    tx_burst.insert(tx_burst.end(), first_frame_sig.begin(), first_frame_sig.end());
    tx_all_bits.insert(tx_all_bits.end(), first_frame_bits.begin(), first_frame_bits.end());

    for (size_t i = 1; i < total_frames; ++i)
    {
        VecComplex frame_sig = tx.generateTransmitSignal();
        VecInt frame_bits = tx.getLastSourceBits();

        tx_burst.insert(tx_burst.end(), frame_sig.begin(), frame_sig.end());
        tx_all_bits.insert(tx_all_bits.end(), frame_bits.begin(), frame_bits.end());
    }

    VecComplex rx_sig;

    if (mode == RunMode::LOOPBACK)
    {
        rx_sig = tx_burst;
    }
    else if (mode == RunMode::AWGN)
    {
        rx_sig = Channel::awgn(tx_burst, awgn_snr_db);
    }
    else if (mode == RunMode::USRP)
    {
#ifdef WITH_UHD
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
#else
        throw std::runtime_error("USRP mode requested, but WITH_UHD is not enabled.");
#endif
    }
    else
    {
        throw std::runtime_error("Unknown run mode.");
    }

    if (rx_sig.empty()) {
        throw std::runtime_error("rx_sig is empty before receiver processing.");
    }

    VecInt rx_bits = rx.receive(rx_sig);

    TestResult tr;
    build_constellation_result(rx.getLastConstellationPoints(), tr);

    std::cout << "[DBG][TX bits first 80] ";
    for (size_t i = 0; i < std::min<size_t>(80, tx_all_bits.size()); ++i) {
        std::cout << tx_all_bits[i];
    }
    std::cout << "\n";

    std::cout << "[DBG][RX bits first 80] ";
    for (size_t i = 0; i < std::min<size_t>(80, rx_bits.size()); ++i) {
        std::cout << rx_bits[i];
    }
    std::cout << "\n";

    std::cout << "[DBG][BIT ERR IDX < 80] ";
    for (size_t i = 0; i < std::min<size_t>(80, std::min(tx_all_bits.size(), rx_bits.size())); ++i) {
        if (tx_all_bits[i] != rx_bits[i]) {
            std::cout << i << " ";
        }
    }
    std::cout << "\n";

    size_t bit_errors = 0;
    size_t total_compared_bits = std::min(tx_all_bits.size(), rx_bits.size());
    double ber = compute_ber(tx_all_bits, rx_bits, bit_errors);

    tr.total_compared_bits = total_compared_bits;
    tr.total_bit_errors = bit_errors;
    tr.decoded_frames = bits_per_frame ? (rx_bits.size() / bits_per_frame) : 0;
    tr.total_ber = ber;

    std::string recovered_filename;
    std::vector<unsigned char> tx_bytes_dbg = bits_to_bytes(tx_all_bits);
    std::vector<unsigned char> rx_bytes_dbg = bits_to_bytes(rx_bits);

    std::cout << "[DBG][TX bytes from tx_all_bits] ";
    for (size_t i = 0; i < std::min<size_t>(32, tx_bytes_dbg.size()); ++i) {
        std::cout << static_cast<int>(tx_bytes_dbg[i]) << " ";
    }
    std::cout << "\n";

    std::cout << "[DBG][RX bytes from rx_bits] ";
    for (size_t i = 0; i < std::min<size_t>(32, rx_bytes_dbg.size()); ++i) {
        std::cout << static_cast<int>(rx_bytes_dbg[i]) << " ";
    }
    std::cout << "\n";

    bool save_ok = recover_file_from_bits(rx_bits, output_file_path, recovered_filename);

    tr.file_saved = save_ok;
    tr.saved_file_path = save_ok ? output_file_path : "";

    log << "[CONSTELLATION POINTS] " << tr.constellation_i.size() << "\n";
    log << "[RX BITS] " << rx_bits.size() << "\n";
    log << "[DECODED FRAMES] " << tr.decoded_frames << "\n";
    log << "BER = " << tr.total_ber << "\n";
    log << "FILE RECOVERED = " << (save_ok ? "YES" : "NO") << "\n";

    if (save_ok) {
        log << "RECOVERED ORIGINAL NAME = " << recovered_filename << "\n";
        log << "SAVED TO = " << output_file_path << "\n";
    }

    {
        const VecComplex& tx_sig_full = tx.getLastPureModulatedSignal();

        if (!tx_sig_full.empty())
        {
            const VecComplex tx_sig_wave_mid = extract_center_segment(tx_sig_full, 4000);
            const VecComplex tx_sig_spec_mid = extract_center_segment(tx_sig_full, 1024);
            const VecComplex tx_sig_tf_mid = extract_center_segment(tx_sig_full, 4096);

            build_waveform_from_signal(
                tx_sig_wave_mid,
                modulation,
                tr.waveform,
                tr.waveform_type
            );

            build_spectrum_from_signal(
                tx_sig_spec_mid,
                cfg.fs,
                tr.spectrum_freq,
                tr.spectrum_mag
            );

            build_spectrogram(tx_sig_tf_mid, cfg.fs, tr);
        }
    }

    {
        const VecComplex rx_sig_wave_mid = extract_center_segment(rx_sig, 4000);
        const VecComplex rx_sig_spec_mid = extract_center_segment(rx_sig, 1024);

        build_waveform_from_signal(
            rx_sig_wave_mid,
            modulation,
            tr.rx_waveform,
            tr.rx_waveform_type
        );

        build_spectrum_from_signal(
            rx_sig_spec_mid,
            cfg.fs,
            tr.rx_spectrum_freq,
            tr.rx_spectrum_mag
        );
    }

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
    log << "[SWEEP] Source = RANDOM_BITS\n";
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
            modulation,
            50000.0,
            1
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


#include "ofdm_link.h"
#include <random>

TestResult run_ofdm_random_bit_test(double awgn_snr_db)
{
    std::ostringstream log;

    // ===== OFDM 参数 =====
    OFDMConfig cfg;
    cfg.N_fft = 256;
    cfg.N_cp = 32;
    cfg.N_sc = 128;
    cfg.Nd = 10;
    cfg.N_frm = 1;
    cfg.N_zeros = 256;
    cfg.P_f_inter = 6;
    cfg.M = 4;            // 4=QPSK, 16=16QAM
    cfg.L = 7;
    cfg.tblen = 32;
    cfg.delta_f = 15e3;

    constexpr int kNumFrames = 200;

    TestResult tr;

    // ===== 每帧可承载的原始 bit 数 =====
    auto dataPos = OFDMUtils::dataPositions(cfg.N_sc, cfg.P_f_inter);
    const int dataRow = static_cast<int>(dataPos.size());
    const int codedBitsPerFrame = dataRow * cfg.Nd * static_cast<int>(std::log2(cfg.M));
    const int uncodedBitsPerFrame = codedBitsPerFrame / 2; // 1/2 卷积码

    std::mt19937 rng(1);
    std::uniform_int_distribution<int> dist01(0, 1);

    size_t total_bit_errors = 0;
    size_t total_compared_bits = 0;

    // 只保留最后一帧的图形数据，避免 200 帧全塞进去太大
    VecComplex last_eqSyms;
    VecComplex last_tx_sig;
    VecComplex last_rx_sig;

    for (int frm = 0; frm < kNumFrames; ++frm)
    {
        // ===== 生成随机 bit =====
        VecInt tx_bits;
        tx_bits.reserve(uncodedBitsPerFrame);
        for (int i = 0; i < uncodedBitsPerFrame; ++i) {
            tx_bits.push_back(dist01(rng));
        }

        // ===== 发端：编码 + QAM =====
        VecInt codedBits = OFDMUtils::convEncode_171_133(tx_bits);
        VecComplex qamSyms = OFDMUtils::qamMod(codedBits, cfg.M);

        // ===== 资源映射 =====
        auto pilotPos = OFDMUtils::pilotPositions(cfg.N_sc, cfg.P_f_inter);
        auto dataPos2 = OFDMUtils::dataPositions(cfg.N_sc, cfg.P_f_inter);

        const int pilotNum = static_cast<int>(pilotPos.size());
        std::vector<std::vector<Complex>> grid(cfg.N_sc, std::vector<Complex>(cfg.Nd, Complex(0.0, 0.0)));

        // 与发端一致的 pilot 序列
        std::vector<std::vector<Complex>> pilotSeq(pilotNum, std::vector<Complex>(cfg.Nd, Complex(1.0, 0.0)));
        {
            uint32_t s = 1u;
            auto rbit = [&]() {
                s = 1664525u * s + 1013904223u;
                return (s >> 31) & 1u;
                };

            for (int r = 0; r < pilotNum; ++r) {
                for (int c = 0; c < cfg.Nd; ++c) {
                    pilotSeq[r][c] = Complex(rbit() ? 1.0 : -1.0, 0.0);
                    grid[pilotPos[r]][c] = pilotSeq[r][c];
                }
            }
        }

        {
            size_t idx = 0;
            for (int c = 0; c < cfg.Nd; ++c) {
                for (size_t r = 0; r < dataPos2.size(); ++r) {
                    grid[dataPos2[r]][c] = (idx < qamSyms.size()) ? qamSyms[idx++] : Complex(0.0, 0.0);
                }
            }
        }

        // ===== OFDM 调制 =====
        VecComplex tx_data;
        for (int col = 0; col < cfg.Nd; ++col)
        {
            VecComplex X(cfg.N_fft, Complex(0.0, 0.0));

            // bin0=DC, 1..N_sc 放数据
            for (int k = 0; k < cfg.N_sc; ++k) {
                X[1 + k] = grid[k][col];
            }

            VecComplex x = OFDMUtils::ifft(X);

            // CP
            for (int i = cfg.N_fft - cfg.N_cp; i < cfg.N_fft; ++i) {
                tx_data.push_back(x[i]);
            }
            tx_data.insert(tx_data.end(), x.begin(), x.end());
        }

        // ===== 加前导零 + PSS =====
        VecComplex tx_sig;
        tx_sig.insert(tx_sig.end(), cfg.N_zeros, Complex(0.0, 0.0));
        VecComplex pss = OFDMUtils::makePSS(cfg.N_fft);
        tx_sig.insert(tx_sig.end(), pss.begin(), pss.end());
        tx_sig.insert(tx_sig.end(), tx_data.begin(), tx_data.end());

        // ===== AWGN =====
        VecComplex rx_sig = Channel::awgn(tx_sig, awgn_snr_db);

        // ===== 接收同步 =====
        int pssPos = -1;
        {
            double best = -1.0;
            for (size_t d = 0; d + pss.size() <= rx_sig.size(); ++d) {
                Complex acc(0.0, 0.0);
                for (size_t k = 0; k < pss.size(); ++k) {
                    acc += std::conj(pss[k]) * rx_sig[d + k];
                }
                double v = std::abs(acc);
                if (v > best) {
                    best = v;
                    pssPos = static_cast<int>(d);
                }
            }
        }

        if (pssPos < 0) {
            throw std::runtime_error("OFDM PSS detect failed.");
        }

        // ===== 提取 OFDM 符号 =====
        std::vector<VecComplex> symbolsNoCP;
        {
            // 如果后面高 SNR 仍有异常误码，可试试改成 pssPos + pss.size()
            int pos0 = pssPos + static_cast<int>(pss.size()) - 1;

            for (int i = 0; i < cfg.Nd; ++i) {
                int cpStart = pos0 + i * (cfg.N_cp + cfg.N_fft);
                int dataStart = cpStart + cfg.N_cp;
                int dataEnd = dataStart + cfg.N_fft;

                if (dataStart < 0 || dataEnd > static_cast<int>(rx_sig.size())) {
                    throw std::runtime_error("OFDM symbol extraction failed.");
                }

                symbolsNoCP.emplace_back(
                    rx_sig.begin() + dataStart,
                    rx_sig.begin() + dataEnd
                );
            }
        }

        // ===== FFT + 导频估计 + 均衡 =====
        VecComplex eqSyms;

        for (int col = 0; col < cfg.Nd; ++col)
        {
            VecComplex Y = OFDMUtils::fft(symbolsNoCP[col]);

            VecComplex used(cfg.N_sc);
            for (int k = 0; k < cfg.N_sc; ++k) {
                used[k] = Y[1 + k];
            }

            VecComplex rxPilot(pilotNum), txPilot(pilotNum);
            for (int i = 0; i < pilotNum; ++i) {
                rxPilot[i] = used[pilotPos[i]];
                txPilot[i] = pilotSeq[i][col];
            }

            auto H = OFDMUtils::linearInterpChannel(pilotPos, rxPilot, txPilot, dataPos2);

            for (size_t i = 0; i < dataPos2.size(); ++i) {
                Complex h = H[i];
                if (std::abs(h) < 1e-12) h = Complex(1.0, 0.0);
                eqSyms.push_back(used[dataPos2[i]] / h);
            }
        }

        // ===== 解调 + 译码 =====
        VecInt rx_coded_bits = OFDMUtils::qamDemod(eqSyms, cfg.M);
        VecInt rx_bits = OFDMUtils::viterbiDecodeHard_171_133(rx_coded_bits, cfg.tblen);

        if (rx_bits.size() > tx_bits.size()) {
            rx_bits.resize(tx_bits.size());
        }

        size_t bit_errors = 0;
        double ber = compute_ber(tx_bits, rx_bits, bit_errors);

        total_bit_errors += bit_errors;
        total_compared_bits += std::min(tx_bits.size(), rx_bits.size());

        // 保存最后一帧用于前端显示
        if (frm == kNumFrames - 1) {
            last_eqSyms = eqSyms;
            last_tx_sig = tx_sig;
            last_rx_sig = rx_sig;
        }

        log << "[Frame " << (frm + 1) << "/" << kNumFrames << "] "
            << "bits=" << std::min(tx_bits.size(), rx_bits.size())
            << ", errors=" << bit_errors
            << ", ber=" << ber << "\n";
    }

    tr.total_bit_errors = total_bit_errors;
    tr.total_compared_bits = total_compared_bits;
    tr.total_ber = (total_compared_bits > 0)
        ? static_cast<double>(total_bit_errors) / static_cast<double>(total_compared_bits)
        : 0.0;
    tr.decoded_frames = kNumFrames;

    // ===== 图形数据：只显示最后一帧 =====
    build_constellation_result(last_eqSyms, tr);

    const VecComplex tx_sig_wave_mid = extract_center_segment(last_tx_sig, 4000);
    const VecComplex tx_sig_spec_mid = extract_center_segment(last_tx_sig, 1024);
    const VecComplex tx_sig_tf_mid = extract_center_segment(last_tx_sig, 4096);

    build_waveform_from_signal(
        tx_sig_wave_mid,
        ModulationType::QPSK,
        tr.waveform,
        tr.waveform_type
    );

    build_spectrum_from_signal(
        tx_sig_spec_mid,
        cfg.sampleRate(),
        tr.spectrum_freq,
        tr.spectrum_mag
    );

    build_spectrogram(tx_sig_tf_mid, cfg.sampleRate(), tr);

    const VecComplex rx_sig_wave_mid = extract_center_segment(last_rx_sig, 4000);
    const VecComplex rx_sig_spec_mid = extract_center_segment(last_rx_sig, 1024);

    build_waveform_from_signal(
        rx_sig_wave_mid,
        ModulationType::QPSK,
        tr.rx_waveform,
        tr.rx_waveform_type
    );

    build_spectrum_from_signal(
        rx_sig_spec_mid,
        cfg.sampleRate(),
        tr.rx_spectrum_freq,
        tr.rx_spectrum_mag
    );

    log << "========== OFDM RANDOM BIT TEST ==========\n";
    log << "[CHANNEL] AWGN\n";
    log << "[Frames] " << kNumFrames << "\n";
    log << "[N_fft] " << cfg.N_fft << "\n";
    log << "[N_cp] " << cfg.N_cp << "\n";
    log << "[N_sc] " << cfg.N_sc << "\n";
    log << "[Nd] " << cfg.Nd << "\n";
    log << "[P_f_inter] " << cfg.P_f_inter << "\n";
    log << "[M] " << cfg.M << "\n";
    log << "[delta_f] " << cfg.delta_f << "\n";
    log << "[fs] " << cfg.sampleRate() << "\n";
    log << "[Total compared bits] " << total_compared_bits << "\n";
    log << "[Total bit errors] " << total_bit_errors << "\n";
    log << "BER = " << tr.total_ber << "\n";

    tr.log_text = log.str();
    return tr;
}

TestResult run_channel_test(
    double snr_db,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern,
    const ChannelConfig& ch_cfg
)
{
    std::ostringstream log;

    // ===== 发射机配置 =====
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
    cfg.Rb = info_rate_bps;
    cfg.hop_pattern = hop_pattern;
    cfg.connect = false;

    cfg.source_mode = SourceMode::RandomBits;

    Transmitter tx(cfg);

    VecComplex one_frame_sig = tx.generateTransmitSignal();
    VecInt one_frame_bits = tx.getLastSourceBits();

    cfg.fs = tx.getFS();

    Receiver rx(cfg);

    // ===== 构造 burst =====
    VecComplex tx_burst;
    for (int i = 0; i < tx_repeat_frames; ++i) {
        tx_burst.insert(tx_burst.end(), one_frame_sig.begin(), one_frame_sig.end());
    }

    // ===== 新信道入口 =====
    // 对正 STO 场景追加尾保护，避免有限长 burst 被前补零后截断误伤最后几帧
    if (ch_cfg.enable_sto && ch_cfg.sto_samp > 0) {
        const size_t tail_guard = static_cast<size_t>(ch_cfg.sto_samp) + one_frame_sig.size();
        tx_burst.insert(tx_burst.end(), tail_guard, Complex(0.0, 0.0));
    }

    ChannelConfig cfg_local = ch_cfg;
    cfg_local.snr_dB = snr_db;

    VecComplex rx_sig = Channel::process(tx_burst, cfg_local, cfg.fs);

    if (rx_sig.empty()) {
        throw std::runtime_error("rx_sig is empty.");
    }

    // ===== 接收 =====
    VecInt rx_bits = rx.receive(rx_sig);

    TestResult tr;
    build_constellation_result(rx.getLastConstellationPoints(), tr);

    // ===== BER =====
    const size_t bits_per_frame = one_frame_bits.size();
    const size_t decoded_frames =
        bits_per_frame > 0 ? (rx_bits.size() / bits_per_frame) : 0;

    size_t total_compared_bits = 0;
    size_t total_bit_errors = 0;

    for (size_t i = 0; i < decoded_frames; ++i)
    {
        VecInt rx_frame(
            rx_bits.begin() + static_cast<long long>(i * bits_per_frame),
            rx_bits.begin() + static_cast<long long>((i + 1) * bits_per_frame)
        );

        size_t frame_errors = 0;
        compute_ber(one_frame_bits, rx_frame, frame_errors);

        total_bit_errors += frame_errors;
        total_compared_bits += bits_per_frame;
    }

    tr.total_bit_errors = total_bit_errors;
    tr.total_compared_bits = total_compared_bits;
    tr.decoded_frames = decoded_frames;

    tr.total_ber =
        total_compared_bits ?
        (double)total_bit_errors / (double)total_compared_bits :
        0.0;

    // ===== 波形/频谱 =====
    {
        const VecComplex& tx_sig_full = tx.getLastPureModulatedSignal();

        const VecComplex tx_mid = extract_center_segment(tx_sig_full, 1024);

        build_waveform_from_signal(tx_mid, modulation,
            tr.waveform, tr.waveform_type);

        build_spectrum_from_signal(tx_mid, cfg.fs,
            tr.spectrum_freq, tr.spectrum_mag);
    }

    {
        const VecComplex rx_mid = extract_center_segment(rx_sig, 1024);

        build_waveform_from_signal(rx_mid, modulation,
            tr.rx_waveform, tr.rx_waveform_type);

        build_spectrum_from_signal(rx_mid, cfg.fs,
            tr.rx_spectrum_freq, tr.rx_spectrum_mag);
    }

    // ===== 日志 =====
    log << "========== CUSTOM CHANNEL TEST ==========\n";
    log << "[MOD] " << modulation_to_string(modulation) << "\n";
    log << "[SNR] " << snr_db << " dB\n";

    log << "[STO] " << (ch_cfg.enable_sto ? "ON" : "OFF")
        << " samp=" << ch_cfg.sto_samp << "\n";

    log << "[CFO] " << (ch_cfg.enable_cfo ? "ON" : "OFF")
        << " Hz=" << ch_cfg.cfo_hz << "\n";

    log << "[SFO] " << (ch_cfg.enable_sfo ? "ON" : "OFF")
        << " ppm=" << ch_cfg.sfo_ppm << "\n";

    log << "[AWGN] " << (ch_cfg.enable_awgn ? "ON" : "OFF") << "\n";

    log << "[DECODED FRAMES] " << decoded_frames << "\n";
    log << "BER = " << tr.total_ber << "\n";

    tr.log_text = log.str();

    return tr;
}
