#include "role_runner.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <iostream>

#include "transmitter.h"
#include "receiver.h"

#define WITH_UHD
#ifdef WITH_UHD
#include "usrp_driver.h"
#endif

namespace
{

    static std::vector<std::complex<double>> compute_dft(
        const std::vector<std::complex<double>>& x)
    {
        const size_t N = x.size();
        std::vector<std::complex<double>> X(N);

        for (size_t k = 0; k < N; ++k) {
            std::complex<double> sum = 0.0;
            for (size_t n = 0; n < N; ++n) {
                double angle = -2.0 * M_PI
                    * static_cast<double>(k)
                    * static_cast<double>(n)
                    / static_cast<double>(N);
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

        return VecComplex(
            sig.begin() + static_cast<long long>(start),
            sig.begin() + static_cast<long long>(start + takeN)
        );
    }

    static void build_waveform_from_signal(
        const VecComplex& sig_mid,
        ModulationType /*modulation*/,
        std::vector<double>& out_waveform,
        WaveformType& out_type)
    {
        out_waveform.clear();

        if (sig_mid.empty()) {
            out_type = WaveformType::REAL;
            return;
        }

        out_type = WaveformType::REAL;
        out_waveform.reserve(sig_mid.size());

        for (const auto& v : sig_mid) {
            out_waveform.push_back(v.real());
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

    static void build_spectrogram_from_signal(
        const VecComplex& sig_mid,
        double fs,
        std::vector<double>& out_data,
        int& out_width,
        int& out_height,
        double& out_time_span,
        double& out_freq_min,
        double& out_freq_max)
    {
        out_data.clear();
        out_width = 0;
        out_height = 0;
        out_time_span = 0.0;
        out_freq_min = -fs / 2.0;
        out_freq_max = fs / 2.0;

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

        out_width = frameCount;
        out_height = freqBins;
        out_time_span = static_cast<double>(sig_mid.size()) / fs;
        out_freq_min = -fs / 2.0;
        out_freq_max = fs / 2.0;
        out_data.assign(static_cast<size_t>(frameCount * freqBins), -120.0);

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

                out_data[static_cast<size_t>(frame * freqBins + k)] = db;
                if (db > globalMaxDb) {
                    globalMaxDb = db;
                }
            }
        }

        const double floorDb = globalMaxDb - 60.0;

        for (double& v : out_data) {
            if (v < floorDb) v = floorDb;
            if (v > globalMaxDb) v = globalMaxDb;

            double norm = (v - floorDb) / (globalMaxDb - floorDb + 1e-12);
            v = std::clamp(norm, 0.0, 1.0);
        }
    }

    template<typename TResult>
    static void build_constellation_result(
        const VecComplex& pts,
        TResult& rr)
    {
        rr.constellation_i.clear();
        rr.constellation_q.clear();

        const size_t N = std::min<size_t>(pts.size(), 600);
        rr.constellation_i.reserve(N);
        rr.constellation_q.reserve(N);

        for (size_t i = 0; i < N; ++i) {
            rr.constellation_i.push_back(pts[i].real());
            rr.constellation_q.push_back(pts[i].imag());
        }
    }

    static TransmitterConfig make_base_config(
        ModulationType modulation,
        double info_rate_bps,
        int hop_pattern,
        bool connect_usrp)
    {
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
        cfg.connect = connect_usrp;

        return cfg;
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

    static double compute_ber(const VecInt& tx, const VecInt& rx, size_t& bit_errors)
    {
        const size_t n = std::min(tx.size(), rx.size());
        bit_errors = 0;

        for (size_t i = 0; i < n; ++i) {
            if ((tx[i] & 1) != (rx[i] & 1)) {
                bit_errors++;
            }
        }

        return n ? static_cast<double>(bit_errors) / static_cast<double>(n) : 0.0;
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
        std::string& recovered_filename)
    {
        std::vector<unsigned char> bytes = bits_to_bytes(rx_bits);

        if (bytes.size() < 16) {
            return false;
        }

        if (!(bytes[0] == 'U' && bytes[1] == 'A' && bytes[2] == 'V' && bytes[3] == 'F')) {
            return false;
        }

        const unsigned char version = bytes[4];
        const unsigned short filename_len = read_u16_be(bytes, 6);
        const unsigned long long file_size = read_u64_be(bytes, 8);

        if (version != 1) {
            return false;
        }

        const size_t header_size = 16ull + static_cast<size_t>(filename_len);

        if (bytes.size() < header_size) {
            return false;
        }

        if (bytes.size() < header_size + static_cast<size_t>(file_size)) {
            return false;
        }

        recovered_filename.assign(
            reinterpret_cast<const char*>(&bytes[16]),
            reinterpret_cast<const char*>(&bytes[16 + filename_len])
        );

        std::ofstream ofs(output_file_path, std::ios::binary);
        if (!ofs) {
            throw std::runtime_error("Cannot open output file for writing: " + output_file_path);
        }

        if (file_size > 0) {
            ofs.write(
                reinterpret_cast<const char*>(&bytes[header_size]),
                static_cast<std::streamsize>(file_size)
            );
        }

        return true;
    }

    static VecComplex capture_rx_samples(
        const std::string& rx_device_args,
        double center_freq_hz,
        double fs,
        size_t rx_need,
        std::ostringstream& log)
    {
#ifdef WITH_UHD
        USRPDriver::Config uc;
        uc.device_args = rx_device_args;
        uc.sample_rate = fs;
        uc.center_freq = center_freq_hz;
        uc.recv_timeout = 0.2;

        // 如果接收端是 X310，后面可以直接开这些
        // uc.rx_subdev = "A:0";
        // uc.clock_source = "internal";
        // uc.time_source  = "internal";

        USRPDriver usrp(uc);
        usrp.init();

        log << "[USRP] RX device args = " << rx_device_args << "\n";
        log << usrp.get_pp_string() << "\n";

        usrp.start_rx_worker(rx_need);
        usrp.wait_rx_worker();

        VecComplex rx_sig = usrp.fetch_rx_buffer();

        log << "[USRP] RX capture complete\n";
        log << "[USRP] RX captured samples = " << rx_sig.size() << "\n";

        return rx_sig;
#else
        (void)rx_device_args;
        (void)center_freq_hz;
        (void)fs;
        (void)rx_need;
        (void)log;
        throw std::runtime_error("RX role requires WITH_UHD enabled.");
#endif
    }

    static void send_tx_samples(
        const std::string& tx_device_args,
        double center_freq_hz,
        double fs,
        const VecComplex& tx_burst,
        std::ostringstream& log)
    {
#ifdef WITH_UHD
        USRPDriver::Config uc;
        uc.device_args = tx_device_args;
        uc.sample_rate = fs;
        uc.center_freq = center_freq_hz;

        // B210 一般不用写 subdev
        // uc.clock_source = "internal";
        // uc.time_source  = "internal";

        USRPDriver usrp(uc);
        usrp.init();

        log << "[USRP] TX device args = " << tx_device_args << "\n";
        log << usrp.get_pp_string() << "\n";

        usrp.send_burst(tx_burst);

        log << "[USRP] TX burst sent successfully\n";
#else
        (void)tx_device_args;
        (void)center_freq_hz;
        (void)fs;
        (void)tx_burst;
        (void)log;
        throw std::runtime_error("TX role requires WITH_UHD enabled.");
#endif
    }

} // namespace

TxOnlyResult run_tx_role_once(
    const std::string& tx_device_args,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern,
    SourceMode source_mode,
    const std::string& input_file_path,
    int tx_repeat_frames)
{
    std::ostringstream log;

    if (tx_repeat_frames <= 0) {
        tx_repeat_frames = 1;
    }

    TransmitterConfig cfg = make_base_config(
        modulation,
        info_rate_bps,
        hop_pattern,
        true
    );

    cfg.source_mode = source_mode;
    cfg.input_file_path = input_file_path;
    cfg.file_bits.clear();
    cfg.file_bit_offset = 0;

    Transmitter tx(cfg);

    VecComplex tx_burst;
    size_t tx_frame_count = 0;

    if (source_mode == SourceMode::RandomBits)
    {
        VecComplex one_frame_sig = tx.generateTransmitSignal();
        if (one_frame_sig.empty()) {
            throw std::runtime_error("Generated TX frame is empty");
        }

        for (int i = 0; i < tx_repeat_frames; ++i) {
            tx_burst.insert(tx_burst.end(), one_frame_sig.begin(), one_frame_sig.end());
        }

        tx_frame_count = static_cast<size_t>(tx_repeat_frames);
    }
    else if (source_mode == SourceMode::FileBits)
    {
        VecComplex first_frame = tx.generateTransmitSignal();
        if (first_frame.empty()) {
            throw std::runtime_error("Generated first TX file frame is empty");
        }

        const TransmitterConfig& tx_cfg = tx.getConfig();
        const size_t total_file_bits = tx_cfg.file_bits.size();
        const size_t bits_per_frame = static_cast<size_t>(tx_cfg.frame_bit * tx_cfg.n);

        if (bits_per_frame == 0) {
            throw std::runtime_error("bits_per_frame == 0");
        }

        const size_t total_frames =
            (total_file_bits + bits_per_frame - 1) / bits_per_frame;

        tx_burst.insert(tx_burst.end(), first_frame.begin(), first_frame.end());

        for (size_t i = 1; i < total_frames; ++i) {
            VecComplex frame_sig = tx.generateTransmitSignal();
            if (frame_sig.empty()) {
                throw std::runtime_error("Generated file TX frame is empty");
            }
            tx_burst.insert(tx_burst.end(), frame_sig.begin(), frame_sig.end());
        }

        tx_frame_count = total_frames;
    }
    else
    {
        throw std::runtime_error("Unsupported source_mode in run_tx_role_once");
    }

    cfg.fs = tx.getFS();

    log << "[ROLE] UAV(TX)\n";
    log << "[MOD] " << modulation_to_string(modulation) << "\n";
    log << "[CENTER FREQ] " << center_freq_hz << " Hz\n";
    log << "[INFO RATE] " << info_rate_bps << " bps\n";
    log << "[HOP PATTERN] " << hop_pattern << "\n";
    log << "[FS] " << cfg.fs << " Hz\n";
    log << "[TX DEVICE ARGS] " << tx_device_args << "\n";
    log << "[SOURCE MODE] " << (source_mode == SourceMode::FileBits ? "FILE_BITS" : "RANDOM_BITS") << "\n";
    log << "[TX REPEAT FRAMES] " << tx_repeat_frames << "\n";

    if (source_mode == SourceMode::FileBits) {
        log << "[INPUT FILE] " << input_file_path << "\n";
    }

    log << "[TX FRAME COUNT] " << tx_frame_count << "\n";
    log << "[TX SAMPLE COUNT] " << tx_burst.size() << "\n";

    send_tx_samples(tx_device_args, center_freq_hz, cfg.fs, tx_burst, log);

    TxOnlyResult tr;
    tr.tx_frame_count = tx_frame_count;
    tr.tx_sample_count = tx_burst.size();
    tr.fs = cfg.fs;

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

            build_spectrogram_from_signal(
                tx_sig_tf_mid,
                cfg.fs,
                tr.spectrogram_data,
                tr.spectrogram_width,
                tr.spectrogram_height,
                tr.spectrogram_time_span,
                tr.spectrogram_freq_min,
                tr.spectrogram_freq_max
            );
        }
    }

    tr.log_text = log.str();
    return tr;
}

RxOnlyResult run_rx_role_once(
    const std::string& rx_device_args,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern,
    int expected_frames)
{
    std::ostringstream log;

    if (expected_frames <= 0) {
        expected_frames = 1;
    }

    TransmitterConfig cfg = make_base_config(
        modulation,
        info_rate_bps,
        hop_pattern,
        true
    );

    cfg.source_mode = SourceMode::RandomBits;
    cfg.input_file_path.clear();
    cfg.file_bits.clear();
    cfg.file_bit_offset = 0;

    Transmitter tx_ref(cfg);
    VecComplex ref_frame = tx_ref.generateTransmitSignal();
    VecInt ref_bits = tx_ref.getLastSourceBits();
    cfg.fs = tx_ref.getFS();

    if (ref_frame.empty()) {
        throw std::runtime_error("Reference frame is empty");
    }

    const size_t single_frame_samples = ref_frame.size();
    const size_t margin_samples = std::max<size_t>(single_frame_samples, 50000);
    const size_t rx_need =
        single_frame_samples * static_cast<size_t>(expected_frames) + margin_samples;

    Receiver rx(cfg);

    log << "[ROLE] GROUND(RX)\n";
    log << "[MOD] " << modulation_to_string(modulation) << "\n";
    log << "[CENTER FREQ] " << center_freq_hz << " Hz\n";
    log << "[INFO RATE] " << info_rate_bps << " bps\n";
    log << "[HOP PATTERN] " << hop_pattern << "\n";
    log << "[FS] " << cfg.fs << " Hz\n";
    log << "[RX DEVICE ARGS] " << rx_device_args << "\n";
    log << "[EXPECTED FRAMES] " << expected_frames << "\n";
    log << "[REF FRAME SAMPLES] " << single_frame_samples << "\n";
    log << "[RX NEED SAMPLES] " << rx_need << "\n";

    VecComplex rx_sig = capture_rx_samples(rx_device_args, center_freq_hz, cfg.fs, rx_need, log);

    if (rx_sig.empty()) {
        throw std::runtime_error("RX signal is empty");
    }

    VecInt rx_bits = rx.receive(rx_sig);

    const size_t bits_per_frame = static_cast<size_t>(cfg.frame_bit * cfg.n);
    const size_t decoded_frames =
        (bits_per_frame > 0) ? (rx_bits.size() / bits_per_frame) : 0;

    size_t total_compared_bits = 0;
    size_t total_bit_errors = 0;

    for (size_t i = 0; i < decoded_frames; ++i)
    {
        VecInt rx_frame(
            rx_bits.begin() + static_cast<long long>(i * bits_per_frame),
            rx_bits.begin() + static_cast<long long>((i + 1) * bits_per_frame)
        );

        size_t frame_errors = 0;
        compute_ber(ref_bits, rx_frame, frame_errors);

        total_bit_errors += frame_errors;
        total_compared_bits += std::min(ref_bits.size(), rx_frame.size());
    }

    RxOnlyResult rr;
    rr.rx_sample_count = rx_sig.size();
    rr.decoded_bits_count = rx_bits.size();
    rr.decoded_frames = decoded_frames;
    rr.total_bit_errors = total_bit_errors;
    rr.total_compared_bits = total_compared_bits;
    rr.total_ber =
        total_compared_bits ?
        static_cast<double>(total_bit_errors) / static_cast<double>(total_compared_bits) :
        0.0;
    rr.fs = cfg.fs;

    {
        const VecComplex rx_sig_wave_mid = extract_center_segment(rx_sig, 4000);
        const VecComplex rx_sig_spec_mid = extract_center_segment(rx_sig, 1024);

        build_waveform_from_signal(
            rx_sig_wave_mid,
            modulation,
            rr.rx_waveform,
            rr.rx_waveform_type
        );

        build_spectrum_from_signal(
            rx_sig_spec_mid,
            cfg.fs,
            rr.rx_spectrum_freq,
            rr.rx_spectrum_mag
        );
    }

    build_constellation_result(rx.getLastConstellationPoints(), rr);

    log << "[CONSTELLATION POINTS] " << rr.constellation_i.size() << "\n";
    log << "[DECODED BITS] " << rr.decoded_bits_count << "\n";
    log << "[DECODED FRAMES] " << rr.decoded_frames << "\n";
    log << "[BIT ERRORS] " << rr.total_bit_errors << "\n";
    log << "[COMPARED BITS] " << rr.total_compared_bits << "\n";
    log << "BER = " << rr.total_ber << "\n";

    rr.log_text = log.str();
    return rr;
}

RxFileResult run_rx_file_role_once(
    const std::string& rx_device_args,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern,
    const std::string& output_file_path,
    int max_frames
)
{
    std::ostringstream log;

    if (output_file_path.empty()) {
        throw std::runtime_error("output_file_path is empty");
    }

    if (max_frames <= 0) {
        throw std::runtime_error("max_frames must be > 0");
    }

    TransmitterConfig cfg = make_base_config(
        modulation,
        info_rate_bps,
        hop_pattern,
        true
    );

    cfg.source_mode = SourceMode::RandomBits;

    Transmitter tx_ref(cfg);
    VecComplex ref_frame = tx_ref.generateTransmitSignal();
    cfg.fs = tx_ref.getFS();

    if (ref_frame.empty()) {
        throw std::runtime_error("Reference frame is empty");
    }

    const size_t single_frame_samples = ref_frame.size();
    const size_t rx_need =
        single_frame_samples * static_cast<size_t>(max_frames);

    Receiver rx(cfg);

    log << "[ROLE] GROUND(RX FILE AUTO)\n";
    log << "[FS] " << cfg.fs << "\n";
    log << "[RX DEVICE ARGS] " << rx_device_args << "\n";
    log << "[RX NEED] " << rx_need << "\n";

    VecComplex rx_sig = capture_rx_samples(rx_device_args, center_freq_hz, cfg.fs, rx_need, log);

    if (rx_sig.empty()) {
        throw std::runtime_error("RX signal empty");
    }

    VecInt rx_bits = rx.receive(rx_sig);

    log << "[RX BITS] " << rx_bits.size() << "\n";

    std::vector<unsigned char> bytes = bits_to_bytes(rx_bits);

    if (bytes.size() < 16) {
        throw std::runtime_error("Not enough data for header");
    }

    size_t pos = 0;
    bool found = false;

    for (size_t i = 0; i + 4 <= bytes.size(); ++i)
    {
        if (bytes[i] == 'U' &&
            bytes[i + 1] == 'A' &&
            bytes[i + 2] == 'V' &&
            bytes[i + 3] == 'F')
        {
            pos = i;
            found = true;
            break;
        }
    }

    if (!found) {
        throw std::runtime_error("UAVF header not found");
    }

    const unsigned short filename_len = read_u16_be(bytes, pos + 6);
    const unsigned long long file_size = read_u64_be(bytes, pos + 8);

    const size_t header_size = 16ull + filename_len;
    const size_t total_size = header_size + static_cast<size_t>(file_size);

    log << "[FILE SIZE] " << file_size << "\n";
    log << "[HEADER SIZE] " << header_size << "\n";
    log << "[TOTAL SIZE] " << total_size << "\n";

    if (bytes.size() < pos + total_size) {
        throw std::runtime_error("RX data not enough for full file");
    }

    std::string recovered_filename(
        reinterpret_cast<const char*>(&bytes[pos + 16]),
        reinterpret_cast<const char*>(&bytes[pos + 16 + filename_len])
    );

    std::ofstream ofs(output_file_path, std::ios::binary);
    if (!ofs) {
        throw std::runtime_error("Cannot open output file for writing: " + output_file_path);
    }

    ofs.write(
        reinterpret_cast<const char*>(&bytes[pos + header_size]),
        static_cast<std::streamsize>(file_size)
    );

    RxFileResult rr;

    rr.rx_sample_count = rx_sig.size();
    rr.decoded_bits_count = rx_bits.size();
    rr.decoded_frames = rx_bits.size() / (cfg.frame_bit * cfg.n);
    rr.fs = cfg.fs;

    rr.file_saved = true;
    rr.recovered_filename = recovered_filename;
    rr.saved_file_path = output_file_path;

    build_constellation_result(rx.getLastConstellationPoints(), rr);

    log << "FILE RECOVERED = YES\n";
    log << "FILENAME = " << recovered_filename << "\n";

    rr.log_text = log.str();

    return rr;
}
