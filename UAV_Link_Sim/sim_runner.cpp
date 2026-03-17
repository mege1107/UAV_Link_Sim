#include "sim_runner.h"

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <ctime>
#include <stdexcept>
#include <fstream>

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
            const int bit = raw & 1;   // 只取最低位，而不是用真假判断
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

    // 打印前 32 个字节，方便查头
    std::cout << "[FILE][RX] first bytes = ";
    for (size_t i = 0; i < std::min<size_t>(32, bytes.size()); ++i) {
        std::cout << (int)bytes[i] << " ";
    }
    std::cout << "\n";

    // 最小头长度：
    // 4B magic + 1B version + 1B file_type + 2B filename_len + 8B file_size = 16B
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

// =========================
// 原有随机 bit 测试：保留
// =========================
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

    // 随机 bit 模式
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
            rx_bits.begin() + static_cast<long long>(i * bits_per_frame),
            rx_bits.begin() + static_cast<long long>((i + 1) * bits_per_frame)
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
        (double)total_bit_errors / (double)total_compared_bits :
        0.0;

    log << "BER = " << tr.total_ber << "\n";

    tr.log_text = log.str();

    return tr;
}

// =========================
// 新增：文件传输测试
// 连续多帧发送整个文件
// =========================
TestResult run_file_transfer_test(
    RunMode mode,
    double awgn_snr_db,
    double center_freq_hz,
    ModulationType modulation,
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
    cfg.Rb = 50000;
    cfg.connect = (mode == RunMode::USRP);

    // 文件模式
    cfg.source_mode = SourceMode::FileBits;
    cfg.input_file_path = input_file_path;
    cfg.file_bits.clear();
    cfg.file_bit_offset = 0;

    Transmitter tx(cfg);

    // 第一帧先生成一次，让 tx 内部完成文件打包
    VecComplex first_frame_sig = tx.generateTransmitSignal();
    VecInt first_frame_bits = tx.getLastSourceBits();

    cfg.fs = tx.getFS();

    // 注意：tx 内部 config_ 会维护自己的 file_bits 和 offset
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
    log << "[INPUT FILE] " << input_file_path << "\n";
    log << "[OUTPUT FILE] " << output_file_path << "\n";
    log << "[PACKED FILE BITS] " << total_file_bits << "\n";
    log << "[BITS PER FRAME] " << bits_per_frame << "\n";
    log << "[TOTAL FRAMES] " << total_frames << "\n";

    VecComplex tx_burst;
    VecInt tx_all_bits;

    // 已经生成了第一帧
    tx_burst.insert(tx_burst.end(), first_frame_sig.begin(), first_frame_sig.end());
    tx_all_bits.insert(tx_all_bits.end(), first_frame_bits.begin(), first_frame_bits.end());

    // 后续帧继续按文件偏移发送
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

    TestResult tr;
    tr.total_compared_bits = total_compared_bits;
    tr.total_bit_errors = bit_errors;
    tr.decoded_frames = bits_per_frame ? (rx_bits.size() / bits_per_frame) : 0;
    tr.total_ber = ber;

    std::string recovered_filename;
    std::vector<unsigned char> tx_bytes_dbg = bits_to_bytes(tx_all_bits);
    std::vector<unsigned char> rx_bytes_dbg = bits_to_bytes(rx_bits);

    std::cout << "[DBG][TX bytes from tx_all_bits] ";
    for (size_t i = 0; i < std::min<size_t>(32, tx_bytes_dbg.size()); ++i) {
        std::cout << (int)tx_bytes_dbg[i] << " ";
    }
    std::cout << "\n";

    std::cout << "[DBG][RX bytes from rx_bits] ";
    for (size_t i = 0; i < std::min<size_t>(32, rx_bytes_dbg.size()); ++i) {
        std::cout << (int)rx_bytes_dbg[i] << " ";
    }
    std::cout << "\n";
    bool save_ok = recover_file_from_bits(rx_bits, output_file_path, recovered_filename);

    tr.file_saved = save_ok;
    tr.saved_file_path = save_ok ? output_file_path : "";

    log << "[RX BITS] " << rx_bits.size() << "\n";
    log << "[DECODED FRAMES] " << tr.decoded_frames << "\n";
    log << "BER = " << tr.total_ber << "\n";
    log << "FILE RECOVERED = " << (save_ok ? "YES" : "NO") << "\n";

    if (save_ok) {
        log << "RECOVERED ORIGINAL NAME = " << recovered_filename << "\n";
        log << "SAVED TO = " << output_file_path << "\n";
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