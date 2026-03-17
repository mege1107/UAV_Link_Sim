#include "transmitter.h"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <cctype>

Transmitter::Transmitter(const TransmitterConfig& config) : config_(config) {
    calculateFS();
    setRandomSeed(1); // 对应 MATLAB rng(1)
}

void Transmitter::calculateFS() {
    // 对应 MATLAB: fs = round(Rb*FrameRate*RsRate*spsp*samp);
    config_.fs = std::round(
        config_.Rb * config_.FrameRate *
        config_.RsRate * config_.spsp * config_.samp
    );
}

// =========================
// 原有随机 bit 源：保留
// =========================
VecInt Transmitter::generateRandomSourceData() {
    int total_bits = config_.frame_bit * config_.n;
    return generateRandomBits(total_bits);
}

// =========================
// 文件名提取（不使用 filesystem）
// =========================
std::string Transmitter::extractFilename(const std::string& fullpath)
{
    if (fullpath.empty()) return "";

    size_t p1 = fullpath.find_last_of('/');
    size_t p2 = fullpath.find_last_of('\\');

    size_t pos = std::string::npos;
    if (p1 == std::string::npos) pos = p2;
    else if (p2 == std::string::npos) pos = p1;
    else pos = std::max(p1, p2);

    if (pos == std::string::npos) return fullpath;
    if (pos + 1 >= fullpath.size()) return "";

    return fullpath.substr(pos + 1);
}

std::string Transmitter::detectFileExtensionLower(const std::string& filename)
{
    size_t pos = filename.find_last_of('.');
    if (pos == std::string::npos || pos + 1 >= filename.size()) {
        return "";
    }

    std::string ext = filename.substr(pos);
    std::transform(ext.begin(), ext.end(), ext.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

int Transmitter::fileTypeFromExtension(const std::string& ext)
{
    if (ext == ".txt") return 1;
    if (ext == ".jpg" || ext == ".jpeg") return 2;
    if (ext == ".mp4") return 3;
    return 255;
}

// =========================
// 读二进制文件
// =========================
std::vector<unsigned char> Transmitter::readBinaryFile(const std::string& filepath)
{
    std::ifstream ifs(filepath, std::ios::binary);
    if (!ifs) {
        throw std::runtime_error("Cannot open input file: " + filepath);
    }

    ifs.seekg(0, std::ios::end);
    std::streamoff sz = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    if (sz < 0) {
        throw std::runtime_error("Invalid file size: " + filepath);
    }

    std::vector<unsigned char> data(static_cast<size_t>(sz));
    if (!data.empty()) {
        ifs.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
    }

    return data;
}

// =========================
// byte -> bits
// 高位在前
// =========================
VecInt Transmitter::bytesToBits(const std::vector<unsigned char>& bytes)
{
    VecInt bits;
    bits.reserve(bytes.size() * 8);

    for (unsigned char b : bytes) {
        for (int i = 7; i >= 0; --i) {
            bits.push_back((b >> i) & 1);
        }
    }

    return bits;
}

void Transmitter::appendUint16(std::vector<unsigned char>& out, unsigned short v)
{
    out.push_back(static_cast<unsigned char>((v >> 8) & 0xFF));
    out.push_back(static_cast<unsigned char>(v & 0xFF));
}

void Transmitter::appendUint64(std::vector<unsigned char>& out, unsigned long long v)
{
    for (int i = 7; i >= 0; --i) {
        out.push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xFF));
    }
}

// =========================
// 构造文件传输 packet：
// [4B magic="UAVF"]
// [1B version]
// [1B file_type]
// [2B filename_len]
// [8B file_size]
// [filename bytes]
// [file bytes]
// =========================
VecInt Transmitter::buildFilePacketBits(const std::string& filepath)
{
    const std::string filename = extractFilename(filepath);
    const std::string ext = detectFileExtensionLower(filename);
    const int file_type = fileTypeFromExtension(ext);

    std::vector<unsigned char> file_bytes = readBinaryFile(filepath);

    if (filename.size() > 65535) {
        throw std::runtime_error("Filename too long");
    }

    std::vector<unsigned char> packed;
    packed.reserve(16 + filename.size() + file_bytes.size());

    // magic
    packed.push_back('U');
    packed.push_back('A');
    packed.push_back('V');
    packed.push_back('F');

    // version
    packed.push_back(1);

    // file type
    packed.push_back(static_cast<unsigned char>(file_type));

    // filename len
    appendUint16(packed, static_cast<unsigned short>(filename.size()));

    // file size
    appendUint64(packed, static_cast<unsigned long long>(file_bytes.size()));

    // filename
    for (char c : filename) {
        packed.push_back(static_cast<unsigned char>(c));
    }

    // file payload
    packed.insert(packed.end(), file_bytes.begin(), file_bytes.end());

    std::cout << "[FILE][TX][PACKED BYTES] ";
    for (size_t i = 0; i < std::min<size_t>(32, packed.size()); ++i) {
        std::cout << (int)packed[i] << " ";
    }
    std::cout << "\n";

    return bytesToBits(packed);
}

// =========================
// 新增文件 bit 源
// 每次取一帧所需 bit，不足补 0
// =========================
VecInt Transmitter::generateFileSourceBits()
{
    const int total_bits = config_.frame_bit * config_.n;
    VecInt out;
    out.reserve(static_cast<size_t>(total_bits));

    if (config_.source_mode != SourceMode::FileBits) {
        throw std::runtime_error("generateFileSourceBits called while source_mode != FileBits");
    }

    // 第一次调用时，自动把整个文件封装成 bit 流
    if (config_.file_bits.empty()) {
        if (config_.input_file_path.empty()) {
            throw std::runtime_error("input_file_path is empty");
        }

        config_.file_bits = buildFilePacketBits(config_.input_file_path);
        config_.file_bit_offset = 0;

        std::cout << "[FILE][TX] input_file = " << config_.input_file_path << "\n";
        std::cout << "[FILE][TX] packed file bits = " << config_.file_bits.size() << "\n";
    }

    if (config_.file_bit_offset > config_.file_bits.size()) {
        config_.file_bit_offset = config_.file_bits.size();
    }

    const size_t remain = config_.file_bits.size() - config_.file_bit_offset;
    const size_t take = std::min(static_cast<size_t>(total_bits), remain);

    if (take > 0) {
        out.insert(
            out.end(),
            config_.file_bits.begin() + static_cast<long long>(config_.file_bit_offset),
            config_.file_bits.begin() + static_cast<long long>(config_.file_bit_offset + take)
        );
        config_.file_bit_offset += take;
    }

    // 不足一帧补 0
    if (out.size() < static_cast<size_t>(total_bits)) {
        out.insert(out.end(), static_cast<size_t>(total_bits) - out.size(), 0);
    }

    return out;
}

VecComplex Transmitter::buildRemoteControlPulse(
    const VecComplex& hoppedSig,
    int pulse_len,
    int gap_len,
    int total_pulses
) {
    VecComplex pulsed;
    if (pulse_len <= 0 || total_pulses <= 0) return pulsed;

    const size_t pulse_len_u = static_cast<size_t>(pulse_len);
    const size_t gap_len_u = static_cast<size_t>(std::max(0, gap_len));

    pulsed.reserve(
        static_cast<size_t>(total_pulses) * (pulse_len_u + gap_len_u)
    );

    for (int i = 0; i < total_pulses; ++i) {
        size_t start = static_cast<size_t>(i) * pulse_len_u;
        size_t end = start + pulse_len_u;

        if (end > hoppedSig.size()) break;

        // 插入一个 pulse
        pulsed.insert(
            pulsed.end(),
            hoppedSig.begin() + static_cast<long long>(start),
            hoppedSig.begin() + static_cast<long long>(end)
        );

        // pulse 后插入保护间隔 gap
        if (gap_len_u > 0) {
            pulsed.insert(
                pulsed.end(),
                gap_len_u,
                Complex(0.0, 0.0)
            );
        }
    }

    return pulsed;
}

VecComplex Transmitter::generateTransmitSignal() {
    // 1. 信源
    if (config_.source_mode == SourceMode::FileBits) {
        last_source_bits_ = generateFileSourceBits();
    }
    else {
        last_source_bits_ = generateRandomSourceData();
    }

    VecInt dataIn = last_source_bits_;

    // 2. RS编码 (31,15)
    VecInt encoded_msg;
    const int rs_msg_bits = 15 * 5;   // 75

    for (size_t off = 0; off + rs_msg_bits <= dataIn.size(); off += rs_msg_bits) {
        VecInt blk(
            dataIn.begin() + static_cast<long long>(off),
            dataIn.begin() + static_cast<long long>(off + rs_msg_bits)
        );
        VecInt enc = HXL_RSCode(blk, 31, 15);   // 75 -> 155
        encoded_msg.insert(encoded_msg.end(), enc.begin(), enc.end());
    }

    std::cout << "[DBG][TX] source bits = " << dataIn.size() << "\n";
    std::cout << "[DBG][TX] RS encoded bits = " << encoded_msg.size() << "\n";

    // 3. CCSK (32,5) 扩频
    VecInt ccsk_msg = ZCY_CCSK32(encoded_msg, config_.ccskcode);

    // 4. 生成同步序列
    VecInt m_coarse = mseq({ 1, 1, 1, 0, 1 });
    VecComplex sync_wide = generateCoarseSync(m_coarse, config_.samp, config_.coarse_length);

    VecInt pss1 = mseq({ 1, 0, 1, 0, 1, 1, 1, 0, 1 });
    VecInt pss2 = mseq({ 1, 0, 1, 0, 1, 0, 1 });
    VecComplex sync_fine = generateFineSync(pss1, pss2, config_.fine_length);

    VecComplex SYNC = concatSync(sync_wide, sync_fine);

    // 5. 调制与信号处理
    VecComplex txSig;

    if (config_.function == FunctionType::RemoteControl) {
        const int pulse_len = static_cast<int>(config_.ccskcode.size()) * config_.samp; // 32*samp
        const int gap_len = 33 * config_.samp;
        const int nt = 1;

        VecInt tx_bits_mat;
        tx_bits_mat.reserve(ccsk_msg.size());
        for (size_t i = 0; i < ccsk_msg.size(); i += 32) {
            for (int j = 0; j < 32 && (i + static_cast<size_t>(j)) < ccsk_msg.size(); ++j) {
                tx_bits_mat.push_back(ccsk_msg[i + static_cast<size_t>(j)]);
            }
        }

        txSig = mskmod(tx_bits_mat, config_.samp);

        const int total_pulses = static_cast<int>(ccsk_msg.size() / 32);
        VecDouble frq_seq = generate_sequence(-13e3, 13e3, 3e3, total_pulses, 1);

        txSig = frequencyHop(txSig, frq_seq, pulse_len, nt, config_.fs);

        txSig = buildRemoteControlPulse(txSig, pulse_len, gap_len, total_pulses);
    }
    else {
        // =========================
        // 遥测方案
        // =========================
        if (config_.modulation == ModulationType::BPSK) {
            VecInt diff = d_encode(ccsk_msg);
            txSig = bpskmod(diff, config_.samp);
        }
        else if (config_.modulation == ModulationType::QPSK) {
            double fs_local = config_.fs;
            txSig = qpskmod(ccsk_msg, config_.samp, fs_local);
            config_.fs = fs_local;
        }
        else if (config_.modulation == ModulationType::QAM) {
            double fs_local = config_.fs;
            txSig = qammod(ccsk_msg, config_.samp, fs_local);
            config_.fs = fs_local;
        }
        else if (config_.modulation == ModulationType::OOK) {
            txSig = ookmod(ccsk_msg, config_.samp);
        }
        else if (config_.modulation == ModulationType::FSK) {
            double deta_f = 1e6;
            txSig = fskmod(ccsk_msg, 2, deta_f, config_.samp, config_.fs);
        }
        else if (config_.modulation == ModulationType::FM) {
            double kf = 75e5;
            txSig = fmmod(ccsk_msg, config_.samp, config_.fs, kf);
        }
        else if (config_.modulation == ModulationType::MSK) {
            txSig = mskmod(ccsk_msg, config_.samp);
        }
    }

    // 6. 添加同步头
    VecComplex txSig_Sync = SYNC;
    txSig_Sync.insert(txSig_Sync.end(), txSig.begin(), txSig.end());

    // 7. 帧末尾添加 ZP
    const int zp_samp = std::max(0, config_.zp_sym) * std::max(1, config_.samp);
    if (zp_samp > 0) {
        txSig_Sync.insert(txSig_Sync.end(), static_cast<size_t>(zp_samp), Complex(0.0, 0.0));
    }

    return txSig_Sync;
}

void Transmitter::saveSignal(const std::string& filename, const VecComplex& signal) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Cannot open file for writing");
    }

    size_t len = signal.size();
    file.write(reinterpret_cast<const char*>(&len), sizeof(len));

    for (const auto& c : signal) {
        double re = c.real();
        double im = c.imag();
        file.write(reinterpret_cast<const char*>(&re), sizeof(re));
        file.write(reinterpret_cast<const char*>(&im), sizeof(im));
    }

    std::cout << "Signal saved to " << filename
        << " (" << len << " samples)" << std::endl;
}