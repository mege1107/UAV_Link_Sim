#pragma once

#include "common_defs.h"
#include "coding.h"
#include "spread_spectrum.h"
#include "modulation.h"
#include "sync.h"
#include "frequency_hop.h"
#include "utils.h"

#include <string>
#include <vector>

class Transmitter {
public:
    explicit Transmitter(const TransmitterConfig& config);
    ~Transmitter() = default;

    VecComplex generateTransmitSignal();

    void saveSignal(const std::string& filename,
        const VecComplex& signal);

    double getFS() const { return config_.fs; }
    const TransmitterConfig& getConfig() const { return config_; }

    int getSourceBitCount() const {
        return config_.frame_bit * config_.n;
    }

    VecInt getLastSourceBits() const { return last_source_bits_; }

private:
    TransmitterConfig config_;

    void calculateFS();

    // 保留原随机信源
    VecInt generateRandomSourceData();

    // 新增文件信源
    VecInt generateFileSourceBits();

    // 文件工具函数
    static std::string extractFilename(const std::string& fullpath);
    static std::string detectFileExtensionLower(const std::string& filename);
    static int fileTypeFromExtension(const std::string& ext);

    static std::vector<unsigned char> readBinaryFile(const std::string& filepath);
    static VecInt bytesToBits(const std::vector<unsigned char>& bytes);
    static void appendUint16(std::vector<unsigned char>& out, unsigned short v);
    static void appendUint64(std::vector<unsigned char>& out, unsigned long long v);
    static VecInt buildFilePacketBits(const std::string& filepath);

    VecComplex buildRemoteControlPulse(
        const VecComplex& hoppedSig,
        int pulse_len,
        int gap_len,
        int total_pulses
    );

    VecInt last_source_bits_;
};