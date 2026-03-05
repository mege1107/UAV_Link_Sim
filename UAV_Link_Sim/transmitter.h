#pragma once

#include "common_defs.h"
#include "coding.h"
#include "spread_spectrum.h"
#include "modulation.h"
#include "sync.h"
#include "frequency_hop.h"
#include "utils.h"

class Transmitter {
public:
    explicit Transmitter(const TransmitterConfig& config);
    ~Transmitter() = default;

    // ==============================
    // 主流程
    // ==============================
    VecComplex generateTransmitSignal();

    // 保存信号
    void saveSignal(const std::string& filename,
        const VecComplex& signal);

    // ==============================
    // 外部查询接口
    // ==============================
    double getFS() const { return config_.fs; }
    const TransmitterConfig& getConfig() const { return config_; }

    // 信源bit数量
    int getSourceBitCount() const {
        return config_.frame_bit * config_.n;
    }
    VecInt getLastSourceBits() const { return last_source_bits_; }
private:
    TransmitterConfig config_;

    // ==============================
    // 内部流程
    // ==============================
    void calculateFS();                // 计算采样率
    VecInt generateSourceData();       // 生成信源数据

    VecComplex buildRemoteControlPulse(
        const VecComplex& txSig_MSG,
        int zeros_bit,
        int pulse_len
    );
    VecInt last_source_bits_;
};