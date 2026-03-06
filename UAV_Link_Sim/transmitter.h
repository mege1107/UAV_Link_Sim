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
    VecInt generateSourceData();

    // 遥控模式：把“已完成MSK+跳频”的信号重新组织成 pulse + gap
    VecComplex buildRemoteControlPulse(
        const VecComplex& hoppedSig,
        int pulse_len,
        int gap_len,
        int total_pulses
    );

    VecInt last_source_bits_;
};