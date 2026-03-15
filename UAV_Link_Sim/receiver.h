#pragma once

#include "transmitter.h"
#include "utils.h"

#include <vector>
#include <complex>

class Receiver {
public:
    explicit Receiver(const TransmitterConfig& config);

    // 主接收入口
    VecInt receive(const VecComplex& rx_signal);

    // 各种调制解调
    VecInt demodulateTelemetry(const VecComplex& rx);

    VecInt demodulateBPSK(const VecComplex& rx);
    VecInt demodulateQPSK(const VecComplex& rx);
    VecInt demodulateQAM16(const VecComplex& rx);
    VecInt demodulateOOK(const VecComplex& rx);
    VecInt demodulateFSK(const VecComplex& rx);
    VecInt demodulateFM(const VecComplex& rx);
    VecInt demodulateMSK(const VecComplex& rx);

    // CCSK 解扩
    VecInt despreadCCSK(const VecInt& chips);

    // 遥控模式解跳频
    VecComplex dehopRemoteControlPayload(const VecComplex& payload_with_gap,
        int total_pulses,
        int pulse_len,
        int gap_len) const;

private:
    // ===== 频偏 / 相位相关 =====
    double estimateCFOFromPreamble(const VecComplex& rx_preamble,
        const VecComplex& local_preamble,
        double fs);

    static void linearFitLSQ(const std::vector<int>& x,
        const std::vector<double>& y,
        double& out_a,
        double& out_b);

    double estimateResidualPhase(const VecComplex& rx_preamble,
        const VecComplex& local_preamble);

    void resetFreqHistory();

    // ===== 新增：更贴近 MATLAB 的符号处理 =====
    VecComplex meanGroups(const VecComplex& rx, int s) const;
    Complex pickMidSampleInSymbol(const VecComplex& rx, size_t base, int s) const;

    // ===== 辅助 =====
    int getTelemetryPayloadSampleCount(int ccsk_chip_num) const;
    bool modulationNeedsDiffDecode() const;

private:
    TransmitterConfig config_;

    // CFO 历史拟合
    std::vector<double> freq_offset_history_;
    std::vector<int> frame_index_history_;
    double fit_a_ = 0.0;
    double fit_b_ = 0.0;
    int frame_count_ = 0;
};