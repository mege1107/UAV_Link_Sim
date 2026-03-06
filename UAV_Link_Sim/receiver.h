#pragma once

#include "common_defs.h"
#include "transmitter.h"
#include <vector>

class Receiver {
public:
    Receiver() = delete;
    explicit Receiver(const TransmitterConfig& config);

    VecInt receive(const VecComplex& rx_signal);

    double estimateCFOFromPreamble(const VecComplex& rx_preamble,
        const VecComplex& local_preamble,
        double fs);

    void linearFitLSQ(const std::vector<int>& x,
        const std::vector<double>& y,
        double& out_a,
        double& out_b);

    double estimateResidualPhase(const VecComplex& rx_preamble,
        const VecComplex& local_preamble);

    void resetFreqHistory();

private:
    // ===== 统一解调入口 =====
    VecInt demodulateTelemetry(const VecComplex& rx);

    // ===== 各调制方式解调 =====
    VecInt demodulateBPSK(const VecComplex& rx);
    VecInt demodulateQPSK(const VecComplex& rx);
    VecInt demodulateQAM16(const VecComplex& rx);
    VecInt demodulateOOK(const VecComplex& rx);
    VecInt demodulateFSK(const VecComplex& rx);
    VecInt demodulateFM(const VecComplex& rx);

    // ===== 辅助 =====
    VecInt despreadCCSK(const VecInt& chips);

    int getTelemetryPayloadSampleCount(int ccsk_chip_num) const;
    bool modulationNeedsDiffDecode() const;
    Complex pickBestSampleInSymbol(const VecComplex& rx, size_t base, int s) const;

private:
    const TransmitterConfig& config_;

    std::vector<double> freq_offset_history_;
    std::vector<int>    frame_index_history_;
    double fit_a_ = 0.0;
    double fit_b_ = 0.0;
    int frame_count_ = 0;
};