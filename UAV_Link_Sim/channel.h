#pragma once
#include "common_defs.h"

// 新信道配置：STO / CFO / SFO / AWGN 独立开关
struct ChannelConfig {
    // ===== 开关 =====
    bool enable_sto = false;
    bool enable_cfo = false;
    bool enable_sfo = false;
    bool enable_awgn = true;

    // ===== AWGN =====
    double snr_dB = 20.0;

    // ===== STO =====
    // 整数采样偏移
    // >0: 在前面补零，等价于接收端看到信号整体延后
    // =0: 不加 STO
    int sto_samp = 0;

    // ===== CFO =====
    // 固定载波频偏(Hz)
    double cfo_hz = 0.0;

    // 初始相位(rad)，可与 CFO 一起用
    double phase_rad = 0.0;

    // ===== SFO =====
    // 固定采样率失配，单位 ppm
    // 例如 20 ppm => 20e-6
    // 接收端等效采样时钟偏差
    double sfo_ppm = 0.0;

    // ===== 兼容旧字段（可留可不用）=====
    int max_delay_samp = 0;   // 这版不再使用
    int delay_samp = -1;      // 这版不再使用

    bool enable_multipath = false; // 这版先不实现
    int multipath_len = 3;
    double multipath_decay = 0.6;

    // 随机种子
    int seed = 123;
};

class Channel {
public:
    // 统一入口：按开关依次施加 STO / CFO / SFO / AWGN
    static VecComplex process(const VecComplex& signal,
        const ChannelConfig& cfg,
        double fs);

    // 单独模块
    static VecComplex applySTO(const VecComplex& signal, int sto_samp);
    static VecComplex applyCFO(const VecComplex& signal,
        double cfo_hz,
        double fs,
        double phase_rad = 0.0);
    static VecComplex applySFO(const VecComplex& signal, double sfo_ppm);
    static VecComplex awgn(const VecComplex& signal, double snr_dB);

    // 固定随机种子
    static void setSeed(unsigned int seed);
};
