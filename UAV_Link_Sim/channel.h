#pragma once
#include "common_defs.h"

// 信道配置：让同步头真正“有用”
struct ChannelConfig {
    double snr_dB = 20.0;

    // 帧起点随机偏移（采样）
    int max_delay_samp = 0;      // 0 表示不加时延
    int delay_samp = -1;         // -1 表示随机 [0, max_delay_samp]

    // 频偏/相位
    double cfo_hz = 0.0;         // 载波频偏(Hz)
    double phase_rad = 0.0;      // 初相(rad)

    // 简易多径（可选）
    bool enable_multipath = false;
    int multipath_len = 3;       // taps长度
    double multipath_decay = 0.6;// 衰减系数 (0~1)

    // 随机种子
    int seed = 123;
};

class Channel {
public:
    // 统一信道入口：delay + CFO + phase + (multipath) + AWGN
    static VecComplex awgn(const VecComplex& signal, double snr_dB);

    // 可选：设置固定随机种子，便于复现实验（默认不需要你调用）
    static void setSeed(unsigned int seed);
};