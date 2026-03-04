#pragma once
#include "common_defs.h"

// 生成跳频序列
// min/max_freq: 频率范围 | step: 步长 | length: 长度 | seed: 种子
VecDouble generate_sequence(int min_freq, int max_freq, int step, int length, int seed);

// 跳频调制
// txSig: 基带信号 | freq_seq: 跳频序列 | pulse_len: 单脉冲长度 | nt: 跳频间隔 | fs: 采样率
VecComplex frequencyHop(const VecComplex& txSig, const VecDouble& freq_seq, int pulse_len, int nt, double fs);