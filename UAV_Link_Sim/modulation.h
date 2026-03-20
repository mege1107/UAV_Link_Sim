#pragma once
#include "common_defs.h"

// MSK调制
// bits: 输入比特 | samp: 上采样倍数
VecComplex mskmod(const VecInt& bits, int samp);

// BPSK调制
VecComplex bpskmod(const VecInt& bits, int samp);

// QPSK调制 (会修改fs为fs/2)
VecComplex qpskmod(const VecInt& bits, int samp, double& fs);

// 16QAM调制 (会修改fs为fs/4)
VecComplex qammod(const VecInt& bits, int samp, double& fs);

// OOK调制
VecComplex ookmod(const VecInt& bits, int samp);

// FSK调制
// deta_f: 频差 | fs: 采样率
VecComplex fskmod(const VecInt& bits, int M, double deta_f, int samp, double fs);

// FM调制
// kf: 调制灵敏度 | fs: 采样率
VecComplex fmmod(const VecInt& bits, int samp, double fs, double kf);
