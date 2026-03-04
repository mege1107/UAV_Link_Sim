#pragma once
#include "common_defs.h"

// 生成m序列
// taps: 抽头系数向量 (如 [1,1,1,0,1])
VecInt mseq(const VecInt& taps);

// 整数序列上采样（中间插零）
// input: 输入序列 | samp: 上采样倍数
VecInt upsampleInt(const VecInt& input, int samp);

// 复数序列上采样（中间插零）
VecComplex upsampleComplex(const VecComplex& input, int samp);

// 二进制转十进制
// bits: 二进制序列 | leftMSB: 是否左边为最高位
int bi2de(const VecInt& bits, bool leftMSB = true);

// 设置随机种子（用于信源生成）
void setRandomSeed(int seed);

// 生成随机二进制序列
VecInt generateRandomBits(int length); 
