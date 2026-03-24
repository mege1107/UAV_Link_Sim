#pragma once

#include "common_defs.h"

// RS编码：当前项目实际使用的是 RS(31,15) over GF(2^5)
// 输入/输出都用 bit 流表示：
//   RS(31,15): 75 bit -> 155 bit
VecInt HXL_RSCode(const VecInt& data, int n, int k);

// RS解码：
//   RS(31,15): 155 bit -> 75 bit
// 若无法纠错，会尽力返回“截断后的信息位部分”
VecInt HXL_RSDecode(const VecInt& data, int n, int k);

// 差分编码
VecInt d_encode(const VecInt& data);

// 差分解码
VecInt d_decode(const VecInt& diff);
