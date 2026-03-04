#pragma once
#include "common_defs.h"

// RS编码（占位实现）
VecInt HXL_RSCode(const VecInt& data, int n, int k);

// RS解码（占位实现）
VecInt HXL_RSDecode(const VecInt& data, int n, int k);

// 差分编码
VecInt d_encode(const VecInt& data);

// ⭐ 差分解码
VecInt d_decode(const VecInt& diff);