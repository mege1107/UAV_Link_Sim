#pragma once
#include "common_defs.h"

// CCSK软扩频 (32,5)
// encoded_msg: 编码后比特 | ccskcode: CCSK基序列
VecInt ZCY_CCSK32(const VecInt& encoded_msg, const VecInt& ccskcode);