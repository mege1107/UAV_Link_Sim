#pragma once
#include "common_defs.h"

// 生成粗同步序列
// m_seq: m序列 | samp: 上采样 | coarse_length: 重复次数
VecComplex generateCoarseSync(const VecInt& m_seq, int samp, int coarse_length);

// 生成精同步序列
// pss_code1/2: PSS序列 | fine_length: 重复次数
VecComplex generateFineSync(const VecInt& pss_code1, const VecInt& pss_code2, int fine_length);

// 拼接同步头 (粗同步 + 精同步)
VecComplex concatSync(const VecComplex& coarse, const VecComplex& fine);
