#include "sync.h"

// 生成粗同步序列
VecComplex generateCoarseSync(const VecInt& m_seq, int samp, int coarse_length) {
    // 1. BPSK映射 (0->-1, 1->1)
    VecComplex bpsk;
    for (int b : m_seq) bpsk.emplace_back(b ? 1.0 : -1.0, 0.0);

    // 2. 对应MATLAB: repmat([m_2 0], samp, 1)
    VecComplex padded;
    for (auto s : bpsk) {
        padded.push_back(s);
        padded.emplace_back(0.0, 0.0); // 补零
    }

    // 3. 上采样重复 (reshape逻辑)
    VecComplex upsampled;
    for (int i = 0; i < samp; ++i) {
        for (auto s : padded) {
            upsampled.push_back(s);
        }
    }

    // 4. 重复 coarse_length 次
    VecComplex result;
    for (int i = 0; i < coarse_length; ++i) {
        result.insert(result.end(), upsampled.begin(), upsampled.end());
    }

    return result;
}

// 生成精同步序列
VecComplex generateFineSync(const VecInt& pss_code1, const VecInt& pss_code2, int fine_length) {
    // 拼接序列 + 两个0
    VecInt concat = pss_code1;
    concat.insert(concat.end(), pss_code2.begin(), pss_code2.end());
    concat.push_back(0);
    concat.push_back(0);

    // BPSK映射
    VecComplex bpsk;
    for (int b : concat) bpsk.emplace_back(b ? 1.0 : -1.0, 0.0);

    // 重复 fine_length 次
    VecComplex result;
    for (int i = 0; i < fine_length; ++i) {
        result.insert(result.end(), bpsk.begin(), bpsk.end());
    }

    return result;
}

// 拼接同步头
VecComplex concatSync(const VecComplex& coarse, const VecComplex& fine) {
    VecComplex result = coarse;
    result.insert(result.end(), fine.begin(), fine.end());
    return result;
}