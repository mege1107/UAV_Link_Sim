#include "spread_spectrum.h"
#include "utils.h" 
#include <algorithm>
#include <stdexcept>

// CCSK (32,5) 软扩频
VecInt ZCY_CCSK32(const VecInt& encoded_msg, const VecInt& ccskcode) {
    if (ccskcode.size() != 32) {
        throw std::invalid_argument("CCSK code must be 32 bits long");
    }

    VecInt spread;
    // 每5个bit为一组进行映射
    for (size_t i = 0; i < encoded_msg.size(); i += 5) {
        VecInt bits(5, 0);
        // 取5bit (不足补0)
        for (int j = 0; j < 5 && (i + j) < encoded_msg.size(); ++j) {
            bits[j] = encoded_msg[i + j];
        }

        // 5bit转0-31的索引
        int k = bi2de(bits, true);

        // 循环右移k位 (Link16 CCSK映射)
        VecInt chip(32);
        for (int j = 0; j < 32; ++j) {
            int idx = (j - k + 32) % 32; // 右移k位
            chip[j] = ccskcode[idx];
        }

        // 拼接到结果
        spread.insert(spread.end(), chip.begin(), chip.end());
    }
    return spread;
}