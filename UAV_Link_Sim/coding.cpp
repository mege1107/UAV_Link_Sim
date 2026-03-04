#include "coding.h"

// ========================================
// RS编码（当前占位：直接返回原数据）
// ========================================
VecInt HXL_RSCode(const VecInt& data, int n, int k)
{
    // 这里暂时不做真正RS编码
    // 直接返回原始数据
    return data;
}

// ========================================
// RS解码（当前占位：直接返回原数据）
// ========================================
VecInt HXL_RSDecode(const VecInt& data, int n, int k)
{
    return data;
}

// ========================================
// 差分编码
// ========================================
VecInt d_encode(const VecInt& data)
{
    if (data.empty()) return {};

    VecInt diff(data.size());
    diff[0] = data[0];

    for (size_t i = 1; i < data.size(); ++i)
        diff[i] = data[i] ^ diff[i - 1];

    return diff;
}

// ========================================
// ⭐ 差分解码
// ========================================
VecInt d_decode(const VecInt& diff)
{
    if (diff.empty()) return {};

    VecInt data(diff.size());
    data[0] = diff[0];

    for (size_t i = 1; i < diff.size(); ++i)
        data[i] = diff[i] ^ diff[i - 1];

    return data;
}