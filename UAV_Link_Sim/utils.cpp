#include "utils.h"
#include <algorithm>
#include <stdexcept>
#include <random>
#include <cmath>

static std::mt19937 rng;

// 生成m序列 (线性反馈移位寄存器LFSR，最大长度序列)
// 输入taps：本原多项式系数向量，长度为n+1（n为寄存器级数）
// taps[k]对应多项式x^k项的系数，取值0/1，必须满足taps[0]=1、taps.back()=1
// 例：3级m序列本原多项式x³+x+1，对应taps={1,1,0,1}
VecInt mseq(const VecInt& taps) {
    if (taps.size() < 2)
        throw std::invalid_argument("抽头系数长度至少为2");

    if (taps[0] != 1 || taps.back() != 1)
        throw std::invalid_argument("首尾必须为1");

    int n = taps.size() - 1;
    VecInt reg(n, 1);   // 初始状态不能全0
    VecInt seq;

    int seq_len = (1 << n) - 1;
    seq.reserve(seq_len);

    for (int k = 0; k < seq_len; ++k) {

        // 输出最高位
        seq.push_back(reg[0]);

        // 正确反馈计算（不含 taps[0] 和 taps[n]）
        int feedback = 0;
        for (int i = 1; i < n; ++i) {
            if (taps[i]) {
                feedback ^= reg[i - 1];
            }
        }

        // 左移
        for (int i = 0; i < n - 1; ++i)
            reg[i] = reg[i + 1];

        reg[n - 1] = feedback;
    }

    return seq;
}

// 整数序列上采样（插零）
VecInt upsampleInt(const VecInt& input, int samp) {
    if (samp < 1) {
        throw std::invalid_argument("上采样倍数必须≥1");
    }
    VecInt output(input.size() * samp, 0);
    for (size_t i = 0; i < input.size(); ++i) {
        output[i * samp] = input[i];
    }
    return output;
}

// 复数序列上采样（插零）
VecComplex upsampleComplex(const VecComplex& input, int samp) {
    if (samp < 1) {
        throw std::invalid_argument("上采样倍数必须≥1");
    }
    VecComplex output(input.size() * samp, 0);
    for (size_t i = 0; i < input.size(); ++i) {
        output[i * samp] = input[i];
    }
    return output;
}

// 二进制转十进制 (leftMSB=true表示左边是高位)
int bi2de(const VecInt& bits, bool leftMSB) {
    if (bits.empty()) {
        throw std::invalid_argument("输入比特序列不能为空");
    }
    int val = 0;
    if (leftMSB) {
        for (int b : bits) val = (val << 1) | b;
    }
    else {
        for (int i = bits.size() - 1; i >= 0; --i) val = (val << 1) | bits[i];
    }
    return val;
}

// 设置随机种子
void setRandomSeed(int seed) {
    rng.seed(seed);
}

// 生成随机二进制序列
VecInt generateRandomBits(int length) {
    if (length < 0) {
        throw std::invalid_argument("序列长度不能为负");
    }
    VecInt bits(length);
    std::uniform_int_distribution<int> dist(0, 1);
    for (int i = 0; i < length; ++i) {
        bits[i] = dist(rng);
    }
    return bits;
}

// 设计 sqrt raised cosine (对应 MATLAB rcosdesign(beta, span, sps, 'sqrt'))
VecDouble designSRRC(double beta, int span, int sps)
{
    if (sps <= 0 || span <= 0) return {};

    const int N = span * sps + 1;
    const int mid = N / 2;
    VecDouble h(N, 0.0);

    for (int i = 0; i < N; ++i) {
        double t = (double)(i - mid) / (double)sps;

        if (std::abs(t) < 1e-12) {
            h[i] = 1.0 - beta + 4.0 * beta / PI;
        }
        else if (beta > 0.0 && std::abs(std::abs(t) - 1.0 / (4.0 * beta)) < 1e-12) {
            double term1 = (1.0 + 2.0 / PI) * std::sin(PI / (4.0 * beta));
            double term2 = (1.0 - 2.0 / PI) * std::cos(PI / (4.0 * beta));
            h[i] = (beta / std::sqrt(2.0)) * (term1 + term2);
        }
        else {
            double num1 = std::sin(PI * t * (1.0 - beta));
            double num2 = 4.0 * beta * t * std::cos(PI * t * (1.0 + beta));
            double den = PI * t * (1.0 - 16.0 * beta * beta * t * t);
            h[i] = (num1 + num2) / den;
        }
    }

    // 能量归一化
    double energy = 0.0;
    for (double v : h) energy += v * v;
    if (energy > 0.0) {
        double scale = 1.0 / std::sqrt(energy);
        for (double& v : h) v *= scale;
    }

    return h;
}
