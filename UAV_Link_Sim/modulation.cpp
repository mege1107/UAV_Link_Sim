#include "modulation.h"
#include "utils.h" 
#include "coding.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

// MSK调制 (简化版，对应MATLAB mskmod)
VecComplex mskmod(const VecInt& bits, int samp) {
    VecComplex tx;
    double phase = 0;
    double T = 1.0; // 符号周期
    double Ts = T / samp;

    for (size_t i = 0; i < bits.size(); ++i) {
        double b = bits[i] ? 1.0 : -1.0;
        for (int j = 0; j < samp; ++j) {
            double t = j * Ts;
            // MSK 正交调制公式
            double I = b * std::cos(PI * t / (2 * T)) * std::cos(phase);
            double Q = b * std::sin(PI * t / (2 * T)) * std::sin(phase);
            tx.emplace_back(I, Q);
        }
        // 更新相位 (MSK相位连续性)
        phase += bits[i] ? PI / 2 : -PI / 2;
    }
    return tx;
}

// BPSK调制
VecComplex bpskmod(const VecInt& bits, int samp) {
    VecComplex upsampled = upsampleComplex(VecComplex(bits.size()), samp);
    for (size_t i = 0; i < bits.size(); ++i) {
        upsampled[i * samp] = bits[i] ? 1.0 : -1.0;
    }
    return upsampled;
}

// QPSK调制 (Gray编码, pi/4偏移)
VecComplex qpskmod(const VecInt& bits, int samp, double& fs) {
    fs /= 2.0; // 对应MATLAB代码中 fs = fs/2
    VecComplex symbols;

    for (size_t i = 0; i < bits.size(); i += 2) {
        int b0 = bits[i];
        int b1 = (i + 1 < bits.size()) ? bits[i + 1] : 0;
        // Gray映射到 pi/4, 3pi/4, 5pi/4, 7pi/4
        double theta = (2 * (b0 ^ b1) + 1) * PI / 4 + (b1 ? PI : 0);
        symbols.emplace_back(std::cos(theta), std::sin(theta));
    }

    return upsampleComplex(symbols, samp);
}

// 16QAM调制 (归一化功率)
VecComplex qammod(const VecInt& bits, int samp, double& fs) {
    fs /= 4.0; // 对应MATLAB代码中 fs = fs/4
    VecComplex symbols;

    // 16QAM星座图映射 (Gray编码)
    const double map[4] = { -3.0, -1.0, 1.0, 3.0 };

    for (size_t i = 0; i < bits.size(); i += 4) {
        int b0 = bits[i];
        int b1 = (i + 1 < bits.size()) ? bits[i + 1] : 0;
        int b2 = (i + 2 < bits.size()) ? bits[i + 2] : 0;
        int b3 = (i + 3 < bits.size()) ? bits[i + 3] : 0;

        int I_idx = 2 * (b0 ^ b1) + b1;
        int Q_idx = 2 * (b2 ^ b3) + b3;

        symbols.emplace_back(map[I_idx], map[Q_idx]);
    }

    // 归一化 (平均功率为1)
    double norm = std::sqrt(10.0);
    for (auto& s : symbols) s /= norm;

    return upsampleComplex(symbols, samp);
}

// OOK调制
VecComplex ookmod(const VecInt& bits, int samp) {
    VecInt up = upsampleInt(bits, samp);
    VecComplex res(up.begin(), up.end());
    return res;
}

// FSK调制
VecComplex fskmod(const VecInt& bits, int M, double deta_f, int samp, double fs) {
    VecComplex tx;
    double T = 1.0;
    double Ts = 1.0 / fs;

    for (int b : bits) {
        double f = b ? deta_f : -deta_f;
        for (int j = 0; j < samp; ++j) {
            double t = j * Ts;
            tx.emplace_back(std::cos(2 * PI * f * t), std::sin(2 * PI * f * t));
        }
    }
    return tx;
}

// FM调制 (含升余弦滤波)
VecComplex fmmod(const VecInt& bits, int samp, double fs, double kf) {
    // 1. 差分编码与双极性
    VecInt diff = d_encode(bits);
    VecDouble m(diff.size());
    for (size_t i = 0; i < diff.size(); ++i) m[i] = 2 * diff[i] - 1.0;

    // 2. 上采样
    VecDouble m_up(m.size() * samp, 0.0);
    for (size_t i = 0; i < m.size(); ++i) m_up[i * samp] = m[i];

    // 3. 升余弦滤波器 (简化版，sqrt)
    double rolloff = 0.5;
    int span = 6;
    int filter_len = span * samp + 1;
    VecDouble rcos(filter_len);
    int mid = filter_len / 2;

    for (int i = 0; i < filter_len; ++i) {
        double t = (i - mid) / (double)samp;
        double num = std::sin(PI * t) * std::cos(PI * rolloff * t);
        double den = PI * t * (1 - (2 * rolloff * t) * (2 * rolloff * t));
        rcos[i] = (t == 0) ? 1.0 : num / den;
    }

    // 4. 滤波 (卷积)
    VecDouble m_filtered(m_up.size() + filter_len - 1, 0.0);
    for (size_t i = 0; i < m_up.size(); ++i) {
        for (int j = 0; j < filter_len; ++j) {
            m_filtered[i + j] += m_up[i] * rcos[j];
        }
    }
    // 裁剪延迟
    m_filtered = VecDouble(m_filtered.begin() + mid, m_filtered.end() - mid);

    // 5. 积分与调相
    VecComplex tx;
    double phi = 0.0;
    double Ts = 1.0 / fs;
    for (double sample : m_filtered) {
        phi += 2 * PI * kf * sample * Ts;
        tx.emplace_back(std::cos(phi), std::sin(phi));
    }

    return tx;
}