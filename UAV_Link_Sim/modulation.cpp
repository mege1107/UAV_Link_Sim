#include "modulation.h"
#include "utils.h"
#include "coding.h"

#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace {

    // MATLAB filter(b,1,x) 风格：输出长度 == 输入长度
    VecDouble firFilterSameLength(const VecDouble& b, const VecDouble& x)
    {
        if (b.empty() || x.empty()) return {};

        VecDouble y(x.size(), 0.0);

        for (size_t n = 0; n < x.size(); ++n) {
            double acc = 0.0;
            const size_t kmax = std::min(n, b.size() - 1);
            for (size_t k = 0; k <= kmax; ++k) {
                acc += b[k] * x[n - k];
            }
            y[n] = acc;
        }

        return y;
    }

    // 复数版 MATLAB filter(b,1,x)
    VecComplex firFilterSameLengthComplex(const VecDouble& b, const VecComplex& x)
    {
        if (b.empty() || x.empty()) return {};

        VecComplex y(x.size(), Complex(0.0, 0.0));

        for (size_t n = 0; n < x.size(); ++n) {
            Complex acc(0.0, 0.0);
            const size_t kmax = std::min(n, b.size() - 1);
            for (size_t k = 0; k <= kmax; ++k) {
                acc += x[n - k] * b[k];
            }
            y[n] = acc;
        }

        return y;
    }

    // 发射端 RRC 成形：
    // 1) upsample
    // 2) 末尾补 filter_delay 个 0
    // 3) filter
    // 4) 去掉前 filter_delay 个点
    // 最终输出长度 = symbols.size() * sps
    VecComplex rrcPulseShape(const VecComplex& symbols, int sps, double beta = 0.5, int span = 6)
    {
        if (symbols.empty() || sps <= 0) return {};

        VecDouble rrc = designSRRC(beta, span, sps);
        const int filter_delay = (int)(rrc.size() - 1) / 2;

        VecComplex up(symbols.size() * (size_t)sps, Complex(0.0, 0.0));
        for (size_t i = 0; i < symbols.size(); ++i) {
            up[i * (size_t)sps] = symbols[i];
        }

        up.insert(up.end(), (size_t)filter_delay, Complex(0.0, 0.0));

        VecComplex y = firFilterSameLengthComplex(rrc, up);
        if (y.size() <= (size_t)filter_delay) return {};

        return VecComplex(y.begin() + filter_delay, y.end());
    }

} // namespace

// 真正的 MSK 调制
// MSK = h=0.5 的连续相位 2FSK
// 每个 bit 对应频偏符号 a_k ∈ {+1,-1}
// 每个 symbol 内总相位变化为 ±pi/2
VecComplex mskmod(const VecInt& bits, int samp)
{
    VecComplex tx;
    if (bits.empty() || samp <= 0) return tx;

    tx.reserve(bits.size() * (size_t)samp);

    double phase = 0.0;
    const double dphi_unit = PI / (2.0 * (double)samp); // 每采样点相位增量单位

    for (int bit : bits) {
        const double a = bit ? 1.0 : -1.0;

        for (int n = 0; n < samp; ++n) {
            phase += a * dphi_unit;
            tx.emplace_back(std::cos(phase), std::sin(phase));
        }
    }

    return tx;
}

// BPSK调制（RRC成形）
VecComplex bpskmod(const VecInt& bits, int samp)
{
    if (bits.empty() || samp <= 0) return {};

    VecComplex symbols;
    symbols.reserve(bits.size());

    for (int b : bits) {
        symbols.emplace_back(b ? 1.0 : -1.0, 0.0);
    }

    return rrcPulseShape(symbols, samp, 0.5, 6);
}

// QPSK调制（按你 MATLAB 那版：I/Q 直接映射到 ±1，再做 RRC）
VecComplex qpskmod(const VecInt& bits, int samp)
{
    if (bits.empty() || samp <= 0) return {};

    VecComplex symbols;
    symbols.reserve((bits.size() + 1) / 2);

    for (size_t i = 0; i < bits.size(); i += 2) {
        int I = bits[i];
        int Q = (i + 1 < bits.size()) ? bits[i + 1] : 0;

        double re = 2.0 * (double)I - 1.0;
        double im = 2.0 * (double)Q - 1.0;

        symbols.emplace_back(re / std::sqrt(2.0), im / std::sqrt(2.0));
    }

    return rrcPulseShape(symbols, samp, 0.5, 6);
}

// 16QAM调制（自然二进制映射，带 RRC）
VecComplex qammod(const VecInt& bits, int samp)
{
    if (bits.empty() || samp <= 0) return {};

    const double levels[4] = { -3.0, -1.0, 1.0, 3.0 };

    VecComplex symbols;
    symbols.reserve((bits.size() + 3) / 4);

    for (size_t i = 0; i < bits.size(); i += 4) {
        int b0 = bits[i];
        int b1 = (i + 1 < bits.size()) ? bits[i + 1] : 0;
        int b2 = (i + 2 < bits.size()) ? bits[i + 2] : 0;
        int b3 = (i + 3 < bits.size()) ? bits[i + 3] : 0;

        int I_idx = (b0 << 1) | b1;
        int Q_idx = (b2 << 1) | b3;

        symbols.emplace_back(levels[I_idx], levels[Q_idx]);
    }

    // 平均功率归一化
    const double norm = std::sqrt(10.0);
    for (auto& s : symbols) s /= norm;

    return rrcPulseShape(symbols, samp, 0.5, 6);
}

// OOK调制（带 RRC）
VecComplex ookmod(const VecInt& bits, int samp)
{
    if (bits.empty() || samp <= 0) return {};

    VecComplex symbols;
    symbols.reserve(bits.size());

    for (int b : bits) {
        symbols.emplace_back((double)b, 0.0);
    }

    return rrcPulseShape(symbols, samp, 0.5, 6);
}

// 连续相位 2FSK 调制
VecComplex fskmod(const VecInt& bits, int M, double deta_f, int samp, double fs)
{
    (void)M;

    VecComplex tx;
    if (bits.empty() || samp <= 0 || fs <= 0.0) return tx;

    tx.reserve(bits.size() * (size_t)samp);

    double phase = 0.0;
    const double dphi = 2.0 * PI * deta_f / fs; // 每采样点的相位增量幅值

    for (int bit : bits) {
        const double sign = bit ? 1.0 : -1.0;

        for (int n = 0; n < samp; ++n) {
            phase += sign * dphi;
            tx.emplace_back(std::cos(phase), std::sin(phase));
        }
    }

    return tx;
}

// FM调制（严格对齐 MATLAB）
VecComplex fmmod(const VecInt& bits, int samp, double fs, double kf)
{
    if (bits.empty() || samp <= 0 || fs <= 0.0) return {};

    // 1) 差分编码
    VecInt encoded_msg = d_encode(bits);

    // 2) 双极性映射：0 -> -1, 1 -> +1
    VecDouble bipolar_msg_source(encoded_msg.size(), 0.0);
    for (size_t i = 0; i < encoded_msg.size(); ++i) {
        bipolar_msg_source[i] = 2.0 * (double)encoded_msg[i] - 1.0;
    }

    // 3) SRRC 滤波器
    const double rolloff_factor = 0.5;
    const int span = 6;
    VecDouble rcos_fir = designSRRC(rolloff_factor, span, samp);
    const int filter_delay = (int)(rcos_fir.size() - 1) / 2;

    // 4) 上采样
    VecDouble up_bipolar_msg_source(encoded_msg.size() * (size_t)samp, 0.0);
    for (size_t i = 0; i < bipolar_msg_source.size(); ++i) {
        up_bipolar_msg_source[i * (size_t)samp] = bipolar_msg_source[i];
    }

    // 5) MATLAB: up = [up, zeros(1, filter_delay)];
    up_bipolar_msg_source.insert(up_bipolar_msg_source.end(), (size_t)filter_delay, 0.0);

    // 6) MATLAB 风格 FIR
    VecDouble m = firFilterSameLength(rcos_fir, up_bipolar_msg_source);

    // 7) MATLAB: m = m(filter_delay+1:end)
    if (m.size() <= (size_t)filter_delay) return {};
    VecDouble m_aligned(m.begin() + filter_delay, m.end());

    // 8) FM 调制：phi = 2*pi*kf*cumsum(m)/fs
    VecComplex txSig;
    txSig.reserve(m_aligned.size());

    double phi = 0.0;
    const double scale = 2.0 * PI * kf / fs;
    for (double v : m_aligned) {
        phi += scale * v;
        txSig.emplace_back(std::cos(phi), std::sin(phi));
    }

    return txSig;
}
