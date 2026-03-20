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
    fs /= 2.0;
    VecComplex symbols;

    for (size_t i = 0; i < bits.size(); i += 2) {
        int b0 = bits[i];
        int b1 = (i + 1 < bits.size()) ? bits[i + 1] : 0;
        double theta = (2 * (b0 ^ b1) + 1) * PI / 4 + (b1 ? PI : 0);
        symbols.emplace_back(std::cos(theta), std::sin(theta));
    }

    return upsampleComplex(symbols, samp);
}

// 16QAM调制
VecComplex qammod(const VecInt& bits, int samp, double& fs) {
    fs /= 4.0;
    VecComplex symbols;

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

    const double norm = std::sqrt(10.0);
    for (auto& s : symbols) s /= norm;

    return upsampleComplex(symbols, samp);
}

// OOK调制
VecComplex ookmod(const VecInt& bits, int samp) {
    VecInt up = upsampleInt(bits, samp);
    VecComplex res(up.begin(), up.end());
    return res;
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
// 对应 MATLAB:
// encoded_msg = d_encode(bits);
// bipolar = 2*encoded_msg - 1;
// rcos_fir = rcosdesign(0.5, 6, samp, 'sqrt');
// filter_delay = (length(rcos_fir)-1)/2;
// up = upsample(bipolar, samp);
// up = [up, zeros(1, filter_delay)];
// m = filter(rcos_fir, 1, up);
// m = m(filter_delay+1:end);
// phi = 2*pi*kf*cumsum(m)/fs;
// txSig = exp(1j*phi).';
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

    // 5) MATLAB: up_bipolar_msg_source = [up_bipolar_msg_source, zeros(1, filter_delay)];
    up_bipolar_msg_source.insert(up_bipolar_msg_source.end(), (size_t)filter_delay, 0.0);

    // 6) MATLAB 的 filter(rcos_fir, 1, x)
    // 输出长度与输入长度相同，是“因果 FIR”滤波
    VecDouble m(up_bipolar_msg_source.size(), 0.0);
    for (size_t n = 0; n < up_bipolar_msg_source.size(); ++n) {
        double acc = 0.0;
        for (size_t k = 0; k < rcos_fir.size(); ++k) {
            if (n >= k) {
                acc += rcos_fir[k] * up_bipolar_msg_source[n - k];
            }
        }
        m[n] = acc;
    }

    // 7) MATLAB: m = m(filter_delay+1:end);
    if (m.size() <= (size_t)filter_delay) return {};
    VecDouble m_aligned(m.begin() + filter_delay, m.end());

    // 现在长度应为 bits.size() * samp
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