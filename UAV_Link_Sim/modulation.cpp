#include "modulation.h"
#include "utils.h"
#include "coding.h"

#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace {

// 设计 sqrt raised cosine (对应 MATLAB rcosdesign(beta, span, sps, 'sqrt'))
VecDouble designSRRC(double beta, int span, int sps)
{
    if (sps <= 0 || span <= 0) return {};

    const int N = span * sps + 1;
    const int mid = N / 2;
    VecDouble h(N, 0.0);

    for (int i = 0; i < N; ++i) {
        const double t = (double)(i - mid) / (double)sps;

        if (std::abs(t) < 1e-12) {
            h[i] = 1.0 + beta * (4.0 / PI - 1.0);
        }
        else if (beta > 0.0 && std::abs(std::abs(t) - 1.0 / (4.0 * beta)) < 1e-12) {
            const double a = (1.0 + 2.0 / PI) * std::sin(PI / (4.0 * beta));
            const double b = (1.0 - 2.0 / PI) * std::cos(PI / (4.0 * beta));
            h[i] = (beta / std::sqrt(2.0)) * (a + b);
        }
        else {
            const double num =
                std::sin(PI * t * (1.0 - beta)) +
                4.0 * beta * t * std::cos(PI * t * (1.0 + beta));
            const double den =
                PI * t * (1.0 - std::pow(4.0 * beta * t, 2.0));

            h[i] = (std::abs(den) < 1e-12) ? 0.0 : (num / den);
        }
    }

    return h;
}

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

// MSK调制 (简化版，对应MATLAB mskmod)
VecComplex mskmod(const VecInt& bits, int samp) {
    VecComplex tx;
    double phase = 0;
    double T = 1.0;
    double Ts = T / samp;

    for (size_t i = 0; i < bits.size(); ++i) {
        double b = bits[i] ? 1.0 : -1.0;
        for (int j = 0; j < samp; ++j) {
            double t = j * Ts;
            double I = b * std::cos(PI * t / (2 * T)) * std::cos(phase);
            double Q = b * std::sin(PI * t / (2 * T)) * std::sin(phase);
            tx.emplace_back(I, Q);
        }
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

// FSK调制
VecComplex fskmod(const VecInt& bits, int M, double deta_f, int samp, double fs) {
    (void)M;
    VecComplex tx;
    const double Ts = 1.0 / fs;

    for (int b : bits) {
        const double f = b ? deta_f : -deta_f;
        for (int j = 0; j < samp; ++j) {
            const double t = j * Ts;
            tx.emplace_back(std::cos(2 * PI * f * t), std::sin(2 * PI * f * t));
        }
    }
    return tx;
}

// FM调制（严格对齐 MATLAB 逻辑）
VecComplex fmmod(const VecInt& bits, int samp, double fs, double kf) {
    if (samp <= 0 || fs <= 0.0) return {};

    // 1) 差分编码
    VecInt diff = d_encode(bits);

    // 2) 双极性映射
    VecDouble bipolar(diff.size(), 0.0);
    for (size_t i = 0; i < diff.size(); ++i) {
        bipolar[i] = diff[i] ? 1.0 : -1.0;
    }

    // 3) SRRC 滤波器
    const double rolloff = 0.5;
    const int span = 6;
    VecDouble rcos_fir = designSRRC(rolloff, span, samp);
    const int filter_delay = (int)(rcos_fir.size() - 1) / 2;

    // 4) 上采样
    VecDouble up((size_t)bipolar.size() * (size_t)samp, 0.0);
    for (size_t i = 0; i < bipolar.size(); ++i) {
        up[i * (size_t)samp] = bipolar[i];
    }

    // 5) 尾部补零（和 MATLAB 一致）
    up.insert(up.end(), (size_t)filter_delay, 0.0);

    // 6) 成形滤波，输出长度与输入长度相同
    VecDouble m = firFilterSameLength(rcos_fir, up);

    // 7) 去掉前 filter_delay
    if ((int)m.size() <= filter_delay) return {};
    m.erase(m.begin(), m.begin() + filter_delay);

    // 现在长度应回到 bits.size() * samp
    VecComplex tx;
    tx.reserve(m.size());

    // 8) FM 调制
    double phi = 0.0;
    for (double v : m) {
        phi += 2.0 * PI * kf * v / fs;
        tx.emplace_back(std::cos(phi), std::sin(phi));
    }

    return tx;
}