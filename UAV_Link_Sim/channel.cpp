#include "channel.h"

#include <random>
#include <algorithm>
#include <cmath>

namespace {
    static std::mt19937& rng() {
        static std::mt19937 gen(123);
        return gen;
    }
}

void Channel::setSeed(unsigned int seed) {
    rng().seed(seed);
}

VecComplex Channel::process(const VecComplex& signal,
    const ChannelConfig& cfg,
    double fs)
{
    if (signal.empty()) return {};

    // 用配置里的 seed 保证可复现
    setSeed(static_cast<unsigned int>(cfg.seed));

    VecComplex y = signal;

    // 1) STO
    if (cfg.enable_sto && cfg.sto_samp != 0) {
        y = applySTO(y, cfg.sto_samp);
    }

    // 2) CFO
    if (cfg.enable_cfo && std::abs(cfg.cfo_hz) > 0.0) {
        y = applyCFO(y, cfg.cfo_hz, fs, cfg.phase_rad);
    }

    // 3) SFO
    if (cfg.enable_sfo && std::abs(cfg.sfo_ppm) > 0.0) {
        y = applySFO(y, cfg.sfo_ppm);
    }

    // 4) AWGN
    if (cfg.enable_awgn) {
        y = awgn(y, cfg.snr_dB);
    }

    return y;
}

VecComplex Channel::applySTO(const VecComplex& signal, int sto_samp) {
    if (signal.empty() || sto_samp == 0) return signal;

    // 只按你的要求支持“整数 STO”
    // 这里定义为：前面补 sto_samp 个 0，长度保持不变
    // 相当于信号整体向后平移，尾部被截断
    if (sto_samp > 0) {
        VecComplex out(signal.size(), Complex(0.0, 0.0));

        const size_t d = static_cast<size_t>(sto_samp);
        if (d >= signal.size()) {
            return out;
        }

        for (size_t i = 0; i + d < signal.size(); ++i) {
            out[i + d] = signal[i];
        }
        return out;
    }

    // 如果你后面想支持“负 STO”，这里也顺手兼容了：
    // sto_samp < 0 表示整体提前，前部截掉，尾部补零
    const size_t d = static_cast<size_t>(-sto_samp);
    VecComplex out(signal.size(), Complex(0.0, 0.0));

    if (d >= signal.size()) {
        return out;
    }

    for (size_t i = d; i < signal.size(); ++i) {
        out[i - d] = signal[i];
    }
    return out;
}

VecComplex Channel::applyCFO(const VecComplex& signal,
    double cfo_hz,
    double fs,
    double phase_rad)
{
    if (signal.empty()) return signal;
    if (std::abs(cfo_hz) < 1e-15) return signal;
    if (fs <= 0.0) return signal;

    VecComplex out(signal.size());

    const double w = 2.0 * M_PI * cfo_hz / fs;
    for (size_t n = 0; n < signal.size(); ++n) {
        const double theta = w * static_cast<double>(n) + phase_rad;
        const Complex rot(std::cos(theta), std::sin(theta));
        out[n] = signal[n] * rot;
    }

    return out;
}

VecComplex Channel::applySFO(const VecComplex& signal, double sfo_ppm) {
    if (signal.empty()) return signal;
    if (std::abs(sfo_ppm) < 1e-15) return signal;

    // ppm -> 相对失配
    // eps > 0 表示接收端采样率偏高/偏低的等效失配，这里采用统一重采样模型：
    // y[n] = x[n * (1 + eps)]
    const double eps = sfo_ppm * 1e-6;
    const double alpha = 1.0 + eps;

    VecComplex out(signal.size(), Complex(0.0, 0.0));

    for (size_t n = 0; n < out.size(); ++n) {
        const double src_index = static_cast<double>(n) * alpha;

        // 线性插值
        const long long i0 = static_cast<long long>(std::floor(src_index));
        const long long i1 = i0 + 1;
        const double mu = src_index - static_cast<double>(i0);

        if (i0 < 0) {
            out[n] = Complex(0.0, 0.0);
        }
        else if (static_cast<size_t>(i0) >= signal.size()) {
            out[n] = Complex(0.0, 0.0);
        }
        else if (static_cast<size_t>(i1) >= signal.size()) {
            out[n] = signal[static_cast<size_t>(i0)];
        }
        else {
            const Complex s0 = signal[static_cast<size_t>(i0)];
            const Complex s1 = signal[static_cast<size_t>(i1)];
            out[n] = (1.0 - mu) * s0 + mu * s1;
        }
    }

    return out;
}

VecComplex Channel::awgn(const VecComplex& signal, double snr_dB) {
    if (signal.empty()) return {};

    const double snr_linear = std::pow(10.0, snr_dB / 10.0);

    double p_sig = 0.0;
    for (const auto& s : signal) {
        p_sig += std::norm(s);
    }
    p_sig /= static_cast<double>(signal.size());

    const double snr_safe = std::max(snr_linear, 1e-12);
    const double p_noise = p_sig / snr_safe;
    const double sigma = std::sqrt(p_noise / 2.0);

    std::normal_distribution<double> dist(0.0, sigma);

    VecComplex noisy(signal.size());
    for (size_t i = 0; i < signal.size(); ++i) {
        noisy[i] = signal[i] + Complex(dist(rng()), dist(rng()));
    }

    return noisy;
}