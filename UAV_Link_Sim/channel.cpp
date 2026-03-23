#include "channel.h"

#include <random>
#include <algorithm>
#include <cmath>

namespace {
    static std::mt19937& rng() {
        static std::mt19937 gen(123);
        return gen;
    }

    static Complex sample_clamped(const VecComplex& signal, long long idx) {
        if (signal.empty()) return Complex(0.0, 0.0);
        if (idx <= 0) return signal.front();
        if (static_cast<size_t>(idx) >= signal.size()) return signal.back();
        return signal[static_cast<size_t>(idx)];
    }

    static Complex cubic_interp(
        const Complex& y0,
        const Complex& y1,
        const Complex& y2,
        const Complex& y3,
        double mu)
    {
        const Complex a0 = -0.5 * y0 + 1.5 * y1 - 1.5 * y2 + 0.5 * y3;
        const Complex a1 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
        const Complex a2 = -0.5 * y0 + 0.5 * y2;
        const Complex a3 = y1;
        return ((a0 * mu + a1) * mu + a2) * mu + a3;
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

    setSeed(static_cast<unsigned int>(cfg.seed));

    VecComplex y = signal;

    if (cfg.enable_sto && cfg.sto_samp != 0) {
        y = applySTO(y, cfg.sto_samp);
    }

    if (cfg.enable_cfo && std::abs(cfg.cfo_hz) > 0.0) {
        y = applyCFO(y, cfg.cfo_hz, fs, cfg.phase_rad);
    }

    if (cfg.enable_sfo && std::abs(cfg.sfo_ppm) > 0.0) {
        y = applySFO(y, cfg.sfo_ppm);
    }

    if (cfg.enable_awgn) {
        y = awgn(y, cfg.snr_dB);
    }

    return y;
}

VecComplex Channel::applySTO(const VecComplex& signal, int sto_samp) {
    if (signal.empty() || sto_samp == 0) return signal;

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

    const double eps = sfo_ppm * 1e-6;
    const double alpha = 1.0 + eps;

    VecComplex out(signal.size(), Complex(0.0, 0.0));

    for (size_t n = 0; n < out.size(); ++n) {
        const double src_index = static_cast<double>(n) * alpha;

        const long long i1 = static_cast<long long>(std::floor(src_index));
        const long long i0 = i1 - 1;
        const long long i2 = i1 + 1;
        const long long i3 = i1 + 2;
        const double mu = src_index - static_cast<double>(i1);

        const Complex s0 = sample_clamped(signal, i0);
        const Complex s1 = sample_clamped(signal, i1);
        const Complex s2 = sample_clamped(signal, i2);
        const Complex s3 = sample_clamped(signal, i3);
        out[n] = cubic_interp(s0, s1, s2, s3, mu);
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
