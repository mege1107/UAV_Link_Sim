#include "channel.h"

#include <random>
#include <algorithm> // std::min/std::max

namespace {
    // 随机数引擎：默认用固定种子，便于复现实验；你也可以在 main 里调用 Channel::setSeed 改
    static std::mt19937& rng() {
        static std::mt19937 gen(123);
        return gen;
    }
}

void Channel::setSeed(unsigned int seed) {
    rng().seed(seed);
}

VecComplex Channel::awgn(const VecComplex& signal, double snr_dB) {
    if (signal.empty()) return {};

    // SNR(dB) -> 线性
    const double snr_linear = std::pow(10.0, snr_dB / 10.0);

    // 计算信号平均功率
    double p_sig = 0.0;
    for (const auto& s : signal) p_sig += std::norm(s);
    p_sig /= static_cast<double>(signal.size());

    // 防御：避免 snr_linear 太小/为0 导致溢出
    const double snr_safe = std::max(snr_linear, 1e-12);

    // 复高斯噪声：每个维度方差 = noise_power/2
    const double p_noise = p_sig / snr_safe;
    const double sigma = std::sqrt(p_noise / 2.0);

    std::normal_distribution<double> dist(0.0, sigma);

    VecComplex noisy(signal.size());
    for (size_t i = 0; i < signal.size(); ++i) {
        noisy[i] = signal[i] + Complex(dist(rng()), dist(rng()));
    }

    return noisy;
}