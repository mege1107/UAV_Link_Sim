#include "channel.h"
#include <random>
#include <cmath>

VecComplex Channel::awgn(const VecComplex& signal,
    double snr_dB)
{
    VecComplex noisy(signal.size());

    double snr = std::pow(10.0, snr_dB / 10.0);

    // 计算信号功率
    double power = 0.0;
    for (const auto& s : signal)
        power += std::norm(s);
    power /= signal.size();

    double noise_power = power / snr;
    double sigma = std::sqrt(noise_power / 2.0);

    std::mt19937 rng(123);
    std::normal_distribution<double> dist(0.0, sigma);

    for (size_t i = 0; i < signal.size(); ++i)
    {
        noisy[i] = signal[i] +
            Complex(dist(rng), dist(rng));
    }

    return noisy;
}