#include "frequency_hop.h"
#include <random>
#include <cmath>

// 生成跳频序列 (简单伪随机实现)
VecDouble generate_sequence(int min_freq, int max_freq, int min_spacing, int length, int seed)
{
    VecDouble seq;
    if (length <= 0) return seq;
    if (min_freq > max_freq) return seq;

    std::mt19937 rng(seed);

    // MATLAB:
    // freq_step = 1;
    // freq_pool = (min_freq:freq_step:max_freq) - (max_freq + min_freq)/2;
    //
    // 对于 [-13, 13] 来说，freq_pool 就是 -13, -12, ..., 13（单位 MHz）
    std::vector<int> freq_pool;
    for (int f = min_freq; f <= max_freq; ++f) {
        freq_pool.push_back(f - (max_freq + min_freq) / 2);
    }

    if (freq_pool.empty()) return seq;

    std::uniform_int_distribution<int> dist_all(0, static_cast<int>(freq_pool.size()) - 1);

    seq.reserve(static_cast<size_t>(length));

    // 第一个频点随机选
    int prev_freq_mhz = freq_pool[dist_all(rng)];
    seq.push_back(static_cast<double>(prev_freq_mhz) * 1e6);

    // 后续频点：满足 |f(i)-f(i-1)| >= min_spacing
    for (int i = 1; i < length; ++i)
    {
        std::vector<int> available;
        available.reserve(freq_pool.size());

        for (int f_mhz : freq_pool) {
            if (std::abs(f_mhz - prev_freq_mhz) >= min_spacing) {
                available.push_back(f_mhz);
            }
        }

        if (available.empty()) {
            available = freq_pool;
        }

        std::uniform_int_distribution<int> dist_avail(0, static_cast<int>(available.size()) - 1);
        int cur_freq_mhz = available[dist_avail(rng)];

        seq.push_back(static_cast<double>(cur_freq_mhz) * 1e6);
        prev_freq_mhz = cur_freq_mhz;
    }

    return seq;
}

// 跳频调制
VecComplex frequencyHop(
    const VecComplex& txSig,
    const VecDouble& freq_seq,
    int pulse_len,
    int nt,
    double fs)
{
    VecComplex hopped;
    if (txSig.empty() || freq_seq.empty() || pulse_len <= 0 || nt <= 0 || !std::isfinite(fs) || fs <= 0.0) {
        return hopped;
    }

    const int group_len = pulse_len * nt;
    if (group_len <= 0) return hopped;

    hopped.reserve(txSig.size());

    const size_t num_groups_by_signal =
        (txSig.size() + static_cast<size_t>(group_len) - 1) / static_cast<size_t>(group_len);
    const size_t num_groups = std::min(num_groups_by_signal, freq_seq.size());

    for (size_t i = 0; i < num_groups; ++i)
    {
        const double freq = freq_seq[i];
        const size_t start = i * static_cast<size_t>(group_len);

        for (int j = 0; j < group_len; ++j)
        {
            const size_t idx = start + static_cast<size_t>(j);
            if (idx >= txSig.size()) break;

            // MATLAB FH_temp = 1:pulse_len*nt
            // 所以时间索引更接近 (j+1)/fs，而不是 j/fs
            const double t = static_cast<double>(j + 1) / fs;
            const double ang = 2.0 * PI * freq * t;
            const Complex carrier(std::cos(ang), std::sin(ang));

            hopped.push_back(txSig[idx] * carrier);
        }
    }

    return hopped;
}