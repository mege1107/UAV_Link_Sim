#include "frequency_hop.h"
#include <random>
#include <cmath>

// 生成跳频序列 (简单伪随机实现)
VecDouble generate_sequence(int min_freq, int max_freq, int step, int length, int seed) {
    VecDouble seq;
    std::mt19937 rng(seed);

    // 生成候选频率列表
    VecDouble candidates;
    for (int f = min_freq; f <= max_freq; f += step) {
        candidates.push_back(f * 1e6); // 对应MATLAB *1e6
    }

    // 随机选取
    std::uniform_int_distribution<int> dist(0, candidates.size() - 1);
    for (int i = 0; i < length; ++i) {
        seq.push_back(candidates[dist(rng)]);
    }

    return seq;
}

// 跳频调制
VecComplex frequencyHop(const VecComplex& txSig, const VecDouble& freq_seq, int pulse_len, int nt, double fs) {
    VecComplex hopped;
    int group_len = pulse_len * nt;

    for (size_t i = 0; i < freq_seq.size(); ++i) {
        double freq = freq_seq[i];
        int start = i * group_len;

        // 生成载波
        for (int j = 0; j < group_len && (start + j) < txSig.size(); ++j) {
            double t = j / fs;
            Complex carrier(std::cos(2 * PI * freq * t), std::sin(2 * PI * freq * t));
            hopped.push_back(txSig[start + j] * carrier);
        }
    }

    return hopped;
}