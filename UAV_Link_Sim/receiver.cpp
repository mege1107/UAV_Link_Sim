#include "receiver.h"
#include "coding.h"
#include "spread_spectrum.h"
#include "sync.h"
#include "channel.h"
#include "utils.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <algorithm>
#include <numeric>

static inline Complex cexpj(double a) {
    return Complex(std::cos(a), std::sin(a));
}

namespace {
    int g_linear_decision_phase = 0;
}

// =============== 一些本地辅助函数 ===============

static std::vector<double> designLowpassFIR(int num_taps, double cutoff_norm)
{
    // cutoff_norm: 相对 Nyquist 归一化频率，范围 (0, 1)
    if (num_taps < 3) num_taps = 3;
    if ((num_taps & 1) == 0) num_taps += 1; // 用奇数 taps 更方便
    cutoff_norm = std::max(1e-4, std::min(0.999, cutoff_norm));

    std::vector<double> h((size_t)num_taps, 0.0);
    const int M = num_taps - 1;
    const double fc = cutoff_norm; // 相对 Nyquist

    for (int n = 0; n < num_taps; ++n) {
        const int m = n - M / 2;

        double sinc_val = 0.0;
        if (m == 0) {
            sinc_val = fc;
        }
        else {
            sinc_val = std::sin(PI * fc * m) / (PI * m);
        }

        // Hamming 窗
        const double w = 0.54 - 0.46 * std::cos(2.0 * PI * n / M);
        h[(size_t)n] = sinc_val * w;
    }

    // 归一化 DC 增益
    double sum_h = std::accumulate(h.begin(), h.end(), 0.0);
    if (std::abs(sum_h) > 1e-12) {
        for (auto& v : h) v /= sum_h;
    }

    return h;
}

static VecComplex applyFIRAndAlign(const VecComplex& x, const std::vector<double>& h)
{
    if (x.empty() || h.empty()) return x;

    const int L = (int)h.size();
    const int delay = (L - 1) / 2;

    VecComplex y(x.size() + (size_t)L - 1, Complex(0.0, 0.0));

    for (size_t n = 0; n < y.size(); ++n) {
        Complex acc(0.0, 0.0);
        for (int k = 0; k < L; ++k) {
            if ((int)n - k < 0) continue;
            const size_t idx = (size_t)((int)n - k);
            if (idx >= x.size()) continue;
            acc += x[idx] * h[(size_t)k];
        }
        y[n] = acc;
    }

    // 群时延补偿，输出长度对齐回原长度
    VecComplex out;
    out.reserve(x.size());
    for (size_t i = 0; i < x.size(); ++i) {
        const size_t src = i + (size_t)delay;
        if (src < y.size()) out.push_back(y[src]);
        else out.push_back(Complex(0.0, 0.0));
    }
    return out;
}

static VecComplex applyRealFIRSame(const VecComplex& x, const VecDouble& h)
{
    if (x.empty() || h.empty()) return {};

    VecComplex y(x.size(), Complex(0.0, 0.0));

    for (size_t n = 0; n < x.size(); ++n) {
        Complex acc(0.0, 0.0);
        const size_t kmax = std::min(n, h.size() - 1);
        for (size_t k = 0; k <= kmax; ++k) {
            acc += x[n - k] * h[k];
        }
        y[n] = acc;
    }

    return y;
}

static VecComplex applySRRCMatchedFilterAndAlign(
    const VecComplex& x, int sps, double beta = 0.5, int span = 6)
{
    if (x.empty() || sps <= 0) return {};

    VecDouble rrc = designSRRC(beta, span, sps);
    const int filter_delay = (int)(rrc.size() - 1) / 2;

    VecComplex padded = x;
    padded.insert(padded.end(), (size_t)filter_delay, Complex(0.0, 0.0));

    VecComplex y = applyRealFIRSame(padded, rrc);
    if (y.size() <= (size_t)filter_delay) return {};

    return VecComplex(y.begin() + filter_delay, y.end());
}

static double computePercentile(std::vector<double> v, double p01)
{
    if (v.empty()) return 0.0;
    p01 = std::max(0.0, std::min(1.0, p01));
    const size_t k = (size_t)std::floor(p01 * (double)(v.size() - 1));
    std::nth_element(v.begin(), v.begin() + (long long)k, v.end());
    return v[k];
}

static double estimateOOKThresholdAdaptive(const VecComplex& rx)
{
    if (rx.empty()) return 0.5;

    std::vector<double> amp;
    amp.reserve(rx.size());
    for (const auto& v : rx) amp.push_back(std::abs(v));

    const double p = 1e-3;
    const double th_low = computePercentile(amp, 0.10);

    double sum_noise = 0.0;
    int cnt_noise = 0;
    for (double a : amp) {
        if (a <= th_low) {
            sum_noise += a;
            cnt_noise++;
        }
    }

    double noise_level = (cnt_noise > 0) ? (sum_noise / cnt_noise) : 0.0;
    double Thsd = noise_level * std::sqrt(std::log(1.0 / p));

    if (!std::isfinite(Thsd) || Thsd <= 0.0) {
        double mean_amp = std::accumulate(amp.begin(), amp.end(), 0.0) / (double)amp.size();
        Thsd = 0.3 * mean_amp;
    }

    return Thsd;
}

static void normalizeQAMSymbolsAdaptive(VecComplex& syms)
{
    if (syms.empty()) return;

    double avg_power = 0.0;
    for (const auto& v : syms) avg_power += std::norm(v);
    avg_power /= (double)syms.size();

    if (avg_power > 1e-12 && std::isfinite(avg_power)) {
        const double scale = 1.0 / std::sqrt(avg_power);
        for (auto& v : syms) v *= scale;
    }

    double mean_abs_I = 0.0;
    double mean_abs_Q = 0.0;
    for (const auto& v : syms) {
        mean_abs_I += std::abs(v.real());
        mean_abs_Q += std::abs(v.imag());
    }
    mean_abs_I /= (double)syms.size();
    mean_abs_Q /= (double)syms.size();

    const double target = 2.0 / std::sqrt(10.0);

    double scale_I = 1.0;
    double scale_Q = 1.0;
    if (mean_abs_I > 1e-12 && std::isfinite(mean_abs_I)) scale_I = target / mean_abs_I;
    if (mean_abs_Q > 1e-12 && std::isfinite(mean_abs_Q)) scale_Q = target / mean_abs_Q;

    for (auto& v : syms) {
        v = Complex(v.real() * scale_I, v.imag() * scale_Q);
    }
}

// =============== 归一化相关 metric（给定绝对位置） ===============
static double metricAtNormalized(
    const VecComplex& rx,
    size_t abs_idx,
    const VecComplex& sync
) {
    const size_t N = rx.size();
    const size_t M = sync.size();
    if (M == 0 || abs_idx + M > N) return -1.0;

    Complex dot(0.0, 0.0);
    double Es = 0.0, Er = 0.0;

    for (size_t k = 0; k < M; ++k) {
        const auto& s = sync[k];
        const auto& r = rx[abs_idx + k];
        dot += std::conj(s) * r;
        Es += std::norm(s);
        Er += std::norm(r);
    }
    if (Es <= 1e-12 || Er <= 1e-12) return -1.0;
    return std::norm(dot) / (Es * Er);
}

// =============== 窄窗内滑动找最大相关 ===============
static bool findSyncStartNormalizedWindow(
    const VecComplex& rx,
    size_t win_start,
    size_t win_end, // [start, end)
    const VecComplex& sync,
    int& best_rel_idx,
    double& best_metric
) {
    const size_t N = rx.size();
    const size_t M = sync.size();
    if (M == 0) return false;
    if (win_start >= win_end) return false;
    if (win_end > N) win_end = N;
    if (win_start + M > win_end) return false;

    best_metric = -1.0;
    best_rel_idx = 0;

    const size_t K = (win_end - win_start) - M;
    for (size_t i = 0; i <= K; ++i) {
        const size_t abs_idx = win_start + i;

        Complex dot(0.0, 0.0);
        double Es = 0.0, Er = 0.0;

        for (size_t k = 0; k < M; ++k) {
            const auto& s = sync[k];
            const auto& r = rx[abs_idx + k];
            dot += std::conj(s) * r;
            Es += std::norm(s);
            Er += std::norm(r);
        }
        if (Es <= 1e-12 || Er <= 1e-12) continue;

        double metric = std::norm(dot) / (Es * Er);
        if (metric > best_metric) {
            best_metric = metric;
            best_rel_idx = (int)i;
        }
    }
    return best_metric > 0.0;
}

static double estimateCoarseCFOFromRepeatedSync(
    const VecComplex& rx,
    size_t coarse_block_len,
    int repeat_count,
    double fs,
    size_t& best_start,
    double& best_score)
{
    best_start = 0;
    best_score = 0.0;

    if (rx.empty() || coarse_block_len == 0 || repeat_count < 2) return 0.0;
    if (!std::isfinite(fs) || fs <= 0.0) return 0.0;

    const size_t span = coarse_block_len * (size_t)repeat_count;
    if (span > rx.size()) return 0.0;

    Complex best_acc(0.0, 0.0);
    const size_t last_start = rx.size() - span;

    for (size_t start = 0; start <= last_start; ++start) {
        Complex acc(0.0, 0.0);
        double energy = 0.0;

        for (int rep = 0; rep < repeat_count - 1; ++rep) {
            const size_t a0 = start + (size_t)rep * coarse_block_len;
            const size_t b0 = a0 + coarse_block_len;
            for (size_t k = 0; k < coarse_block_len; ++k) {
                const Complex& a = rx[a0 + k];
                const Complex& b = rx[b0 + k];
                acc += std::conj(a) * b;
                energy += std::norm(a) + std::norm(b);
            }
        }

        if (energy <= 1e-12) continue;

        const double score = std::abs(acc) / energy;
        if (score > best_score) {
            best_score = score;
            best_start = start;
            best_acc = acc;
        }
    }

    if (best_score <= 0.0) return 0.0;

    const double phase = std::atan2(best_acc.imag(), best_acc.real());
    const double delta_t = (double)coarse_block_len / fs;
    if (!std::isfinite(delta_t) || delta_t <= 0.0) return 0.0;

    double f_coarse = phase / (2.0 * PI * delta_t);
    const double max_freq_offset = fs / (2.0 * (double)coarse_block_len);
    f_coarse = clamp_value(f_coarse, -max_freq_offset, max_freq_offset);

    if (!std::isfinite(f_coarse)) return 0.0;
    return f_coarse;
}

static VecComplex applyFrequencyCorrection(
    const VecComplex& signal,
    double fs,
    double freq_hz)
{
    if (signal.empty()) return {};
    if (!std::isfinite(fs) || fs <= 0.0) return signal;
    if (!std::isfinite(freq_hz) || std::abs(freq_hz) < 1e-12) return signal;

    VecComplex out(signal.size());
    for (size_t n = 0; n < signal.size(); ++n) {
        const double ang = 2.0 * PI * freq_hz * (double)n / fs;
        out[n] = signal[n] * std::conj(cexpj(ang));
    }
    return out;
}

static VecComplex shiftSignalLeftWithZeroPad(
    const VecComplex& signal,
    size_t shift_samp)
{
    if (signal.empty() || shift_samp == 0) return signal;

    VecComplex out(signal.size(), Complex(0.0, 0.0));
    if (shift_samp >= signal.size()) return out;

    const size_t remain = signal.size() - shift_samp;
    for (size_t i = 0; i < remain; ++i) {
        out[i] = signal[i + shift_samp];
    }
    return out;
}

static int chooseBestLinearDecisionPhase(
    const VecComplex& rx,
    int s)
{
    if (rx.empty() || s <= 1) return 0;

    int best_phase = 0;
    double best_score = -1.0;

    for (int phase = 0; phase < s; ++phase) {
        double score = 0.0;
        int count = 0;

        for (size_t idx = (size_t)phase; idx < rx.size(); idx += (size_t)s) {
            score += std::norm(rx[idx]);
            count++;
        }

        if (count > 0) score /= (double)count;
        if (score > best_score) {
            best_score = score;
            best_phase = phase;
        }
    }

    return best_phase;
}

static VecComplex sampleLinearSymbolsAtPhase(
    const VecComplex& rx,
    int s,
    int phase)
{
    VecComplex syms;
    if (rx.empty() || s <= 0) return syms;
    if (phase < 0) phase = 0;
    if (phase >= s) phase = s - 1;
    if ((size_t)phase >= rx.size()) return syms;

    syms.reserve((rx.size() + (size_t)s - 1) / (size_t)s);
    for (size_t idx = (size_t)phase; idx < rx.size(); idx += (size_t)s) {
        syms.push_back(rx[idx]);
    }
    return syms;
}

struct CFOBranchResolveResult {
    double resolved_cfo_hz = 0.0;
    double branch_metric = -1.0;
    size_t first_candidate_idx = 0;
    int branch_k = 0;
    VecComplex corrected_signal;
};

static CFOBranchResolveResult resolveCoarseCFOAmbiguity(
    const VecComplex& rx,
    const VecComplex& sync,
    double coarse_cfo_hz,
    double fs,
    size_t coarse_block_len,
    double first_sync_threshold,
    size_t first_refine_W)
{
    CFOBranchResolveResult best;
    best.resolved_cfo_hz = coarse_cfo_hz;
    best.corrected_signal = rx;

    if (rx.empty() || sync.empty() || coarse_block_len == 0) return best;
    if (!std::isfinite(fs) || fs <= 0.0) return best;

    const double ambiguity_step_hz = fs / (double)coarse_block_len;
    const int branch_candidates[] = { -1, 0, 1 };

    for (int k : branch_candidates) {
        const double cand_cfo_hz = coarse_cfo_hz + (double)k * ambiguity_step_hz;
        VecComplex cand_signal = applyFrequencyCorrection(rx, fs, cand_cfo_hz);

        bool found_candidate = false;
        size_t cand_idx = 0;
        const size_t search_end = cand_signal.size() - sync.size();
        for (size_t i = 0; i <= search_end; ++i) {
            double m = metricAtNormalized(cand_signal, i, sync);
            if (m >= first_sync_threshold) {
                cand_idx = i;
                found_candidate = true;
                break;
            }
        }

        if (!found_candidate) continue;

        size_t win_start = (cand_idx > first_refine_W) ? (cand_idx - first_refine_W) : 0;
        size_t win_end = std::min(cand_signal.size(), cand_idx + first_refine_W + sync.size());

        int best_rel = 0;
        double best_m = 0.0;
        if (!findSyncStartNormalizedWindow(cand_signal, win_start, win_end, sync, best_rel, best_m)) {
            continue;
        }

        if (best_m < first_sync_threshold) continue;

        const bool better_metric = (best_m > best.branch_metric + 1e-12);
        const bool same_metric_earlier =
            (std::abs(best_m - best.branch_metric) <= 1e-12 && cand_idx < best.first_candidate_idx);

        if (best.branch_metric < 0.0 || better_metric || same_metric_earlier) {
            best.resolved_cfo_hz = cand_cfo_hz;
            best.branch_metric = best_m;
            best.first_candidate_idx = cand_idx;
            best.branch_k = k;
            best.corrected_signal = std::move(cand_signal);
        }
    }

    return best;
}

Receiver::Receiver(const TransmitterConfig& config)
    : config_(config)
{
    resetFreqHistory();
}

// ====================== 频偏估计 ======================
double Receiver::estimateCFOFromPreamble(const VecComplex& rx_preamble,
    const VecComplex& local_preamble,
    double fs)
{
    if (rx_preamble.size() != local_preamble.size() || rx_preamble.empty()) return 0.0;
    if (!std::isfinite(fs) || fs <= 0) return 0.0;

    const int N = (int)rx_preamble.size();
    const int half_len = N / 2;
    if (half_len <= 0) return 0.0;

    Complex R1(0.0, 0.0), R2(0.0, 0.0);
    for (int i = 0; i < half_len; ++i) {
        R1 += std::conj(local_preamble[(size_t)i]) * rx_preamble[(size_t)i];
        R2 += std::conj(local_preamble[(size_t)i + (size_t)half_len]) * rx_preamble[(size_t)i + (size_t)half_len];
    }

    double phase_diff = std::atan2(R2.imag(), R2.real()) - std::atan2(R1.imag(), R1.real());
    while (phase_diff > PI)  phase_diff -= 2 * PI;
    while (phase_diff < -PI) phase_diff += 2 * PI;

    const double delta_t = (double)half_len / fs;
    if (!std::isfinite(delta_t) || delta_t <= 0) return 0.0;

    double freq_offset_hz = phase_diff / (2 * PI * delta_t);

    const double max_freq_offset = fs / 20.0;
    freq_offset_hz = clamp_value(freq_offset_hz, -max_freq_offset, max_freq_offset);
    if (!std::isfinite(freq_offset_hz)) freq_offset_hz = 0.0;

    return freq_offset_hz;
}

// ====================== 线性最小二乘拟合 ======================
void Receiver::linearFitLSQ(const std::vector<int>& x,
    const std::vector<double>& y,
    double& out_a,
    double& out_b)
{
    if (x.size() != y.size() || x.size() < 2) {
        out_a = 0.0;
        out_b = y.empty() ? 0.0 : y.back();
        return;
    }

    const int n = (int)x.size();
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;

    for (int i = 0; i < n; ++i) {
        sum_x += x[(size_t)i];
        sum_y += y[(size_t)i];
        sum_xy += x[(size_t)i] * y[(size_t)i];
        sum_x2 += (double)x[(size_t)i] * (double)x[(size_t)i];
    }

    double denom = n * sum_x2 - sum_x * sum_x;
    if (std::abs(denom) < 1e-12 || !std::isfinite(denom)) {
        out_a = 0.0;
        out_b = sum_y / n;
        if (!std::isfinite(out_b)) out_b = 0.0;
        return;
    }

    out_a = (n * sum_xy - sum_x * sum_y) / denom;
    out_b = (sum_y - out_a * sum_x) / n;

    if (!std::isfinite(out_a)) out_a = 0.0;
    if (!std::isfinite(out_b)) out_b = 0.0;
}

// ====================== 残余相位估计 ======================
double Receiver::estimateResidualPhase(const VecComplex& rx_preamble,
    const VecComplex& local_preamble)
{
    if (rx_preamble.size() != local_preamble.size() || rx_preamble.empty()) return 0.0;

    Complex acc(0.0, 0.0);
    for (size_t i = 0; i < rx_preamble.size(); ++i) {
        acc += rx_preamble[i] * std::conj(local_preamble[i]);
    }

    double ph = std::atan2(acc.imag(), acc.real());
    if (!std::isfinite(ph)) ph = 0.0;
    return ph;
}

void Receiver::resetFreqHistory() {
    freq_offset_history_.clear();
    frame_index_history_.clear();
    fit_a_ = 0.0;
    fit_b_ = 0.0;
    frame_count_ = 0;
}

// ====================== 每组求均值（保留备用） ======================
VecComplex Receiver::meanGroups(const VecComplex& rx, int s) const
{
    VecComplex out;
    if (s <= 0) return out;

    const size_t nsym = rx.size() / (size_t)s;
    out.reserve(nsym);

    for (size_t k = 0; k < nsym; ++k) {
        Complex acc(0.0, 0.0);
        for (int j = 0; j < s; ++j) {
            acc += rx[k * (size_t)s + (size_t)j];
        }
        out.push_back(acc / (double)s);
    }

    return out;
}

// ====================== 取每个 symbol 中点采样（保留备用） ======================
Complex Receiver::pickMidSampleInSymbol(const VecComplex& rx, size_t base, int s) const
{
    if (s <= 0) return Complex(0.0, 0.0);
    size_t idx = base + (size_t)(s / 2);
    if (idx >= rx.size()) return Complex(0.0, 0.0);
    return rx[idx];
}

// ====================== payload长度：按调制方式计算 ======================
int Receiver::getTelemetryPayloadSampleCount(int ccsk_chip_num) const
{
    const int s = std::max(1, config_.samp);

    switch (config_.modulation) {
    case ModulationType::BPSK:
    case ModulationType::OOK:
    case ModulationType::FSK:
    case ModulationType::FM:
    case ModulationType::MSK:
        return ccsk_chip_num * s;

    case ModulationType::QPSK:
        return ((ccsk_chip_num + 1) / 2) * s;

    case ModulationType::QAM:
        return ((ccsk_chip_num + 3) / 4) * s;

    default:
        return ccsk_chip_num * s;
    }
}

// ====================== 哪些调制需要差分解码 ======================
bool Receiver::modulationNeedsDiffDecode() const
{
    return (config_.modulation == ModulationType::BPSK);
}

// ====================== 统一解调入口 ======================
VecInt Receiver::demodulateTelemetry(const VecComplex& rx)
{
    switch (config_.modulation) {
    case ModulationType::BPSK:
        return demodulateBPSK(rx);

    case ModulationType::QPSK:
        return demodulateQPSK(rx);

    case ModulationType::QAM:
        return demodulateQAM16(rx);

    case ModulationType::OOK:
        return demodulateOOK(rx);

    case ModulationType::FSK:
        return demodulateFSK(rx);

    case ModulationType::FM:
        return demodulateFM(rx);

    case ModulationType::MSK:
        return demodulateMSK(rx);

    default:
        std::cerr << "[Receiver] Modulation scheme not implemented in current receiver\n";
        return {};
    }
}

// ====================== 主接收函数（锁帧+窄窗同步） ======================
VecInt Receiver::receive(const VecComplex& rx_signal)
{
    last_constellation_points_.clear();

    const double fs = config_.fs;
    if (!std::isfinite(fs) || fs <= 0) {
        std::cerr << "[Receiver] ERROR: invalid fs=" << fs << "\n";
        return {};
    }
    if (config_.samp <= 0) {
        std::cerr << "[Receiver] ERROR: invalid samp=" << config_.samp << "\n";
        return {};
    }

    // 1) 同步序列
    VecInt m_coarse = mseq({ 1,1,1,0,1 });
    VecComplex sync_wide = generateCoarseSync(m_coarse, config_.samp, config_.coarse_length);

    VecInt pss1 = mseq({ 1,0,1,0,1,1,1,0,1 });
    VecInt pss2 = mseq({ 1,0,1,0,1,0,1 });
    VecComplex sync_fine = generateFineSync(pss1, pss2, config_.fine_length);

    VecComplex SYNC = concatSync(sync_wide, sync_fine);
    const int sync_len = (int)SYNC.size();
    const int fine_sync_len = (int)sync_fine.size();
    const size_t coarse_block_len =
        sync_wide.empty() ? 0 : (sync_wide.size() / (size_t)std::max(1, config_.coarse_length));

    // 2) 单帧长度（按 RS 编码后的 payload 算）
    const int source_bits = config_.frame_bit * config_.n;
    const int rs_msg_bits = 15 * 5;   // 75
    const int rs_code_bits = 31 * 5;  // 155

    if (source_bits % rs_msg_bits != 0) {
        std::cerr << "[Receiver] ERROR: source_bits is not a multiple of 75\n";
        return {};
    }

    const int rs_blocks = source_bits / rs_msg_bits;
    const int encoded_bits = rs_blocks * rs_code_bits;

    // CCSK(32,5): 每 5 bit -> 32 chips
    const int ccsk_chip_num = (encoded_bits / 5) * 32;

    const int s = std::max(1, config_.samp);
    const int zp_samp_num = config_.zp_sym * s;

    int payload_samp_num = 0;
    int total_pulses_dbg = 0;

    if (config_.function == FunctionType::RemoteControl) {
        const int pulse_len = (int)config_.ccskcode.size() * s; // 32*samp
        const int gap_len = 33 * s;
        const int total_pulses = ccsk_chip_num / 32;
        total_pulses_dbg = total_pulses;
        payload_samp_num = total_pulses * (pulse_len + gap_len);
    }
    else {
        payload_samp_num = getTelemetryPayloadSampleCount(ccsk_chip_num);
    }

    const int single_frame_total_samp = sync_len + payload_samp_num + zp_samp_num;

    std::cout << "[DBG] ccsk_chip_num=" << ccsk_chip_num
        << " total_pulses=" << total_pulses_dbg
        << " payload_samp_num=" << payload_samp_num
        << " single_frame_total_samp=" << single_frame_total_samp
        << "\n";

    if (rx_signal.size() < (size_t)single_frame_total_samp) {
        std::cerr << "[Receiver] Signal length insufficient for one frame, cannot demodulate\n";
        return {};
    }

    VecComplex rx_sync_signal = rx_signal;
    size_t coarse_cfo_start = 0;
    double coarse_cfo_score = 0.0;
    double coarse_cfo_hz_aliased = estimateCoarseCFOFromRepeatedSync(
        rx_signal,
        coarse_block_len,
        config_.coarse_length,
        fs,
        coarse_cfo_start,
        coarse_cfo_score);

    const double sync_threshold = 0.08;
    const size_t W = (size_t)(2 * s);

    const double first_sync_threshold = 0.4;
    const size_t first_refine_W = (size_t)(8 * s);

    CFOBranchResolveResult cfo_branch = resolveCoarseCFOAmbiguity(
        rx_signal,
        SYNC,
        coarse_cfo_hz_aliased,
        fs,
        coarse_block_len,
        first_sync_threshold,
        first_refine_W);

    double coarse_cfo_hz = cfo_branch.resolved_cfo_hz;
    if (!cfo_branch.corrected_signal.empty()) {
        rx_sync_signal = std::move(cfo_branch.corrected_signal);
    }
    else if (std::abs(coarse_cfo_hz) >= 1.0) {
        rx_sync_signal = applyFrequencyCorrection(rx_signal, fs, coarse_cfo_hz);
    }

    std::cout << "[DBG][ACQ_CFO] coarse_block_len=" << coarse_block_len
        << " coarse_cfo_start=" << coarse_cfo_start
        << " coarse_cfo_score=" << coarse_cfo_score
        << " coarse_cfo_hz_aliased=" << coarse_cfo_hz_aliased
        << " coarse_cfo_branch_k=" << cfo_branch.branch_k
        << " coarse_cfo_branch_metric=" << cfo_branch.branch_metric
        << " coarse_cfo_hz_resolved=" << coarse_cfo_hz
        << " sync_input=" << ((std::abs(coarse_cfo_hz) >= 1.0) ? "pre_corrected" : "raw")
        << "\n";

    bool first_found_candidate = false;
    size_t first_cand_idx = 0;

    const size_t first_search_end = rx_sync_signal.size() - (size_t)sync_len;
    for (size_t i = 0; i <= first_search_end; ++i) {
        double m = metricAtNormalized(rx_sync_signal, i, SYNC);
        if (m >= first_sync_threshold) {
            first_cand_idx = i;
            first_found_candidate = true;
            break;
        }
    }

    if (!first_found_candidate) {
        std::cout << "[Receiver] No sync candidate above threshold found for first frame, demodulation ended\n";
        return {};
    }

    size_t first_win_start = (first_cand_idx > first_refine_W) ? (first_cand_idx - first_refine_W) : 0;
    size_t first_win_end = std::min(rx_sync_signal.size(), first_cand_idx + first_refine_W + (size_t)sync_len);

    int first_best_rel = 0;
    double first_best_m = 0.0;
    if (!findSyncStartNormalizedWindow(rx_sync_signal, first_win_start, first_win_end, SYNC, first_best_rel, first_best_m)) {
        std::cout << "[Receiver] Fine search near first frame candidate failed, demodulation ended\n";
        return {};
    }

    const size_t estimated_sto_samp = first_win_start + (size_t)first_best_rel;
    if (first_best_m < first_sync_threshold) {
        std::cout << "[Receiver] First frame sync peak too low (" << first_best_m << "), demodulation ended\n";
        return {};
    }

    std::cout << "[DBG][STO] estimated_sto_samp=" << estimated_sto_samp
        << " first_cand_idx=" << first_cand_idx
        << " first_metric=" << first_best_m
        << "\n";

    VecComplex rx_aligned_signal = shiftSignalLeftWithZeroPad(rx_sync_signal, estimated_sto_samp);

    VecInt total_rx_bits;
    size_t current_offset = 0;
    const size_t total_rx_len = rx_aligned_signal.size();
    double sfo_hat_ppm = 0.0;
    int sfo_track_count = 0;

    while (current_offset + (size_t)single_frame_total_samp <= total_rx_len) {

        size_t sync_abs_start = current_offset;
        double sync_metric = -1.0;
        bool ok = false;

        size_t win_start = (current_offset > W) ? (current_offset - W) : 0;
        size_t win_end = std::min(total_rx_len, current_offset + W + (size_t)sync_len);

        int best_rel = 0;
        double best_m = 0.0;
        if (!findSyncStartNormalizedWindow(rx_aligned_signal, win_start, win_end, SYNC, best_rel, best_m)) {
            std::cout << "[Receiver] No valid sync header detected, demodulation ended\n";
            break;
        }

        sync_abs_start = win_start + (size_t)best_rel;
        sync_metric = best_m;
        ok = (sync_metric >= sync_threshold);

        if (!ok) {
            std::cout << "[Receiver] Sync peak too low (" << sync_metric << "), demodulation ended\n";
            break;
        }

        if (frame_count_ == 0) {
            std::cout << "[DBG][FIRST] aligned_sync_abs_start=" << sync_abs_start
                << " metric=" << sync_metric
                << "\n";
        }

        const size_t frame_abs_end = sync_abs_start + (size_t)single_frame_total_samp;
        if (frame_abs_end > total_rx_len) {
            std::cout << "[Receiver] Remaining samples insufficient for a full frame, demodulation ended\n";
            break;
        }

        std::cout << "[DBG] current_offset=" << current_offset
            << " sync_abs_start=" << sync_abs_start
            << " drift=" << (long long)sync_abs_start - (long long)current_offset
            << " metric=" << sync_metric
            << "\n";

        if (frame_count_ > 0 && sync_abs_start > 0) {
            const double nominal_start = (double)frame_count_ * (double)single_frame_total_samp;
            const double cumulative_drift = (double)sync_abs_start - nominal_start;
            const double sfo_inst_ppm = (nominal_start > 1.0)
                ? (cumulative_drift / nominal_start) * 1e6
                : 0.0;
            const double sfo_inst_ppm_clamped = clamp_value(sfo_inst_ppm, -200.0, 200.0);

            if (sfo_track_count == 0) {
                sfo_hat_ppm = sfo_inst_ppm_clamped;
            }
            else {
                sfo_hat_ppm = 0.8 * sfo_hat_ppm + 0.2 * sfo_inst_ppm_clamped;
            }
            sfo_track_count++;
        }

        VecComplex current_frame(rx_aligned_signal.begin() + sync_abs_start,
            rx_aligned_signal.begin() + frame_abs_end);

        // ====== CFO + 相位校正 ======
        VecComplex rx_fine_sync(current_frame.begin() + (sync_len - fine_sync_len),
            current_frame.begin() + sync_len);

        double frame_freq_offset = estimateCFOFromPreamble(rx_fine_sync, sync_fine, fs);
        frame_count_++;

        freq_offset_history_.push_back(frame_freq_offset);
        frame_index_history_.push_back(frame_count_);

        linearFitLSQ(frame_index_history_, freq_offset_history_, fit_a_, fit_b_);

        double f_hat = fit_a_ * frame_count_ + fit_b_;
        if (!std::isfinite(f_hat)) f_hat = 0.0;

        // ===== 只对 BPSK / QPSK / QAM 开 CFO =====
        const bool need_cfo_correction =
            (config_.modulation == ModulationType::BPSK ||
                config_.modulation == ModulationType::QPSK ||
                config_.modulation == ModulationType::QAM);

        // ===== 带门限的 CFO 补偿 =====
        // 如果 USRP 上仍然误补太多，可以调到 300~500 Hz
        const double cfo_deadband_hz = 200.0;

        // 二选一：
        // 方案A：用拟合值
        double f_use = f_hat;

        // 方案B：如果你后面测试发现拟合反而不稳，就改成下面这一句
        // double f_use = frame_freq_offset;

        // 死区门限：小于门限就不补
        if (std::abs(f_use) < cfo_deadband_hz || !std::isfinite(f_use)) {
            f_use = 0.0;
        }

        const double total_cfo_est_hz = coarse_cfo_hz + frame_freq_offset;
        const double total_cfo_fit_hz = coarse_cfo_hz + f_hat;

        if (config_.enable_cfo && need_cfo_correction && f_use != 0.0) {
            for (size_t n = 0; n < current_frame.size(); ++n) {
                double t = (double)n / fs;
                double ang = 2.0 * PI * f_use * t;
                if (!std::isfinite(ang)) continue;
                current_frame[n] *= std::conj(Complex(std::cos(ang), std::sin(ang)));
            }
        }

        VecComplex rx_fine_sync_corrected(current_frame.begin() + (sync_len - fine_sync_len),
            current_frame.begin() + sync_len);

        double residual_phase = estimateResidualPhase(rx_fine_sync_corrected, sync_fine);
        if (!std::isfinite(residual_phase)) residual_phase = 0.0;

        const bool need_phase_rotation =
            (config_.modulation == ModulationType::BPSK ||
                config_.modulation == ModulationType::QPSK ||
                config_.modulation == ModulationType::QAM);

        if (need_phase_rotation && residual_phase != 0.0) {
            Complex ph_rot = std::conj(Complex(std::cos(residual_phase), std::sin(residual_phase)));
            for (auto& v : current_frame) v *= ph_rot;
        }

        std::cout << "[DBG][CFO] frame=" << frame_count_
            << " coarse_cfo_hz=" << coarse_cfo_hz
            << " residual_cfo_hz=" << frame_freq_offset
            << " residual_cfo_fit_hz=" << f_hat
            << " total_cfo_est_hz=" << total_cfo_est_hz
            << " total_cfo_fit_hz=" << total_cfo_fit_hz
            << " applied_residual_cfo_hz=" << f_use
            << " cfo_deadband_hz=" << cfo_deadband_hz
            << " residual_phase=" << residual_phase
            << "\n";

        // ====== 提取 payload（去掉 sync 和尾部 ZP） ======
        const size_t payload_start = (size_t)sync_len;
        const size_t payload_end = payload_start + (size_t)payload_samp_num;
        if (payload_end > current_frame.size()) {
            std::cout << "[Receiver] Payload out of bounds, demodulation ended\n";
            break;
        }

        VecComplex payload_with_gap(current_frame.begin() + payload_start,
            current_frame.begin() + payload_end);

        VecComplex payload_sig = payload_with_gap;

        if (config_.function == FunctionType::RemoteControl) {
            const int pulse_len = (int)config_.ccskcode.size() * s;
            const int gap_len = 33 * s;
            const int total_pulses = ccsk_chip_num / 32;

            // 1) 先解跳频
            payload_sig = dehopRemoteControlPayload(payload_with_gap, total_pulses, pulse_len, gap_len);

            // 2) 再去掉每个 pulse 后面的 gap，只保留有效 payload
            VecComplex depulsed;
            depulsed.reserve((size_t)total_pulses * (size_t)pulse_len);

            size_t pos = 0;
            for (int p = 0; p < total_pulses; ++p) {
                if (pos + (size_t)pulse_len > payload_sig.size()) break;

                depulsed.insert(depulsed.end(),
                    payload_sig.begin() + pos,
                    payload_sig.begin() + pos + (size_t)pulse_len);

                pos += (size_t)pulse_len;

                if (pos + (size_t)gap_len <= payload_sig.size()) {
                    pos += (size_t)gap_len;
                }
            }

            payload_sig.swap(depulsed);

            std::cout << "[DBG] depulsed payload samples=" << payload_sig.size() << "\n";
        }
        else {
            // 对带 RRC 的线性调制：低通 + SRRC 匹配滤波
            if (config_.modulation == ModulationType::BPSK ||
                config_.modulation == ModulationType::QPSK ||
                config_.modulation == ModulationType::QAM ||
                config_.modulation == ModulationType::OOK) {

                // 对齐 MATLAB fir1(128, 0.2)
                std::vector<double> fir_lp = designLowpassFIR(129, 0.2);
                payload_sig = applyFIRAndAlign(payload_sig, fir_lp);

                // RRC 匹配滤波
                payload_sig = applySRRCMatchedFilterAndAlign(payload_sig, s, 0.5, 6);
            }
        }

        if (std::abs(sfo_hat_ppm) > 0.01 &&
            (config_.modulation == ModulationType::BPSK ||
                config_.modulation == ModulationType::QPSK ||
                config_.modulation == ModulationType::QAM ||
                config_.modulation == ModulationType::OOK)) {
            payload_sig = Channel::applySFO(payload_sig, -sfo_hat_ppm);
        }

        if (config_.modulation == ModulationType::BPSK ||
            config_.modulation == ModulationType::QPSK ||
            config_.modulation == ModulationType::QAM ||
            config_.modulation == ModulationType::OOK) {
            g_linear_decision_phase = chooseBestLinearDecisionPhase(payload_sig, s);
        }
        else {
            g_linear_decision_phase = 0;
        }

        std::cout << "[DBG][TIMING] frame=" << frame_count_
            << " sync_refine_win=[" << win_start << "," << win_end << ")"
            << " best_sync_start=" << sync_abs_start
            << " decision_phase=" << g_linear_decision_phase
            << " sfo_hat_ppm=" << sfo_hat_ppm
            << "\n";

        // ====== 保存星座图点（取最后一次成功锁帧的 payload） ======
        buildConstellationPoints(payload_sig);

        // ====== 解调 ======
        VecInt rx_chips_or_syms = demodulateTelemetry(payload_sig);
        if (rx_chips_or_syms.empty()) {
            std::cout << "[Receiver] Demodulation output is empty, demodulation ended\n";
            break;
        }

        if (modulationNeedsDiffDecode()) {
            rx_chips_or_syms = d_decode(rx_chips_or_syms);
        }

        if ((int)rx_chips_or_syms.size() > ccsk_chip_num) {
            rx_chips_or_syms.resize((size_t)ccsk_chip_num);
        }

        if ((int)rx_chips_or_syms.size() < ccsk_chip_num) {
            std::cout << "[Receiver] Insufficient chips after demodulation, got "
                << rx_chips_or_syms.size()
                << ", expected " << ccsk_chip_num << "\n";
            break;
        }

        VecInt rx_encoded_bits = despreadCCSK(rx_chips_or_syms);
        if (rx_encoded_bits.empty()) {
            std::cout << "[Receiver] Despreading failed, demodulation ended\n";
            break;
        }

        if ((int)rx_encoded_bits.size() > encoded_bits) {
            rx_encoded_bits.resize((size_t)encoded_bits);
        }

        if ((int)rx_encoded_bits.size() < encoded_bits) {
            std::cout << "[Receiver] Insufficient coded bits after despreading, got "
                << rx_encoded_bits.size()
                << ", expected " << encoded_bits << "\n";
            break;
        }

        std::cout << "[DBG][RX] ccsk_decoded bits = " << rx_encoded_bits.size() << "\n";

        VecInt rx_source_bits = HXL_RSDecode(rx_encoded_bits, 31, 15);
        if (rx_source_bits.empty()) {
            std::cout << "[Receiver] RS decoding failed, demodulation ended\n";
            break;
        }

        total_rx_bits.insert(total_rx_bits.end(), rx_source_bits.begin(), rx_source_bits.end());

        std::cout << "[Receiver] Frame " << frame_count_
            << " demodulated, sync peak: " << sync_metric
            << ", coarse CFO: " << coarse_cfo_hz << "Hz"
            << ", residual CFO: " << frame_freq_offset << "Hz"
            << ", total CFO est: " << total_cfo_est_hz << "Hz"
            << ", demodulated bits: " << rx_source_bits.size()
            << "\n";

        const double sfo_scale = 1.0 + sfo_hat_ppm * 1e-6;
        long long next_step = (long long)std::llround((double)single_frame_total_samp * sfo_scale);
        if (next_step <= 0) next_step = single_frame_total_samp;
        current_offset = sync_abs_start + (size_t)next_step;
    }

    return total_rx_bits;
}

// ====================== BPSK ======================
VecInt Receiver::demodulateBPSK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);
    VecComplex syms = sampleLinearSymbolsAtPhase(rx, s, g_linear_decision_phase);
    for (const auto& sym : syms) {
        bits.push_back(sym.real() > 0.0 ? 1 : 0);
    }

    return bits;
}

// ====================== QPSK ======================
VecInt Receiver::demodulateQPSK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);
    VecComplex syms = sampleLinearSymbolsAtPhase(rx, s, g_linear_decision_phase);
    for (const auto& sym : syms) {

        int I_bit = (sym.real() > 0.0) ? 1 : 0;
        int Q_bit = (sym.imag() > 0.0) ? 1 : 0;

        bits.push_back(I_bit);
        bits.push_back(Q_bit);
    }

    return bits;
}

// ====================== 16QAM ======================
static inline int nearest_qam_level_natural(double x)
{
    const int levels[4] = { -3, -1, 1, 3 };
    int best = levels[0];
    double best_d = std::abs(x - (double)levels[0]);

    for (int i = 1; i < 4; ++i) {
        double d = std::abs(x - (double)levels[i]);
        if (d < best_d) {
            best_d = d;
            best = levels[i];
        }
    }
    return best;
}

static inline void qam_level_to_bits_natural(int level, int& b0, int& b1)
{
    switch (level) {
    case -3: b0 = 0; b1 = 0; break;
    case -1: b0 = 0; b1 = 1; break;
    case  1: b0 = 1; b1 = 0; break;
    case  3: b0 = 1; b1 = 1; break;
    default: b0 = 0; b1 = 0; break;
    }
}

VecInt Receiver::demodulateQAM16(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);
    VecComplex syms = sampleLinearSymbolsAtPhase(rx, s, g_linear_decision_phase);
    if (syms.empty()) return bits;

    normalizeQAMSymbolsAdaptive(syms);

    for (const auto& sym : syms) {
        double I = sym.real() * std::sqrt(10.0);
        double Q = sym.imag() * std::sqrt(10.0);

        int I_level = nearest_qam_level_natural(I);
        int Q_level = nearest_qam_level_natural(Q);

        int b0, b1, b2, b3;
        qam_level_to_bits_natural(I_level, b0, b1);
        qam_level_to_bits_natural(Q_level, b2, b3);

        bits.push_back(b0);
        bits.push_back(b1);
        bits.push_back(b2);
        bits.push_back(b3);
    }

    return bits;
}

// ====================== OOK ======================
VecInt Receiver::demodulateOOK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);

    std::vector<double> samples;
    VecComplex syms = sampleLinearSymbolsAtPhase(rx, s, g_linear_decision_phase);
    for (const auto& sym : syms) {
        samples.push_back(sym.real());
    }

    if (samples.empty()) return bits;

    double minv = *std::min_element(samples.begin(), samples.end());
    double maxv = *std::max_element(samples.begin(), samples.end());
    double th = 0.5 * (minv + maxv);

    if (!std::isfinite(th)) th = 0.5;

    for (double v : samples) {
        bits.push_back(v > th ? 1 : 0);
    }

    return bits;
}

// ====================== 2FSK ======================
VecInt Receiver::demodulateFSK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);

    if (rx.size() < 2 || s <= 0) return bits;

    const size_t nsym = rx.size() / (size_t)s;
    bits.reserve(nsym);

    for (size_t k = 0; k < nsym; ++k) {
        const size_t base = k * (size_t)s;

        double acc_phase = 0.0;

        for (int j = 0; j < s; ++j) {
            const size_t n = base + (size_t)j;
            if (n == 0 || n >= rx.size()) continue;

            Complex z = std::conj(rx[n - 1]) * rx[n];
            acc_phase += std::atan2(z.imag(), z.real());
        }

        bits.push_back(acc_phase >= 0.0 ? 1 : 0);
    }

    return bits;
}

// ====================== FM ======================
VecInt Receiver::demodulateFM(const VecComplex& rx)
{
    if (rx.size() < 2) return {};

    const int samp = std::max(1, config_.samp);
    const double fs = config_.fs;
    const double kf = 75e5;
    const double ts = 1.0 / fs;

    VecDouble dem;
    dem.reserve(rx.size());
    dem.push_back(0.0);

    const double denom = 2.0 * PI * kf * ts;
    for (size_t n = 1; n < rx.size(); ++n) {
        Complex prod_diff = rx[n] * std::conj(rx[n - 1]);
        double phase_diff = std::atan2(prod_diff.imag(), prod_diff.real());
        dem.push_back(phase_diff / denom);
    }

    const double rolloff_factor = 0.5;
    const int span = 6;
    VecDouble rcos_fir = designSRRC(rolloff_factor, span, samp);
    const int filter_delay = (int)(rcos_fir.size() - 1) / 2;

    VecDouble dem_padded = dem;
    dem_padded.insert(dem_padded.end(), (size_t)filter_delay, 0.0);

    VecDouble dem_filtered(dem_padded.size(), 0.0);
    for (size_t n = 0; n < dem_padded.size(); ++n) {
        double acc = 0.0;
        for (size_t k = 0; k < rcos_fir.size(); ++k) {
            if (n >= k) {
                acc += rcos_fir[k] * dem_padded[n - k];
            }
        }
        dem_filtered[n] = acc;
    }

    if (dem_filtered.size() <= (size_t)filter_delay) return {};
    VecDouble dem_filtered_aligned(dem_filtered.begin() + filter_delay, dem_filtered.end());

    VecInt received_bits;
    received_bits.reserve((dem_filtered_aligned.size() + samp - 1) / samp);

    for (size_t i = 0; i < dem_filtered_aligned.size(); i += (size_t)samp) {
        received_bits.push_back(dem_filtered_aligned[i] > 0.0 ? 1 : 0);
    }

    VecInt decode_output(received_bits.size(), 0);
    int b_prev = 1;
    for (size_t i = 0; i < received_bits.size(); ++i) {
        decode_output[i] = received_bits[i] ^ b_prev;
        b_prev = received_bits[i];
    }

    return decode_output;
}

// ====================== MSK ======================
VecInt Receiver::demodulateMSK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);

    if (rx.size() < 2 || s <= 0) return bits;

    const size_t nsym = rx.size() / (size_t)s;
    bits.reserve(nsym);

    for (size_t k = 0; k < nsym; ++k) {
        const size_t base = k * (size_t)s;

        double acc_phase = 0.0;

        for (int j = 0; j < s; ++j) {
            const size_t n = base + (size_t)j;
            if (n == 0 || n >= rx.size()) continue;

            Complex z = std::conj(rx[n - 1]) * rx[n];
            acc_phase += std::atan2(z.imag(), z.real());
        }

        bits.push_back(acc_phase >= 0.0 ? 1 : 0);
    }

    return bits;
}

VecComplex Receiver::dehopRemoteControlPayload(
    const VecComplex& payload_with_gap,
    int total_pulses,
    int pulse_len,
    int gap_len) const
{
    VecComplex out = payload_with_gap;
    if (out.empty() || total_pulses <= 0 || pulse_len <= 0) return out;

    VecDouble frq_seq = generate_sequence(-13, 13, 3, total_pulses, config_.hop_pattern);

    const double fs = config_.fs;
    if (!std::isfinite(fs) || fs <= 0.0) return out;

    size_t pos = 0;
    for (int p = 0; p < total_pulses; ++p)
    {
        if (pos + static_cast<size_t>(pulse_len) > out.size()) break;

        const double f = frq_seq[static_cast<size_t>(p)];

        for (int j = 0; j < pulse_len; ++j)
        {
            const double t = static_cast<double>(j + 1) / fs;
            const double ang = -2.0 * PI * f * t;
            out[pos + static_cast<size_t>(j)] *= Complex(std::cos(ang), std::sin(ang));
        }

        pos += static_cast<size_t>(pulse_len);

        if (pos + static_cast<size_t>(gap_len) <= out.size()) {
            pos += static_cast<size_t>(gap_len);
        }
    }

    return out;
}

// ====================== CCSK 解扩 ======================
VecInt Receiver::despreadCCSK(const VecInt& chips)
{
    VecInt decoded;
    const int block_len = 32;
    const VecInt& ccsk_code = config_.ccskcode;

    for (size_t i = 0; i + (size_t)block_len <= chips.size(); i += (size_t)block_len) {
        int best_k = 0;
        int max_corr = -1;

        for (int k = 0; k < 32; ++k) {
            int corr = 0;
            for (int j = 0; j < 32; ++j) {
                int tx_chip = ccsk_code[(j - k + 32) % 32];
                if (chips[i + (size_t)j] == tx_chip) corr++;
            }

            if (corr > max_corr) {
                max_corr = corr;
                best_k = k;
            }
        }

        for (int b = 4; b >= 0; --b) {
            decoded.push_back((best_k >> b) & 1);
        }
    }

    return decoded;
}

void Receiver::buildConstellationPoints(const VecComplex& payload_sig)
{
    last_constellation_points_.clear();

    if (payload_sig.empty()) return;

    const int s = std::max(1, config_.samp);
    const size_t max_points = 600;

    // ===== 线性调制：直接取每符号抽样点 =====
    if (config_.modulation == ModulationType::BPSK ||
        config_.modulation == ModulationType::QPSK ||
        config_.modulation == ModulationType::QAM ||
        config_.modulation == ModulationType::OOK)
    {
        VecComplex syms = sampleLinearSymbolsAtPhase(payload_sig, s, g_linear_decision_phase);
        for (const auto& sym : syms) {
            last_constellation_points_.push_back(sym);
            if (last_constellation_points_.size() >= max_points) break;
        }

        // QAM 单独做一下自适应归一化，显示更稳
        if (config_.modulation == ModulationType::QAM) {
            normalizeQAMSymbolsAdaptive(last_constellation_points_);
        }

        return;
    }

    // ===== FSK / FM / MSK：画差分相位“星座” =====
    // 每个符号统计一次 conj(r[n-1])*r[n] 的平均值
    if (config_.modulation == ModulationType::FSK ||
        config_.modulation == ModulationType::FM ||
        config_.modulation == ModulationType::MSK)
    {
        const size_t nsym = payload_sig.size() / (size_t)s;

        for (size_t k = 0; k < nsym; ++k) {
            const size_t base = k * (size_t)s;

            Complex acc(0.0, 0.0);
            int cnt = 0;

            for (int j = 1; j < s; ++j) {
                const size_t n = base + (size_t)j;
                if (n >= payload_sig.size()) break;

                acc += std::conj(payload_sig[n - 1]) * payload_sig[n];
                cnt++;
            }

            if (cnt > 0) {
                last_constellation_points_.push_back(acc / (double)cnt);
            }

            if (last_constellation_points_.size() >= max_points) break;
        }

        return;
    }
}
