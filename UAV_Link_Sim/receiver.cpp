#include "receiver.h"
#include "coding.h"
#include "spread_spectrum.h"
#include "sync.h"
#include "utils.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <algorithm>
#include <numeric>

static inline Complex cexpj(double a) {
    return Complex(std::cos(a), std::sin(a));
}

// =============== 一些本地辅助函数 ===============

static std::vector<double> designLowpassFIR(int num_taps, double cutoff_norm)
{
    // cutoff_norm: 相对 Nyquist 归一化频率，范围 (0, 1)
    // MATLAB fir1(N, Wn) 中 Wn 也是相对 Nyquist 归一化
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

    // 类似 MATLAB：估计低能量区域的噪声水平
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

    // 防止阈值太低
    if (!std::isfinite(Thsd) || Thsd <= 0.0) {
        double mean_amp = std::accumulate(amp.begin(), amp.end(), 0.0) / (double)amp.size();
        Thsd = 0.3 * mean_amp;
    }

    return Thsd;
}

static void normalizeQAMSymbolsAdaptive(VecComplex& syms)
{
    if (syms.empty()) return;

    // 先按平均功率归一到 16QAM 理想平均功率 1
    double avg_power = 0.0;
    for (const auto& v : syms) avg_power += std::norm(v);
    avg_power /= (double)syms.size();

    if (avg_power > 1e-12 && std::isfinite(avg_power)) {
        const double scale = 1.0 / std::sqrt(avg_power);
        for (auto& v : syms) v *= scale;
    }

    // 再做一个简单轴向拉伸自适应：让 I/Q 的平均绝对值更接近理想 16QAM
    // 对归一化 16QAM，E|I| = E|Q| = 2/sqrt(10)
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

// ====================== 每组求均值（对齐 MATLAB mean_groups） ======================
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

// ====================== 取每个 symbol 中点采样（对齐 MATLAB round(samp/2)） ======================
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
    return (config_.modulation == ModulationType::BPSK ||
        config_.modulation == ModulationType::FM);
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
        std::cerr << "[Receiver] 当前接收机未实现该调制方式\n";
        return {};
    }
}

// ====================== 主接收函数（锁帧+窄窗同步） ======================
VecInt Receiver::receive(const VecComplex& rx_signal)
{
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
        std::cerr << "[Receiver] 信号长度不足一帧，无法解调\n";
        return {};
    }

    VecInt total_rx_bits;
    size_t current_offset = 0;
    const size_t total_rx_len = rx_signal.size();

    // 后续帧跟踪阈值：不要太低，避免局部误锁
    const double sync_threshold = 0.08;
    const size_t W = (size_t)(2 * s);

    // 首帧专用：全局找“最早有效峰”
    const double first_sync_threshold = 0.4;   // 可按实际改成 0.35~0.50
    const size_t first_refine_W = (size_t)(2 * s);

    bool acquired = false;

    while (current_offset + (size_t)single_frame_total_samp <= total_rx_len) {

        size_t sync_abs_start = current_offset;
        double sync_metric = -1.0;
        bool ok = false;

        if (!acquired) {
            // ===== 首帧：全局从前往后找第一个超过高阈值的位置 =====
            bool found_candidate = false;
            size_t cand_idx = 0;

            const size_t search_end = total_rx_len - (size_t)sync_len;
            for (size_t i = 0; i <= search_end; ++i) {
                double m = metricAtNormalized(rx_signal, i, SYNC);
                if (m >= first_sync_threshold) {
                    cand_idx = i;
                    found_candidate = true;
                    break;
                }
            }

            if (!found_candidate) {
                std::cout << "[Receiver] 首帧未找到超过阈值的同步候选，解调结束\n";
                break;
            }

            // ===== 在候选点附近小窗精搜索，避免锁在峰的边沿 =====
            size_t win_start = (cand_idx > first_refine_W) ? (cand_idx - first_refine_W) : 0;
            size_t win_end = std::min(total_rx_len, cand_idx + first_refine_W + (size_t)sync_len);

            int best_rel = 0;
            double best_m = 0.0;
            if (!findSyncStartNormalizedWindow(rx_signal, win_start, win_end, SYNC, best_rel, best_m)) {
                std::cout << "[Receiver] 首帧候选附近精搜索失败，解调结束\n";
                break;
            }

            sync_abs_start = win_start + (size_t)best_rel;
            sync_metric = best_m;
            ok = (sync_metric >= first_sync_threshold);

            if (!ok) {
                std::cout << "[Receiver] 首帧同步峰值过低(" << sync_metric << ")，解调结束\n";
                break;
            }

            std::cout << "[DBG][FIRST] cand_idx=" << cand_idx
                << " refined_sync_abs_start=" << sync_abs_start
                << " metric=" << sync_metric << "\n";

            acquired = true;
        }
        else {
            // ===== 后续帧：按预测位置局部跟踪 =====
            sync_abs_start = current_offset;
            sync_metric = metricAtNormalized(rx_signal, sync_abs_start, SYNC);
            ok = (sync_metric >= sync_threshold);

            if (!ok) {
                size_t win_start = (current_offset > W) ? (current_offset - W) : 0;
                size_t win_end = std::min(total_rx_len, current_offset + W + (size_t)sync_len);

                int best_rel = 0;
                double best_m = 0.0;
                if (!findSyncStartNormalizedWindow(rx_signal, win_start, win_end, SYNC, best_rel, best_m)) {
                    std::cout << "[Receiver] 未检测到有效同步头，解调结束\n";
                    break;
                }

                sync_abs_start = win_start + (size_t)best_rel;
                sync_metric = best_m;
                ok = (sync_metric >= sync_threshold);
            }

            if (!ok) {
                std::cout << "[Receiver] 同步峰值过低(" << sync_metric << ")，解调结束\n";
                break;
            }
        }

        const size_t frame_abs_end = sync_abs_start + (size_t)single_frame_total_samp;
        if (frame_abs_end > total_rx_len) {
            std::cout << "[Receiver] 剩余采样不足完整一帧，解调结束\n";
            break;
        }

        std::cout << "[DBG] current_offset=" << current_offset
            << " sync_abs_start=" << sync_abs_start
            << " drift=" << (long long)sync_abs_start - (long long)current_offset
            << " metric=" << sync_metric
            << "\n";

        VecComplex current_frame(rx_signal.begin() + sync_abs_start,
            rx_signal.begin() + frame_abs_end);

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

        // 只对真正依赖载波相位/频率精确对齐的调制做 CFO 校正
        const bool need_cfo_correction =
            (config_.modulation == ModulationType::BPSK ||
                config_.modulation == ModulationType::QPSK ||
                config_.modulation == ModulationType::QAM ||
                config_.modulation == ModulationType::FM);

        if (config_.enable_cfo && need_cfo_correction && f_hat != 0.0) {
            for (size_t n = 0; n < current_frame.size(); ++n) {
                double t = (double)n / fs;
                double ang = 2.0 * PI * f_hat * t;
                if (!std::isfinite(ang)) continue;
                current_frame[n] *= std::conj(Complex(std::cos(ang), std::sin(ang)));
            }
        }

        VecComplex rx_fine_sync_corrected(current_frame.begin() + (sync_len - fine_sync_len),
            current_frame.begin() + sync_len);

        double residual_phase = estimateResidualPhase(rx_fine_sync_corrected, sync_fine);
        if (!std::isfinite(residual_phase)) residual_phase = 0.0;

        // 只对相位型调制做常相位补偿
        const bool need_phase_rotation =
            (config_.modulation == ModulationType::BPSK ||
                config_.modulation == ModulationType::QPSK ||
                config_.modulation == ModulationType::QAM ||
                config_.modulation == ModulationType::FM);

        if (need_phase_rotation && residual_phase != 0.0) {
            Complex ph_rot = std::conj(Complex(std::cos(residual_phase), std::sin(residual_phase)));
            for (auto& v : current_frame) v *= ph_rot;
        }

        // ====== 提取 payload（去掉 sync 和尾部 ZP） ======
        const size_t payload_start = (size_t)sync_len;
        const size_t payload_end = payload_start + (size_t)payload_samp_num;
        if (payload_end > current_frame.size()) {
            std::cout << "[Receiver] payload 越界，解调结束\n";
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
            // 遥测模式：QPSK / QAM / OOK 先做低通；BPSK / FSK / FM / MSK 保持原链路
            if (config_.modulation == ModulationType::QPSK ||
                config_.modulation == ModulationType::QAM ||
                config_.modulation == ModulationType::OOK) {
                std::vector<double> fir_lp = designLowpassFIR(129, 4.0 / (double)s);
                payload_sig = applyFIRAndAlign(payload_sig, fir_lp);
            }
        }

        // ====== 解调 ======
        VecInt rx_chips_or_syms = demodulateTelemetry(payload_sig);
        if (rx_chips_or_syms.empty()) {
            std::cout << "[Receiver] 解调输出为空，解调结束\n";
            break;
        }

        if (modulationNeedsDiffDecode()) {
            rx_chips_or_syms = d_decode(rx_chips_or_syms);
        }

        if ((int)rx_chips_or_syms.size() > ccsk_chip_num) {
            rx_chips_or_syms.resize((size_t)ccsk_chip_num);
        }

        if ((int)rx_chips_or_syms.size() < ccsk_chip_num) {
            std::cout << "[Receiver] 解调后 chip 数不足，得到 "
                << rx_chips_or_syms.size()
                << "，期望 " << ccsk_chip_num << "\n";
            break;
        }

        VecInt rx_encoded_bits = despreadCCSK(rx_chips_or_syms);
        if (rx_encoded_bits.empty()) {
            std::cout << "[Receiver] 解扩失败，解调结束\n";
            break;
        }

        if ((int)rx_encoded_bits.size() > encoded_bits) {
            rx_encoded_bits.resize((size_t)encoded_bits);
        }

        if ((int)rx_encoded_bits.size() < encoded_bits) {
            std::cout << "[Receiver] 解扩后编码比特数不足，得到 "
                << rx_encoded_bits.size()
                << "，期望 " << encoded_bits << "\n";
            break;
        }

        std::cout << "[DBG][RX] ccsk_decoded bits = " << rx_encoded_bits.size() << "\n";

        VecInt rx_source_bits = HXL_RSDecode(rx_encoded_bits, 31, 15);
        if (rx_source_bits.empty()) {
            std::cout << "[Receiver] RS 解码失败，解调结束\n";
            break;
        }

        total_rx_bits.insert(total_rx_bits.end(), rx_source_bits.begin(), rx_source_bits.end());

        std::cout << "[Receiver] 第" << frame_count_
            << "帧解调完成，同步峰值：" << sync_metric
            << "，频偏估计：" << f_hat << "Hz"
            << "，解调比特数：" << rx_source_bits.size()
            << "\n";

        current_offset = frame_abs_end;
    }

    return total_rx_bits;
}

// ====================== BPSK ======================
VecInt Receiver::demodulateBPSK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);

    for (size_t base = 0; base + (size_t)s <= rx.size(); base += (size_t)s) {
        int best_j = 0;
        double best_e = -1.0;

        for (int j = 0; j < s; ++j) {
            double e = std::norm(rx[base + (size_t)j]);
            if (e > best_e) {
                best_e = e;
                best_j = j;
            }
        }

        Complex sym = rx[base + (size_t)best_j];
        bits.push_back(sym.real() > 0.0 ? 1 : 0);
    }

    return bits;
}

// ====================== QPSK ======================
// 保持你原来的 bit 映射：
// 00 -> (+,+)
// 01 -> (+,-)
// 10 -> (-,+)
// 11 -> (-,-)
VecInt Receiver::demodulateQPSK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);

    VecComplex syms = meanGroups(rx, s);
    if (syms.empty()) return {};

    std::cout << "[DBG][QPSK] mean_groups symbols = " << syms.size() << "\n";

    for (const auto& sym : syms) {
        int b0 = (sym.real() < 0.0) ? 1 : 0;
        int b1 = (sym.imag() < 0.0) ? 1 : 0;

        bits.push_back(b0);
        bits.push_back(b1);
    }

    return bits;
}

// ====================== 16QAM ======================
// 发射端映射：
// 00 -> -3
// 01 -> +3
// 10 -> +1
// 11 -> -1
static inline int nearest_qam_level(double x)
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

static inline void qam_level_to_bits(int level, int& b0, int& b1)
{
    switch (level) {
    case -3: b0 = 0; b1 = 0; break;
    case  3: b0 = 0; b1 = 1; break;
    case  1: b0 = 1; b1 = 0; break;
    case -1: b0 = 1; b1 = 1; break;
    default: b0 = 0; b1 = 0; break;
    }
}

VecInt Receiver::demodulateQAM16(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);

    VecComplex syms = meanGroups(rx, s);
    if (syms.empty()) return {};

    normalizeQAMSymbolsAdaptive(syms);

    std::cout << "[DBG][QAM16] mean_groups symbols = " << syms.size() << "\n";

    for (const auto& sym : syms) {
        double I = sym.real() * std::sqrt(10.0);
        double Q = sym.imag() * std::sqrt(10.0);

        int I_level = nearest_qam_level(I);
        int Q_level = nearest_qam_level(Q);

        int b0, b1, b2, b3;
        qam_level_to_bits(I_level, b0, b1);
        qam_level_to_bits(Q_level, b2, b3);

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
    if (rx.empty()) return bits;

    // 每个 symbol 用“最大若干个采样幅度的平均”作为统计量
    // 比单纯 max 更抗噪声尖峰，同时仍保留 OOK 的幅度差异
    std::vector<double> sym_amp;
    sym_amp.reserve(rx.size() / (size_t)s);

    const int top_k = std::max(1, s / 4); // 可调：s/4 通常比较稳

    for (size_t base = 0; base + (size_t)s <= rx.size(); base += (size_t)s) {
        std::vector<double> amps;
        amps.reserve((size_t)s);

        for (int j = 0; j < s; ++j) {
            amps.push_back(std::abs(rx[base + (size_t)j]));
        }

        // 取最大的 top_k 个样本做平均
        std::nth_element(
            amps.begin(),
            amps.end() - top_k,
            amps.end()
        );

        double acc = 0.0;
        for (auto it = amps.end() - top_k; it != amps.end(); ++it) {
            acc += *it;
        }

        sym_amp.push_back(acc / (double)top_k);
    }

    if (sym_amp.empty()) return bits;

    // 用百分位数而不是 min/max 来估门限，避免少量异常值把门限拉坏
    std::vector<double> sorted_amp = sym_amp;
    std::sort(sorted_amp.begin(), sorted_amp.end());

    auto percentile = [&](double p) -> double {
        p = std::max(0.0, std::min(1.0, p));
        size_t idx = (size_t)std::floor(p * (sorted_amp.size() - 1));
        return sorted_amp[idx];
        };

    const double p10 = percentile(0.10);
    const double p90 = percentile(0.90);
    double th = 0.5 * (p10 + p90);

    // 退化保护：如果这一帧高低两簇分离很差，就退回均值门限
    if (!std::isfinite(th) || (p90 - p10) < 1e-8) {
        th = std::accumulate(sym_amp.begin(), sym_amp.end(), 0.0) / (double)sym_amp.size();
    }

    std::cout << "[DBG][OOK] p10=" << p10
        << " p90=" << p90
        << " th=" << th
        << " top_k=" << top_k
        << "\n";

    bits.reserve(sym_amp.size());
    for (double a : sym_amp) {
        bits.push_back(a > th ? 1 : 0);
    }

    return bits;
}

// ====================== 2FSK ======================
VecInt Receiver::demodulateFSK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);

    const double deta_f = 1e6;
    const double fs = config_.fs;

    if (!std::isfinite(fs) || fs <= 0.0) return {};

    for (size_t base = 0; base + (size_t)s <= rx.size(); base += (size_t)s) {
        Complex corr_pos(0.0, 0.0);
        Complex corr_neg(0.0, 0.0);

        for (int j = 0; j < s; ++j) {
            double t = (double)j / fs;
            Complex c_pos(std::cos(2 * PI * deta_f * t), std::sin(2 * PI * deta_f * t));
            Complex c_neg(std::cos(-2 * PI * deta_f * t), std::sin(-2 * PI * deta_f * t));

            corr_pos += rx[base + (size_t)j] * std::conj(c_pos);
            corr_neg += rx[base + (size_t)j] * std::conj(c_neg);
        }

        bits.push_back(std::norm(corr_pos) >= std::norm(corr_neg) ? 1 : 0);
    }

    return bits;
}

// ====================== FM ======================
VecInt Receiver::demodulateFM(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);

    if (rx.size() < 2) return {};

    VecDouble disc(rx.size(), 0.0);
    disc[0] = 0.0;

    for (size_t n = 1; n < rx.size(); ++n) {
        Complex z = std::conj(rx[n - 1]) * rx[n];
        disc[n] = std::atan2(z.imag(), z.real());
    }

    for (size_t base = 0; base + (size_t)s <= rx.size(); base += (size_t)s) {
        double acc = 0.0;
        for (int j = 0; j < s; ++j) {
            acc += disc[base + (size_t)j];
        }
        bits.push_back(acc > 0.0 ? 1 : 0);
    }

    return bits;
}

// ====================== MSK ======================
VecInt Receiver::demodulateMSK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);
    if (rx.size() < (size_t)s) return {};

    const double phase_table[4] = { 0.0, PI / 2.0, PI, 3.0 * PI / 2.0 };

    auto build_template = [&](int state_idx, int bit_val) -> VecComplex {
        VecComplex tmpl;
        tmpl.reserve((size_t)s);

        double phase = phase_table[state_idx];
        double T = 1.0;
        double Ts = T / s;
        double b = bit_val ? 1.0 : -1.0;

        for (int j = 0; j < s; ++j) {
            double t = j * Ts;
            double I = b * std::cos(PI * t / (2.0 * T)) * std::cos(phase);
            double Q = b * std::sin(PI * t / (2.0 * T)) * std::sin(phase);
            tmpl.emplace_back(I, Q);
        }
        return tmpl;
        };

    VecComplex tmpl[4][2];
    for (int st = 0; st < 4; ++st) {
        tmpl[st][0] = build_template(st, 0);
        tmpl[st][1] = build_template(st, 1);
    }

    int state = 0;

    const size_t nsym = rx.size() / (size_t)s;
    bits.reserve(nsym);

    for (size_t k = 0; k < nsym; ++k) {
        const size_t base = k * (size_t)s;

        double best_err = 1e300;
        int best_bit = 0;

        for (int bit = 0; bit <= 1; ++bit) {
            double err = 0.0;

            for (int j = 0; j < s; ++j) {
                Complex d = rx[base + (size_t)j] - tmpl[state][bit][(size_t)j];
                err += std::norm(d);
            }

            if (err < best_err) {
                best_err = err;
                best_bit = bit;
            }
        }

        bits.push_back(best_bit);

        if (best_bit == 1)
            state = (state + 1) & 3;
        else
            state = (state + 3) & 3;
    }

    return bits;
}

VecComplex Receiver::dehopRemoteControlPayload(const VecComplex& payload_with_gap,
    int total_pulses,
    int pulse_len,
    int gap_len) const
{
    VecComplex out = payload_with_gap;
    if (out.empty() || total_pulses <= 0 || pulse_len <= 0) return out;

    VecDouble frq_seq = generate_sequence(-13e3, 13e3, 3e3, total_pulses, 1);

    const double fs = config_.fs;
    if (!std::isfinite(fs) || fs <= 0.0) return out;

    size_t pos = 0;
    for (int p = 0; p < total_pulses; ++p) {
        if (pos + (size_t)pulse_len > out.size()) break;

        const double f = frq_seq[(size_t)p];

        for (int j = 0; j < pulse_len; ++j) {
            const double t = (double)j / fs;
            const double ang = -2.0 * PI * f * t;
            out[pos + (size_t)j] *= Complex(std::cos(ang), std::sin(ang));
        }

        pos += (size_t)pulse_len;

        if (pos + (size_t)gap_len <= out.size()) {
            pos += (size_t)gap_len;
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