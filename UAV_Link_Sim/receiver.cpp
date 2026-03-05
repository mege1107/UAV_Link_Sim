#include "receiver.h"
#include "coding.h"
#include "spread_spectrum.h"
#include "sync.h"
#include "utils.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <algorithm>

static inline Complex cexpj(double a) {
    return Complex(std::cos(a), std::sin(a));
}

// ====================== 归一化滑动相关同步检测 ======================
static bool findSyncStartNormalized(
    const VecComplex& rx,
    const VecComplex& sync,
    int& best_idx,
    double& best_metric,
    int max_search_len = -1
) {
    if (rx.size() < sync.size() || sync.empty()) return false;

    const int N = (int)rx.size();
    const int M = (int)sync.size();
    const int K = (max_search_len < 0) ? (N - M) : std::min(N - M, max_search_len);

    double Es = 0.0;
    for (const auto& s : sync) Es += std::norm(s);
    if (Es <= 1e-12) return false;

    best_metric = -1.0;
    best_idx = 0;

    for (int i = 0; i <= K; ++i) {
        Complex dot(0.0, 0.0);
        double Er = 0.0;

        for (int k = 0; k < M; ++k) {
            dot += std::conj(sync[(size_t)k]) * rx[(size_t)i + (size_t)k];
            Er += std::norm(rx[(size_t)i + (size_t)k]);
        }
        if (Er <= 1e-12) continue;

        double metric = std::norm(dot) / (Es * Er);
        if (metric > best_metric) {
            best_metric = metric;
            best_idx = i;
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

// ====================== 残余固定相位估计 ======================
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

// ====================== 主接收函数 ======================
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

    // 2) 单帧长度
    const int source_bits = config_.frame_bit * config_.n;
    const int ccsk_chip_num = (source_bits / 5) * 32;
    const int payload_samp_num = ccsk_chip_num * config_.samp;
    const int zp_samp_num = config_.zp_sym * config_.samp;
    const int single_frame_total_samp = sync_len + payload_samp_num + zp_samp_num;

    if (rx_signal.size() < (size_t)single_frame_total_samp) {
        std::cerr << "[Receiver] 信号长度不足一帧，无法解调\n";
        return {};
    }

    VecInt total_rx_bits;
    size_t current_offset = 0;
    const size_t total_rx_len = rx_signal.size();
    const double sync_threshold = 0.08;

    while (current_offset + (size_t)single_frame_total_samp <= total_rx_len) {
        VecComplex current_segment(rx_signal.begin() + current_offset, rx_signal.end());

        int sync_rel_idx = 0;
        double sync_metric = 0.0;

        int max_search = std::min((int)current_segment.size() - sync_len, 2 * single_frame_total_samp);
        if (!findSyncStartNormalized(current_segment, SYNC, sync_rel_idx, sync_metric, max_search)) {
            std::cout << "[Receiver] 未检测到有效同步头，解调结束\n";
            break;
        }
        if (sync_metric < sync_threshold) {
            std::cout << "[Receiver] 同步峰值过低(" << sync_metric << ")，解调结束\n";
            break;
        }

        const size_t sync_abs_start = current_offset + (size_t)sync_rel_idx;
        const size_t frame_abs_end = sync_abs_start + (size_t)single_frame_total_samp;
        if (frame_abs_end > total_rx_len) break;

        VecComplex current_frame(rx_signal.begin() + sync_abs_start, rx_signal.begin() + frame_abs_end);

        // ====== CFO + 相位校正（受开关控制）======
        double f_hat = 0.0;
        if (config_.enable_cfo) {
            VecComplex rx_fine_sync(current_frame.begin() + (sync_len - fine_sync_len),
                current_frame.begin() + sync_len);

            double frame_freq_offset = estimateCFOFromPreamble(rx_fine_sync, sync_fine, fs);
            frame_count_++;

            frame_index_history_.push_back(frame_count_);
            freq_offset_history_.push_back(frame_freq_offset);

            linearFitLSQ(frame_index_history_, freq_offset_history_, fit_a_, fit_b_);
            f_hat = fit_a_ * frame_count_ + fit_b_;
            if (!std::isfinite(f_hat)) f_hat = 0.0;

            if (f_hat != 0.0) {
                for (size_t n = 0; n < current_frame.size(); ++n) {
                    double t = (double)n / fs;
                    double ang = 2 * PI * f_hat * t;
                    if (!std::isfinite(ang)) continue;
                    current_frame[n] *= std::conj(cexpj(ang));
                }
            }

            VecComplex rx_fine_sync_corrected(current_frame.begin() + (sync_len - fine_sync_len),
                current_frame.begin() + sync_len);
            double residual_phase = estimateResidualPhase(rx_fine_sync_corrected, sync_fine);
            if (!std::isfinite(residual_phase)) residual_phase = 0.0;

            if (residual_phase != 0.0) {
                Complex rot = std::conj(cexpj(residual_phase));
                for (auto& samp : current_frame) samp *= rot;
            }
        }
        else {
            // CFO关闭时：仍然计帧数用于日志
            frame_count_++;
        }

        // 4) payload 截取并裁 ZP
        VecComplex payload(current_frame.begin() + sync_len, current_frame.end());
        if (zp_samp_num > 0 && (int)payload.size() > zp_samp_num) {
            payload.resize(payload.size() - (size_t)zp_samp_num);
        }

        // 5) 解调链路
        VecInt bpsk_bits = demodulateBPSK(payload);
        VecInt diff_decoded = d_decode(bpsk_bits);
        VecInt ccsk_decoded = despreadCCSK(diff_decoded);
        VecInt rs_decoded = HXL_RSDecode(ccsk_decoded, 31, 15);

        total_rx_bits.insert(total_rx_bits.end(), rs_decoded.begin(), rs_decoded.end());

        std::cout << "[Receiver] 第" << frame_count_
            << "帧解调完成，同步峰值：" << sync_metric
            << "，频偏估计：" << (config_.enable_cfo ? f_hat : 0.0) << "Hz"
            << "，解调比特数：" << rs_decoded.size()
            << (config_.enable_cfo ? "" : " (CFO disabled)")
            << std::endl;

        current_offset = frame_abs_end;
    }

    return total_rx_bits;
}

// ====================== BPSK：符号窗内最大能量点判决 ======================
VecInt Receiver::demodulateBPSK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);

    for (size_t base = 0; base + (size_t)s <= rx.size(); base += (size_t)s) {
        int best_j = 0;
        double best_e = -1.0;

        for (int j = 0; j < s; ++j) {
            double e = std::norm(rx[base + (size_t)j]);
            if (e > best_e) { best_e = e; best_j = j; }
        }

        double val = rx[base + (size_t)best_j].real();
        bits.push_back(val > 0 ? 1 : 0);
    }

    return bits;
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