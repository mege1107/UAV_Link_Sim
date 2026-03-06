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

// ====================== 辅助：每符号挑最大能量采样点 ======================
Complex Receiver::pickBestSampleInSymbol(const VecComplex& rx, size_t base, int s) const
{
    int best_j = 0;
    double best_e = -1.0;

    for (int j = 0; j < s; ++j) {
        double e = std::norm(rx[base + (size_t)j]);
        if (e > best_e) {
            best_e = e;
            best_j = j;
        }
    }
    return rx[base + (size_t)best_j];
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

    // 2) 单帧长度（必须按 RS 编码后的 payload 算）
    const int source_bits = config_.frame_bit * config_.n;   // 原始信息位
    const int rs_msg_bits = 15 * 5;                          // 75
    const int rs_code_bits = 31 * 5;                         // 155

    if (source_bits % rs_msg_bits != 0) {
        std::cerr << "[Receiver] ERROR: source_bits is not a multiple of 75\n";
        return {};
    }

    const int rs_blocks = source_bits / rs_msg_bits;
    const int encoded_bits = rs_blocks * rs_code_bits;

    // CCSK(32,5): 每 5 bit -> 32 chips
    const int ccsk_chip_num = (encoded_bits / 5) * 32;

    const int zp_samp_num = config_.zp_sym * std::max(1, config_.samp);

    int payload_samp_num = 0;
    int total_pulses_dbg = 0;

    if (config_.function == FunctionType::RemoteControl) {
        const int pulse_len = (int)config_.ccskcode.size() * std::max(1, config_.samp); // 32*samp
        const int gap_len = 33 * std::max(1, config_.samp);

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
    const double sync_threshold = 0.08;
    const size_t W = (size_t)(2 * std::max(1, config_.samp));

    while (current_offset + (size_t)single_frame_total_samp <= total_rx_len) {

        size_t sync_abs_start = current_offset;
        double sync_metric = metricAtNormalized(rx_signal, sync_abs_start, SYNC);

        bool ok = (sync_metric >= sync_threshold);

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

        const size_t frame_abs_end = sync_abs_start + (size_t)single_frame_total_samp;
        if (frame_abs_end > total_rx_len) break;

        std::cout << "[DBG] current_offset=" << current_offset
            << " sync_abs_start=" << sync_abs_start
            << " drift=" << (long long)sync_abs_start - (long long)current_offset
            << " metric=" << sync_metric
            << "\n";

        VecComplex current_frame(rx_signal.begin() + sync_abs_start, rx_signal.begin() + frame_abs_end);

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

        if (config_.enable_cfo && f_hat != 0.0) {
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

        // FM 对整体常相位不敏感，但旋一下也无妨
        if (residual_phase != 0.0) {
            Complex rot = std::conj(cexpj(residual_phase));
            for (auto& samp : current_frame) samp *= rot;
        }

        // 4) payload 截取并裁 ZP
        VecComplex payload(current_frame.begin() + sync_len, current_frame.end());
        if (zp_samp_num > 0 && (int)payload.size() > zp_samp_num) {
            payload.resize(payload.size() - (size_t)zp_samp_num);
        }

        if (config_.function == FunctionType::RemoteControl) {
            const int pulse_len = (int)config_.ccskcode.size() * std::max(1, config_.samp); // 32*samp
            const int gap_len = 33 * std::max(1, config_.samp);
            const int total_pulses = ccsk_chip_num / 32;

            // 1) 先解跳频
            payload = dehopRemoteControlPayload(payload, total_pulses, pulse_len, gap_len);

            // 2) 再去掉 gap
            VecComplex depulsed;
            depulsed.reserve((size_t)total_pulses * (size_t)pulse_len);

            size_t pos = 0;
            for (int p = 0; p < total_pulses; ++p) {
                if (pos + (size_t)pulse_len > payload.size()) break;

                depulsed.insert(depulsed.end(),
                    payload.begin() + pos,
                    payload.begin() + pos + (size_t)pulse_len);

                pos += (size_t)pulse_len;

                if (pos + (size_t)gap_len <= payload.size()) {
                    pos += (size_t)gap_len;
                }
            }

            payload.swap(depulsed);

            std::cout << "[DBG] depulsed payload samples=" << payload.size() << "\n";
        }

        // 5) 解调链路
        VecInt demod_bits = demodulateTelemetry(payload);
        if (demod_bits.empty()) {
            std::cout << "[Receiver] 解调结果为空，结束\n";
            break;
        }

        VecInt post_demod_bits = demod_bits;
        if (modulationNeedsDiffDecode()) {
            post_demod_bits = d_decode(demod_bits);
        }

        VecInt ccsk_decoded = despreadCCSK(post_demod_bits);
        std::cout << "[DBG][RX] ccsk_decoded bits = " << ccsk_decoded.size() << "\n";
        VecInt rs_decoded = HXL_RSDecode(ccsk_decoded, 31, 15);

        total_rx_bits.insert(total_rx_bits.end(), rs_decoded.begin(), rs_decoded.end());

        std::cout << "[Receiver] 第" << frame_count_
            << "帧解调完成，同步峰值：" << sync_metric
            << "，频偏估计：" << f_hat << "Hz"
            << "，解调比特数：" << rs_decoded.size()
            << std::endl;

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
        Complex sym = pickBestSampleInSymbol(rx, base, s);
        bits.push_back(sym.real() > 0.0 ? 1 : 0);
    }

    return bits;
}

// ====================== QPSK ======================
// 对应发射端 qpskmod 的映射：
// 00 -> (+,+)
// 01 -> (+,-)
// 10 -> (-,+)
// 11 -> (-,-)
VecInt Receiver::demodulateQPSK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);

    for (size_t base = 0; base + (size_t)s <= rx.size(); base += (size_t)s) {
        Complex sym = pickBestSampleInSymbol(rx, base, s);

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
    const double norm = std::sqrt(10.0);

    for (size_t base = 0; base + (size_t)s <= rx.size(); base += (size_t)s) {
        Complex sym = pickBestSampleInSymbol(rx, base, s);

        double I = sym.real() * norm;
        double Q = sym.imag() * norm;

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

    for (size_t base = 0; base + (size_t)s <= rx.size(); base += (size_t)s) {
        double best_amp = 0.0;
        for (int j = 0; j < s; ++j) {
            best_amp = std::max(best_amp, std::abs(rx[base + (size_t)j]));
        }
        bits.push_back(best_amp > 0.5 ? 1 : 0);
    }

    return bits;
}

// ====================== 2FSK ======================
VecInt Receiver::demodulateFSK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);

    // 与 transmitter.cpp 中保持一致
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
// 发射端 fmmod 内部先 d_encode，再做双极性、升余弦、积分调相。
// 这里用相邻采样差分相位做频率判决，再按每 symbol 求和取符号。
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

VecInt Receiver::demodulateMSK(const VecComplex& rx)
{
    VecInt bits;
    const int s = std::max(1, config_.samp);
    if (rx.size() < (size_t)s) return {};

    // 4个相位状态：0, pi/2, pi, 3pi/2
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

    // 预生成模板
    VecComplex tmpl[4][2];
    for (int st = 0; st < 4; ++st) {
        tmpl[st][0] = build_template(st, 0);
        tmpl[st][1] = build_template(st, 1);
    }

    // 发射端初始 phase = 0
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

        // 状态更新要和发射端一致
        if (best_bit == 1)
            state = (state + 1) & 3;   // +pi/2
        else
            state = (state + 3) & 3;   // -pi/2
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

    // 发射端用的就是这个固定跳频序列生成方式
    VecDouble frq_seq = generate_sequence(-13, 13, 3, total_pulses, 1);

    const double fs = config_.fs;
    if (!std::isfinite(fs) || fs <= 0.0) return out;

    size_t pos = 0;
    for (int p = 0; p < total_pulses; ++p) {
        if (pos + (size_t)pulse_len > out.size()) break;

        const double f = frq_seq[(size_t)p];

        for (int j = 0; j < pulse_len; ++j) {
            const double t = (double)j / fs;
            const double ang = -2.0 * PI * f * t;   // 乘共轭，解跳频
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