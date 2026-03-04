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

// 归一化滑动相关：返回峰值位置（SYNC起点），以及峰值度量
static bool findSyncStartNormalized(
    const VecComplex& rx,
    const VecComplex& sync,
    int& best_idx,
    double& best_metric,
    int search_len // 只搜索前 search_len 个采样，避免O(N*M)太慢
) {
    if (rx.size() < sync.size() || sync.empty()) return false;

    const int N = (int)rx.size();
    const int M = (int)sync.size();
    const int K = std::min(N - M, std::max(0, search_len));

    // 预计算sync能量
    double Es = 0.0;
    for (const auto& s : sync) Es += std::norm(s);
    if (Es <= 0) return false;

    best_metric = -1.0;
    best_idx = 0;

    // 简化实现：直接滑动点乘 + 能量归一化
    for (int i = 0; i <= K; ++i) {
        Complex dot(0.0, 0.0);
        double Er = 0.0;

        for (int k = 0; k < M; ++k) {
            dot += std::conj(sync[k]) * rx[i + k];
            Er += std::norm(rx[i + k]);
        }
        if (Er <= 0) continue;

        // metric = |dot|^2 / (Es*Er)，范围[0,1]附近（理想匹配接近1）
        double metric = std::norm(dot) / (Es * Er);
        if (metric > best_metric) {
            best_metric = metric;
            best_idx = i;
        }
    }
    return true;
}

// 用“重复的精同步”做 CFO（每采样相位增量 w_hat）估计
// fine_sync = base_fine 重复 fine_length 次
static bool estimateCFOFromRepeatedFineSync(
    const VecComplex& rx_sync,   // 从rx里截出的整段SYNC（已对齐）
    const VecComplex& fine_sync, // 生成的整段fine_sync（含重复）
    int fine_length,
    double& w_hat                // 返回：每采样相位增量(rad/sample)
) {
    if (fine_length < 2) return false;
    if (rx_sync.size() != fine_sync.size()) return false;

    // 计算 base_fine 长度：fine_sync总长 / fine_length
    const int total = (int)fine_sync.size();
    if (total % fine_length != 0) return false;
    const int L = total / fine_length;
    if (L <= 0 || total < 2 * L) return false;

    // 用相邻重复段做相位差：sum conj(r0)*r1
    // 理想情况下：r1 ≈ r0 * exp(j*w*L)  => angle = w*L
    Complex acc(0.0, 0.0);
    for (int i = 0; i < L; ++i) {
        acc += std::conj(rx_sync[i]) * rx_sync[i + L];
    }
    double angle = std::atan2(acc.imag(), acc.real());
    w_hat = angle / (double)L;
    return true;
}

Receiver::Receiver(const TransmitterConfig& config) : config_(config) {}

VecInt Receiver::receive(const VecComplex& rx_signal)
{
    // 1) 生成同步序列（与TX一致）
    VecInt m_coarse = mseq({ 1,1,1,0,1 });
    VecComplex sync_wide = generateCoarseSync(m_coarse, config_.samp, config_.coarse_length);

    VecInt pss1 = mseq({ 1,0,1,0,1,1,1,0,1 });
    VecInt pss2 = mseq({ 1,0,1,0,1,0,1 });
    VecComplex sync_fine = generateFineSync(pss1, pss2, config_.fine_length);

    VecComplex SYNC = concatSync(sync_wide, sync_fine);

    if (rx_signal.size() <= SYNC.size()) return {};

    // 2) 同步检测：相关峰搜索（只搜前面一段，假设帧不会太晚出现）
    int sync_start = 0;
    double metric = 0.0;
    int search_len = (int)std::min<size_t>(rx_signal.size() - SYNC.size(), (size_t)(200000)); // 可按需调
    if (!findSyncStartNormalized(rx_signal, SYNC, sync_start, metric, search_len)) {
        return {};
    }

    // 设置一个阈值：太小说明没对上（可按你的链路调整）
    // 如果你信道加入了较大噪声/多径，阈值可以降低些，比如 0.05
    if (metric < 0.08) {
        // 没找到可靠同步
        // std::cerr << "[RX] Sync metric too low: " << metric << "\n";
        return {};
    }

    // 3) 截取对齐后的SYNC段
    int sync_end = sync_start + (int)SYNC.size();
    if (sync_end > (int)rx_signal.size()) return {};

    VecComplex rx_sync(rx_signal.begin() + sync_start, rx_signal.begin() + sync_end);

    // 4) 用“精同步重复段”估计 CFO，并补偿（对整段信号从 sync_start 开始补偿即可）
    double w_hat = 0.0; // rad/sample
    bool ok_cfo = estimateCFOFromRepeatedFineSync(
        // 只需要把 rx_sync 中的 fine 部分拿出来：它在 SYNC 尾部
        VecComplex(rx_sync.end() - (int)sync_fine.size(), rx_sync.end()),
        sync_fine,
        config_.fine_length,
        w_hat
    );

    // 拷贝一份用于后续解调
    VecComplex rx_aligned(rx_signal.begin() + sync_start, rx_signal.end());

    if (ok_cfo && std::isfinite(w_hat)) {
        // 去频偏：乘 exp(-j*w_hat*n)
        for (size_t n = 0; n < rx_aligned.size(); ++n) {
            rx_aligned[n] *= std::conj(cexpj(w_hat * (double)n));
        }
    }

    // 5) 正式截取 payload（这一步才“去掉SYNC”，但起点是检测出来的）
    if (rx_aligned.size() <= SYNC.size()) return {};
    VecComplex payload(rx_aligned.begin() + SYNC.size(), rx_aligned.end());

    // 6) 后续流程保持你原来的：BPSK硬判决 -> 差分解码 -> CCSK解扩 -> RS解码
    VecInt bits = demodulateBPSK(payload);
    VecInt diff_decoded = d_decode(bits);
    VecInt ccsk_decoded = despreadCCSK(diff_decoded);
    VecInt rs_decoded = HXL_RSDecode(ccsk_decoded, 31, 15);

    return rs_decoded;
}

VecInt Receiver::demodulateBPSK(const VecComplex& rx)
{
    VecInt bits;
    for (size_t i = 0; i < rx.size(); i += config_.samp) {
        double val = rx[i].real();
        bits.push_back(val > 0 ? 1 : 0);
    }
    return bits;
}

VecInt Receiver::despreadCCSK(const VecInt& chips)
{
    VecInt decoded;
    int block = 32;

    for (size_t i = 0; i + block <= chips.size(); i += block) {
        int best_k = 0;
        int best_corr = -1;

        for (int k = 0; k < 32; ++k) {
            int corr = 0;
            for (int j = 0; j < 32; ++j) {
                int idx = (j - k + 32) % 32;
                if (chips[i + j] == config_.ccskcode[idx]) corr++;
            }
            if (corr > best_corr) {
                best_corr = corr;
                best_k = k;
            }
        }

        for (int b = 4; b >= 0; --b)
            decoded.push_back((best_k >> b) & 1);
    }
    return decoded;
}