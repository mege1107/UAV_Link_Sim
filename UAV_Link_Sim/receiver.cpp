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
    int search_len
) {
    if (rx.size() < sync.size() || sync.empty()) return false;

    const int N = (int)rx.size();
    const int M = (int)sync.size();
    const int K = std::min(N - M, std::max(0, search_len));

    double Es = 0.0;
    for (const auto& s : sync) Es += std::norm(s);
    if (Es <= 0) return false;

    best_metric = -1.0;
    best_idx = 0;

    for (int i = 0; i <= K; ++i) {
        Complex dot(0.0, 0.0);
        double Er = 0.0;

        for (int k = 0; k < M; ++k) {
            dot += std::conj(sync[k]) * rx[i + k];
            Er += std::norm(rx[i + k]);
        }
        if (Er <= 0) continue;

        double metric = std::norm(dot) / (Es * Er);
        if (metric > best_metric) {
            best_metric = metric;
            best_idx = i;
        }
    }
    return true;
}

static bool estimateCFOFromRepeatedFineSync(
    const VecComplex& rx_sync,
    const VecComplex& fine_sync,
    int fine_length,
    double& w_hat
) {
    if (fine_length < 2) return false;
    if (rx_sync.size() != fine_sync.size()) return false;

    const int total = (int)fine_sync.size();
    if (total % fine_length != 0) return false;
    const int L = total / fine_length;
    if (L <= 0 || total < 2 * L) return false;

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

    // 2) 同步检测：相关峰搜索
    int sync_start = 0;
    double metric = 0.0;
    int search_len = (int)std::min<size_t>(rx_signal.size() - SYNC.size(), (size_t)(200000));
    if (!findSyncStartNormalized(rx_signal, SYNC, sync_start, metric, search_len)) {
        return {};
    }

    if (metric < 0.08) {
        return {};
    }

    // 3) 截取对齐后的SYNC段
    int sync_end = sync_start + (int)SYNC.size();
    if (sync_end > (int)rx_signal.size()) return {};

    VecComplex rx_sync(rx_signal.begin() + sync_start, rx_signal.begin() + sync_end);

    // 4) CFO估计与补偿
    double w_hat = 0.0;
    bool ok_cfo = estimateCFOFromRepeatedFineSync(
        VecComplex(rx_sync.end() - (int)sync_fine.size(), rx_sync.end()),
        sync_fine,
        config_.fine_length,
        w_hat
    );

    VecComplex rx_aligned(rx_signal.begin() + sync_start, rx_signal.end());

    if (ok_cfo && std::isfinite(w_hat)) {
        for (size_t n = 0; n < rx_aligned.size(); ++n) {
            rx_aligned[n] *= std::conj(cexpj(w_hat * (double)n));
        }
    }

    // 5) 截取 payload（去掉SYNC）
    if (rx_aligned.size() <= SYNC.size()) return {};

    VecComplex payload(rx_aligned.begin() + SYNC.size(), rx_aligned.end());

    // ✅ 5.5) 裁掉帧尾 ZP（Zero Padding）
    const int zp_samp = std::max(0, config_.zp_sym) * std::max(1, config_.samp);
    if (zp_samp > 0 && (int)payload.size() > zp_samp) {
        payload.resize(payload.size() - (size_t)zp_samp);
    }

    // 6) 后续流程：BPSK硬判决 -> 差分解码 -> CCSK解扩 -> RS解码
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