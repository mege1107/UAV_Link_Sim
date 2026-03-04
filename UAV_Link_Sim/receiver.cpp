#include "receiver.h"
#include "coding.h"
#include "spread_spectrum.h"
#include "sync.h"
#include "utils.h" 
#include <cmath>
#include <iostream>

Receiver::Receiver(const TransmitterConfig& config)
    : config_(config)
{
}

VecInt Receiver::receive(const VecComplex& rx_signal)
{
    // ===============================
    // 1️⃣ 生成同步序列（与TX一致）
    // ===============================
    VecInt m_coarse = mseq({ 1,1,1,0,1 });
    VecComplex sync_wide =
        generateCoarseSync(m_coarse,
            config_.samp,
            config_.coarse_length);

    VecInt pss1 = mseq({ 1,0,1,0,1,1,1,0,1 });
    VecInt pss2 = mseq({ 1,0,1,0,1,0,1 });
    VecComplex sync_fine =
        generateFineSync(pss1, pss2,
            config_.fine_length);

    VecComplex SYNC = concatSync(sync_wide, sync_fine);

    if (rx_signal.size() <= SYNC.size())
        return {};

    // ===============================
    // 2️⃣ 去掉SYNC
    // ===============================
    VecComplex payload(rx_signal.begin() + SYNC.size(),
        rx_signal.end());

    // ===============================
    // 3️⃣ BPSK硬判决
    // ===============================
    VecInt bits = demodulateBPSK(payload);

    // ===============================
    // 4️⃣ ⭐ 差分解码（关键修复）
    // ===============================
    VecInt diff_decoded = d_decode(bits);

    // ===============================
    // 5️⃣ CCSK解扩
    // ===============================
    VecInt ccsk_decoded = despreadCCSK(diff_decoded);

    // ===============================
    // 6️⃣ RS解码（当前为占位）
    // ===============================
    VecInt rs_decoded = HXL_RSDecode(ccsk_decoded, 31, 15);

    return rs_decoded;
}

VecInt Receiver::demodulateBPSK(const VecComplex& rx)
{
    VecInt bits;

    for (size_t i = 0; i < rx.size(); i += config_.samp)
    {
        double val = rx[i].real();
        bits.push_back(val > 0 ? 1 : 0);
    }

    return bits;
}

VecInt Receiver::despreadCCSK(const VecInt& chips)
{
    VecInt decoded;

    int block = 32;

    for (size_t i = 0; i + block <= chips.size(); i += block)
    {
        int best_k = 0;
        int best_corr = -1;

        for (int k = 0; k < 32; ++k)
        {
            int corr = 0;

            for (int j = 0; j < 32; ++j)
            {
                int idx = (j - k + 32) % 32;
                if (chips[i + j] ==
                    config_.ccskcode[idx])
                    corr++;
            }

            if (corr > best_corr)
            {
                best_corr = corr;
                best_k = k;
            }
        }

        // 转回5bit
        for (int b = 4; b >= 0; --b)
            decoded.push_back((best_k >> b) & 1);
    }

    return decoded;
}