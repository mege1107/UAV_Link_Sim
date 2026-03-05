#include "transmitter.h"
#include <fstream>
#include <iostream>
#include <stdexcept>

Transmitter::Transmitter(const TransmitterConfig& config) : config_(config) {
    calculateFS();
    setRandomSeed(1); // 对应MATLAB rng(1)
}

void Transmitter::calculateFS() {
    // 对应MATLAB: fs = round(Rb*FrameRate*RsRate*spsp*samp);
    config_.fs = std::round(
        config_.Rb * config_.FrameRate *
        config_.RsRate * config_.spsp * config_.samp
    );
}

VecInt Transmitter::generateSourceData() {
    int total_bits = config_.frame_bit * config_.n;
    return generateRandomBits(total_bits);
}

VecComplex Transmitter::generateTransmitSignal() {
    // 1. 信源
    last_source_bits_ = generateSourceData();
    VecInt dataIn = last_source_bits_;

    // 2. RS编码 (31,15)
    VecInt encoded_msg = HXL_RSCode(dataIn, 31, 15);

    // 3. CCSK (32,5) 扩频
    VecInt ccsk_msg = ZCY_CCSK32(encoded_msg, config_.ccskcode);

    // 4. 生成同步序列
    VecInt m_coarse = mseq({ 1, 1, 1, 0, 1 });
    VecComplex sync_wide = generateCoarseSync(m_coarse, config_.samp, config_.coarse_length);

    VecInt pss1 = mseq({ 1, 0, 1, 0, 1, 1, 1, 0, 1 });
    VecInt pss2 = mseq({ 1, 0, 1, 0, 1, 0, 1 });
    VecComplex sync_fine = generateFineSync(pss1, pss2, config_.fine_length);

    VecComplex SYNC = concatSync(sync_wide, sync_fine);

    // 5. 调制与信号处理
    VecComplex txSig;

    if (config_.function == FunctionType::RemoteControl) {
        // === 遥控方案：MSK + 跳频 + 脉冲 ===
        int zeros_bit = 33 * config_.samp;
        int pulse_len = (int)config_.ccskcode.size() * config_.samp;

        VecInt tx_bits_mat;
        for (size_t i = 0; i < ccsk_msg.size(); i += 32) {
            for (int j = 0; j < 32 && (i + j) < ccsk_msg.size(); ++j) {
                tx_bits_mat.push_back(ccsk_msg[i + j]);
            }
        }

        txSig = mskmod(tx_bits_mat, config_.samp);

        int nt = 1;
        int num_freqs = config_.Pulse_num / nt;
        VecDouble frq_seq = generate_sequence(-13, 13, 3, num_freqs * config_.n, 1);

        txSig = frequencyHop(txSig, frq_seq, pulse_len, nt, config_.fs);

        VecComplex pulsed;
        int pulses_per_group = config_.Pulse_num * config_.n;
        (void)zeros_bit;
        (void)pulses_per_group;
        pulsed = txSig;
    }
    else {
        // === 遥测方案 ===
        if (config_.modulation == ModulationType::BPSK) {
            VecInt diff = d_encode(ccsk_msg);
            txSig = bpskmod(diff, config_.samp);
        }
        else if (config_.modulation == ModulationType::QPSK) {
            double fs_local = config_.fs;
            txSig = qpskmod(ccsk_msg, config_.samp, fs_local);
            config_.fs = fs_local;
        }
        else if (config_.modulation == ModulationType::QAM) {
            double fs_local = config_.fs;
            txSig = qammod(ccsk_msg, config_.samp, fs_local);
            config_.fs = fs_local;
        }
        else if (config_.modulation == ModulationType::OOK) {
            txSig = ookmod(ccsk_msg, config_.samp);
        }
        else if (config_.modulation == ModulationType::FSK) {
            double deta_f = 1e6;
            txSig = fskmod(ccsk_msg, 2, deta_f, config_.samp, config_.fs);
        }
        else if (config_.modulation == ModulationType::FM) {
            double kf = 75e5;
            txSig = fmmod(ccsk_msg, config_.samp, config_.fs, kf);
        }
    }

    // 6. 添加同步头
    VecComplex txSig_Sync = SYNC;
    txSig_Sync.insert(txSig_Sync.end(), txSig.begin(), txSig.end());

    //  7. 在“帧末尾”添加 ZP（Zero Padding）
    //    以“符号数 zp_sym”为单位，最终变为 zp_sym*samp 个复采样
    const int zp_samp = std::max(0, config_.zp_sym) * std::max(1, config_.samp);
    if (zp_samp > 0) {
        txSig_Sync.insert(txSig_Sync.end(), (size_t)zp_samp, Complex(0.0, 0.0));
    }

    return txSig_Sync;
}

void Transmitter::saveSignal(const std::string& filename, const VecComplex& signal) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Cannot open file for writing");
    }

    size_t len = signal.size();
    file.write(reinterpret_cast<const char*>(&len), sizeof(len));

    for (const auto& c : signal) {
        double re = c.real();
        double im = c.imag();
        file.write(reinterpret_cast<const char*>(&re), sizeof(re));
        file.write(reinterpret_cast<const char*>(&im), sizeof(im));
    }

    std::cout << "Signal saved to " << filename << " (" << len << " samples)" << std::endl;
}