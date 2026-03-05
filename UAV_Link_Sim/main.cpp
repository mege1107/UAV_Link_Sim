#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "common_defs.h"
#include "transmitter.h"
#include "receiver.h"
#include "channel.h"

#ifdef WITH_UHD
#include "usrp_driver.h"
#endif

static std::string bits_to_string(const VecInt& bits, size_t n = 64) {
    std::string s;
    size_t m = std::min(n, bits.size());
    s.reserve(m);
    for (size_t i = 0; i < m; ++i) s.push_back(bits[i] ? '1' : '0');
    return s;
}

static double compute_ber(const VecInt& tx, const VecInt& rx, size_t& bit_errors) {
    size_t n = std::min(tx.size(), rx.size());
    bit_errors = 0;
    for (size_t i = 0; i < n; ++i) {
        if ((tx[i] & 1) != (rx[i] & 1)) bit_errors++;
    }
    return (n == 0) ? 0.0 : (double)bit_errors / (double)n;
}

// =========================
// Radio 抽象
// =========================
class IRadio {
public:
    virtual ~IRadio() = default;
    virtual void start_rx() {}
    virtual void stop_rx() {}
    virtual size_t send_burst(const VecComplex& samples) = 0;
    virtual size_t recv_samples(size_t nsamps, VecComplex& out) = 0;
};

class LoopbackRadio final : public IRadio {
public:
    size_t send_burst(const VecComplex& samples) override {
        buf_ = samples;
        return samples.size();
    }
    size_t recv_samples(size_t nsamps, VecComplex& out) override {
        if (nsamps >= buf_.size()) out = buf_;
        else out.assign(buf_.begin(), buf_.begin() + nsamps);
        return out.size();
    }
private:
    VecComplex buf_;
};

class AWGNRadio final : public IRadio {
public:
    explicit AWGNRadio(double snr_db) : snr_db_(snr_db) {}

    size_t send_burst(const VecComplex& samples) override {
        last_tx_ = samples;
        return samples.size();
    }

    size_t recv_samples(size_t nsamps, VecComplex& out) override {
        VecComplex rx = Channel::awgn(last_tx_, snr_db_);
        if (nsamps >= rx.size()) out = rx;
        else out.assign(rx.begin(), rx.begin() + nsamps);
        return out.size();
    }
private:
    double snr_db_;
    VecComplex last_tx_;
};

#ifdef WITH_UHD
class USRPRadio final : public IRadio {
public:
    explicit USRPRadio(const USRPDriver::Config& cfg) : usrp_(cfg) { usrp_.init(); }
    void start_rx() override { usrp_.start_rx_now(); }
    void stop_rx() override { usrp_.stop_rx(); }

    size_t send_burst(const VecComplex& samples) override { return usrp_.send_burst(samples); }
    size_t recv_samples(size_t nsamps, VecComplex& out) override { return usrp_.recv_samples(nsamps, out); }
private:
    USRPDriver usrp_;
};
#endif

enum class LinkMode { LOOPBACK, AWGN, USRP };

int main() {
    // ========= 模式选择（没B210先用 LOOPBACK / AWGN）=========
    LinkMode mode = LinkMode::AWGN;
    //LinkMode mode = LinkMode::LOOPBACK;
    // LinkMode mode = LinkMode::USRP;

    // ========= TX循环发送参数 =========
    const int tx_repeat_frames = 30;  // 发 30 帧（未来USRP时很关键）
    const int rx_capture_frames = 10; // 接收窗口按“帧数倍数”来抓（越大越稳，越大越慢）
    const double awgn_snr_db = 80.0;

    // ========= 配置 =========
    TransmitterConfig cfg;
    cfg.function = FunctionType::Telemetry;
    cfg.modulation = ModulationType::BPSK;
    cfg.n = 10;
    cfg.frame_bit = 75;
    cfg.samp = 8;
    cfg.connect = (mode == LinkMode::USRP);

    // ========= 先构造 TX =========
    Transmitter tx(cfg);

    // ========= 生成“单帧”基带（此时 TX 内部通常会确定 fs）=========
    VecComplex one_frame_sig = tx.generateTransmitSignal();
    VecInt one_frame_bits = tx.getLastSourceBits();

    // ========= 关键：把 TX 内部算出的 fs 写回到共享 cfg =========
    // 这样 Receiver（持有 cfg 引用）才能拿到正确 fs
    cfg.fs = tx.getFS();

    // ========= 再构造 RX（方案一：Receiver 内部引用 cfg）=========
    Receiver rx(cfg);

    std::cout << "One-frame bits: " << one_frame_bits.size()
        << ", samples: " << one_frame_sig.size()
        << ", fs: " << tx.getFS() << " Hz\n";

    // ========= 构造“循环发送 burst” =========
    VecComplex tx_burst;
    tx_burst.reserve((size_t)tx_repeat_frames * one_frame_sig.size());
    for (int i = 0; i < tx_repeat_frames; ++i) {
        tx_burst.insert(tx_burst.end(), one_frame_sig.begin(), one_frame_sig.end());
    }

    // ========= 选择 radio =========
    std::unique_ptr<IRadio> radio;

    if (mode == LinkMode::LOOPBACK) {
        radio = std::make_unique<LoopbackRadio>();
        std::cout << "[MODE] LOOPBACK\n";
    }
    else if (mode == LinkMode::AWGN) {
        Channel::setSeed(123);
        radio = std::make_unique<AWGNRadio>(awgn_snr_db);
        std::cout << "[MODE] AWGN, SNR=" << awgn_snr_db << " dB\n";
    }
    else {
#ifndef WITH_UHD
        std::cerr << "[MODE] USRP selected but WITH_UHD is not enabled.\n";
        return 1;
#else
        USRPDriver::Config uc;
        uc.device_args = "type=b200";
        uc.sample_rate = tx.getFS();
        uc.center_freq = 2.45e9;
        uc.tx_gain = 20;
        uc.rx_gain = 20;
        uc.tx_antenna = "TX/RX";
        uc.rx_antenna = "RX2";
        radio = std::make_unique<USRPRadio>(uc);
        std::cout << "[MODE] USRP\n";
#endif
    }

    // ========= 发送 & 接收 =========
    radio->start_rx();
    radio->send_burst(tx_burst);

    // “大窗口捕获”：按帧数倍数收
    const size_t rx_need = (size_t)rx_capture_frames * one_frame_sig.size();
    VecComplex rx_sig;
    radio->recv_samples(rx_need, rx_sig);
    radio->stop_rx();

    std::cout << "TX burst samples: " << tx_burst.size() << "\n";
    std::cout << "RX captured samples: " << rx_sig.size() << "\n";

    // ========= 解调恢复 =========
    VecInt rx_bits = rx.receive(rx_sig);

    // ========= 打印 & BER（先对齐前 min 长度）=========
    std::cout << "TX bits (first 64): " << bits_to_string(one_frame_bits, 64) << "\n";
    std::cout << "RX bits (first 64): " << bits_to_string(rx_bits, 64) << "\n";
    std::cout << "RX bits total: " << rx_bits.size() << "\n";

    size_t bit_errors = 0;
    double ber = compute_ber(one_frame_bits, rx_bits, bit_errors);

    std::cout << "Compared bits: " << std::min(one_frame_bits.size(), rx_bits.size()) << "\n";
    std::cout << "Bit errors: " << bit_errors << "\n";
    std::cout << "BER = " << std::setprecision(6) << ber << "\n";

    // ========= 多帧BER统计 =========
    size_t total_bit_errors = 0;
    size_t total_compared_bits = 0;
    double average_ber = 0.0;

    const int bits_per_frame = (int)one_frame_bits.size();
    const int total_rx_frames = (bits_per_frame > 0) ? (int)(rx_bits.size() / (size_t)bits_per_frame) : 0;

    if (total_rx_frames > 0 && bits_per_frame > 0) {
        for (int i = 0; i < total_rx_frames; ++i) {
            size_t frame_errors = 0;
            size_t frame_start = (size_t)i * (size_t)bits_per_frame;
            VecInt rx_frame(rx_bits.begin() + frame_start,
                rx_bits.begin() + frame_start + (size_t)bits_per_frame);
            compute_ber(one_frame_bits, rx_frame, frame_errors);
            total_bit_errors += frame_errors;
            total_compared_bits += (size_t)bits_per_frame;
        }
        average_ber = (double)total_bit_errors / (double)total_compared_bits;
    }

    std::cout << "Total decoded frames: " << total_rx_frames << "\n";
    std::cout << "Total compared bits: " << total_compared_bits << "\n";
    std::cout << "Total bit errors: " << total_bit_errors << "\n";
    std::cout << "Average BER = " << std::setprecision(6) << average_ber << "\n";

    return 0;
}