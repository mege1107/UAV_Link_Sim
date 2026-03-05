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

// =========================
// 工具：打印/BER
// =========================
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

// 纯回环：TX samples 直接返回给 RX（无噪声）
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

// AWGN：调用你现有 Channel::awgn()
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
    explicit USRPRadio(const USRPDriver::Config& cfg) : usrp_(cfg) {
        usrp_.init();
    }
    void start_rx() override { usrp_.start_rx_now(); }
    void stop_rx() override { usrp_.stop_rx(); }

    size_t send_burst(const VecComplex& samples) override {
        return usrp_.send_burst(samples);
    }
    size_t recv_samples(size_t nsamps, VecComplex& out) override {
        return usrp_.recv_samples(nsamps, out);
    }
private:
    USRPDriver usrp_;
};
#endif

enum class LinkMode { LOOPBACK, AWGN, USRP };

int main() {
    // =========================
    // 1) 选择模式：你现在没B210，建议 LOOPBACK 或 AWGN
    // =========================
    LinkMode mode = LinkMode::AWGN;
    // LinkMode mode = LinkMode::AWGN;
    // LinkMode mode = LinkMode::USRP;

    // =========================
    // 2) 配置（按你当前默认值来，也可以改）
    // =========================
    TransmitterConfig cfg;
    cfg.function = FunctionType::Telemetry;
    cfg.modulation = ModulationType::BPSK;

    cfg.n = 10;
    cfg.frame_bit = 10000;
    cfg.samp = 8;

    cfg.connect = (mode == LinkMode::USRP);

    // =========================
    // 3) 构建TX/RX，并生成信号
    // =========================
    Transmitter tx(cfg);
    Receiver rx(cfg);

    VecComplex tx_sig = tx.generateTransmitSignal();
    VecInt tx_bits = tx.getLastSourceBits();

    std::cout << "TX bits (first 64): " << bits_to_string(tx_bits, 64) << "\n";
    std::cout << "TX bits total: " << tx_bits.size() << "\n";
    std::cout << "TX samples: " << tx_sig.size() << "\n";
    std::cout << "TX fs (Hz): " << tx.getFS() << "\n";

    // =========================
    // 4) 构建 Radio
    // =========================
    std::unique_ptr<IRadio> radio;

    if (mode == LinkMode::LOOPBACK) {
        radio = std::make_unique<LoopbackRadio>();
        std::cout << "[MODE] LOOPBACK\n";
    }
    else if (mode == LinkMode::AWGN) {
        double snr_db = -10.0;
        Channel::setSeed(123);
        radio = std::make_unique<AWGNRadio>(snr_db);
        std::cout << "[MODE] AWGN, SNR=" << snr_db << " dB\n";
    }
    else {
#ifndef WITH_UHD
        std::cerr << "[MODE] USRP selected but WITH_UHD is not enabled.\n";
        std::cerr << "Open Project Properties -> C/C++ -> Preprocessor -> add WITH_UHD, "
            "and link UHD, then rebuild.\n";
        return 1;
#else
        USRPDriver::Config uc;
        uc.device_args = "type=b200";
        uc.sample_rate = tx.getFS();   // ✅ 用 TX 已计算的 fs
        uc.center_freq = 2.45e9;
        uc.tx_gain = 20;
        uc.rx_gain = 20;
        uc.tx_antenna = "TX/RX";
        uc.rx_antenna = "RX2";

        radio = std::make_unique<USRPRadio>(uc);
        std::cout << "[MODE] USRP\n";
#endif
    }

    // =========================
    // 5) 发送 + 接收
    // =========================
    radio->start_rx();
    radio->send_burst(tx_sig);

    VecComplex rx_sig;
    radio->recv_samples(tx_sig.size(), rx_sig);
    radio->stop_rx();

    std::cout << "RX samples: " << rx_sig.size() << "\n";

    // =========================
    // 6) RX恢复比特
    // =========================
    VecInt rx_bits = rx.receive(rx_sig);

    std::cout << "RX bits (first 64): " << bits_to_string(rx_bits, 64) << "\n";
    std::cout << "RX bits total: " << rx_bits.size() << "\n";

    // =========================
    // 7) BER
    // =========================
    size_t bit_errors = 0;
    double ber = compute_ber(tx_bits, rx_bits, bit_errors);

    std::cout << "Total bits (compared): " << std::min(tx_bits.size(), rx_bits.size()) << "\n";
    std::cout << "Bit errors: " << bit_errors << "\n";
    std::cout << "BER = " << std::setprecision(6) << ber << "\n";

    return 0;
}