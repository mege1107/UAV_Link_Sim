#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <exception>
#include <thread>
#include <chrono>

#include "common_defs.h"
#include "transmitter.h"
#include "receiver.h"
#include "channel.h"

#define WITH_UHD
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

class IRadio {
public:
    virtual ~IRadio() = default;

    virtual void start_rx_worker(size_t target_samps) = 0;
    virtual void stop_rx_worker() = 0;
    virtual void wait_rx_worker() = 0;
    virtual VecComplex fetch_rx_buffer() = 0;

    virtual size_t send_burst(const VecComplex& samples) = 0;
};

#ifdef WITH_UHD
class USRPRadio final : public IRadio {
public:
    explicit USRPRadio(const USRPDriver::Config& cfg) : usrp_(cfg) {
        usrp_.init();
        std::cout << usrp_.get_pp_string() << std::endl;
    }

    void start_rx_worker(size_t target_samps) override {
        usrp_.start_rx_worker(target_samps);
    }

    void stop_rx_worker() override {
        usrp_.stop_rx_worker();
    }

    void wait_rx_worker() override {
        usrp_.wait_rx_worker();
    }

    VecComplex fetch_rx_buffer() override {
        return usrp_.fetch_rx_buffer();
    }

    size_t send_burst(const VecComplex& samples) override {
        return usrp_.send_burst(samples);
    }

private:
    USRPDriver usrp_;
};
#endif

int main() {
    try {

        // =========================
        // 配置
        // =========================
        TransmitterConfig cfg;

        cfg.function = FunctionType::Telemetry;
        cfg.modulation = ModulationType::QAM;

        cfg.n = 10;
        cfg.frame_bit = 75;
        cfg.samp = 8;
        cfg.zp_sym = 33;

        cfg.Rb = 18900;   // ≈2 MHz

        cfg.connect = true;

        // =========================
        // TX / RX
        // =========================
        Transmitter tx(cfg);

        VecComplex one_frame_sig = tx.generateTransmitSignal();
        VecInt one_frame_bits = tx.getLastSourceBits();

        cfg.fs = tx.getFS();

        Receiver rx(cfg);

        std::cout << "One-frame bits: " << one_frame_bits.size()
            << ", samples: " << one_frame_sig.size()
            << ", fs: " << tx.getFS() << " Hz\n";

        // =========================
// TX burst
// =========================
        const int tx_repeat_frames = 20;
        const size_t pre_zeros = 200000;
        const size_t post_zeros = 5000;

        VecComplex tx_burst;
        tx_burst.reserve(
            pre_zeros +
            (size_t)tx_repeat_frames * one_frame_sig.size() +
            post_zeros
        );

        tx_burst.insert(tx_burst.end(), pre_zeros, Complex(0.0, 0.0));

        for (int i = 0; i < tx_repeat_frames; ++i) {
            tx_burst.insert(tx_burst.end(),
                one_frame_sig.begin(),
                one_frame_sig.end());
        }

        tx_burst.insert(tx_burst.end(), post_zeros, Complex(0.0, 0.0));

        std::cout << "[TX CFG] tx_repeat_frames = " << tx_repeat_frames
            << ", pre_zeros = " << pre_zeros
            << ", post_zeros = " << post_zeros << "\n";

#ifdef WITH_UHD

        // =========================
        // USRP 参数
        // =========================
        USRPDriver::Config uc;

        uc.device_args = "type=b200";
        uc.sample_rate = tx.getFS();
        uc.center_freq = 2.45e9;

        uc.tx_gain = 35;
        uc.rx_gain = 40;

        uc.tx_antenna = "TX/RX";
        uc.rx_antenna = "RX2";

        std::unique_ptr<IRadio> radio =
            std::make_unique<USRPRadio>(uc);

        std::cout << "[MODE] USRP\n";
        std::cout << "[USRP CFG] sample_rate = " << uc.sample_rate
            << ", fc = " << uc.center_freq
            << ", tx_gain = " << uc.tx_gain
            << ", rx_gain = " << uc.rx_gain << "\n";

        // =========================
        // RX 长度
        // =========================
        const int rx_extra_frames = 10;
        const size_t rx_need =
            tx_burst.size() + (size_t)rx_extra_frames * one_frame_sig.size();

        std::cout << "[RX CFG] rx_extra_frames = " << rx_extra_frames
            << ", rx_need = " << rx_need << "\n";

        // =========================
        // 启动 RX 线程
        // =========================
        radio->start_rx_worker(rx_need);

        // 等 RX 线程真正进入 recv()
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // =========================
        // 发送 burst
        // =========================
        const size_t tx_sent = radio->send_burst(tx_burst);

        // =========================
        // 等 RX 结束
        // =========================
        radio->wait_rx_worker();

        VecComplex rx_sig = radio->fetch_rx_buffer();
        const size_t rx_got = rx_sig.size();

        std::cout << "TX burst samples: " << tx_burst.size()
            << ", actually sent: " << tx_sent << "\n";

        std::cout << "RX requested samples: " << rx_need
            << ", captured samples: " << rx_got << "\n";

#else
        std::cerr << "UHD disabled\n";
        return 1;
#endif

        // =========================
        // 接收信号统计
        // =========================
        double avg_power = 0.0;
        double max_abs = 0.0;

        for (const auto& s : rx_sig) {
            double p = std::norm(s);
            avg_power += p;

            double a = std::abs(s);
            if (a > max_abs) max_abs = a;
        }

        avg_power /= std::max<size_t>(1, rx_sig.size());

        std::cout << "[DBG][RX] avg_power = " << avg_power
            << ", max_abs = " << max_abs << "\n";

        // =========================
        // 解调
        // =========================
        VecInt rx_bits = rx.receive(rx_sig);

        std::cout << "TX bits (first 64): "
            << bits_to_string(one_frame_bits, 64) << "\n";

        std::cout << "RX bits (first 64): "
            << bits_to_string(rx_bits, 64) << "\n";

        std::cout << "RX bits total: " << rx_bits.size() << "\n";

        const size_t bits_per_frame = one_frame_bits.size();
        const size_t decoded_frames = (bits_per_frame > 0) ? (rx_bits.size() / bits_per_frame) : 0;

        size_t total_compared_bits = 0;
        size_t total_bit_errors = 0;

        for (size_t i = 0; i < decoded_frames; ++i) {
            VecInt rx_frame(
                rx_bits.begin() + i * bits_per_frame,
                rx_bits.begin() + (i + 1) * bits_per_frame
            );

            size_t frame_errors = 0;
            compute_ber(one_frame_bits, rx_frame, frame_errors);

            total_bit_errors += frame_errors;
            total_compared_bits += bits_per_frame;

            std::cout << "[BER] Frame " << (i + 1)
                << ": errors = " << frame_errors
                << " / " << bits_per_frame
                << ", BER = "
                << std::setprecision(6)
                << ((double)frame_errors / (double)bits_per_frame)
                << "\n";
        }

        double total_ber = (total_compared_bits > 0)
            ? (double)total_bit_errors / (double)total_compared_bits
            : 0.0;

        std::cout << "Compared bits: " << total_compared_bits << "\n";
        std::cout << "Bit errors: " << total_bit_errors << "\n";
        std::cout << "BER = " << std::setprecision(6) << total_ber << "\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "[EXCEPTION] " << e.what() << std::endl;
        return -1;
    }
    catch (...) {
        std::cerr << "[EXCEPTION] unknown exception\n";
        return -1;
    }
}