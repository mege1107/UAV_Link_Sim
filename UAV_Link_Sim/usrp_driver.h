#pragma once
#include "common_defs.h"
#include <string>
#include <vector>
#include <complex>
#include <memory>

namespace uhd {
    namespace usrp { class multi_usrp; }
    class tx_streamer;
    class rx_streamer;
}

class USRPDriver {
public:
    struct Config {
        std::string device_args = "";   // e.g. "type=b200" or "serial=XXXXXXXX"
        int tx_chan = 0;
        int rx_chan = 0;

        double sample_rate = 1e6;       // Hz
        double center_freq = 2.45e9;    // Hz
        double tx_gain = 20;            // dB
        double rx_gain = 20;            // dB
        double bandwidth = 0;           // Hz (0 = don't set)
        std::string tx_antenna = "TX/RX";
        std::string rx_antenna = "RX2";

        // 超时/缓冲
        double send_timeout = 3.0;      // seconds
        double recv_timeout = 3.0;      // seconds
    };

    explicit USRPDriver(const Config& cfg);
    ~USRPDriver();

    void init();                 // 创建设备、设参、创建streamer
    void start_rx_now();         // 立即开始接收（连续模式）
    void stop_rx();              // 停止接收

    // 一次性发送一段采样（burst）
    // 会自动做 double->float 转换（你的 VecComplex 是 complex<double>）
    size_t send_burst(const VecComplex& samples);

    // 从设备接收指定数量采样（complex<float>转回complex<double>）
    // 返回实际接收数量；samples_out 会被填充
    size_t recv_samples(size_t nsamps, VecComplex& samples_out);

    // 工具：从UHD读当前实际设置（可打印调试）
    std::string get_pp_string() const;

private:
    Config cfg_;

    std::shared_ptr<uhd::usrp::multi_usrp> usrp_;
    std::shared_ptr<uhd::tx_streamer> tx_stream_;
    std::shared_ptr<uhd::rx_streamer> rx_stream_;

    bool rx_running_ = false;
};