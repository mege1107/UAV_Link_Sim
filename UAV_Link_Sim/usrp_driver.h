#pragma once

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/stream.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <atomic>
#include <thread>
#include <vector>
#include <complex>
#include <string>
#include <cstddef>

using Complex = std::complex<double>;
using VecComplex = std::vector<Complex>;

class USRPDriver {
public:
    struct Config {
        std::string device_args = "type=b200";

        double sample_rate = 2e6;
        double center_freq = 2.45e9;
        double tx_gain = 30.0;
        double rx_gain = 30.0;

        std::string tx_antenna = "TX/RX";
        std::string rx_antenna = "RX2";

        // 给 X310 预留
        std::string tx_subdev;
        std::string rx_subdev;
        std::string clock_source;
        std::string time_source;

        double recv_timeout = 0.1;
    };

public:
    explicit USRPDriver(const Config& cfg);
    ~USRPDriver();

    void init();
    std::string get_pp_string() const;

    size_t send_burst(const VecComplex& samples);

    void start_rx_worker(size_t target_samps);
    void stop_rx_worker();
    void wait_rx_worker();
    bool is_rx_running() const;
    size_t get_rx_sample_count() const;
    VecComplex fetch_rx_buffer();

private:
    void rx_worker_loop(size_t target_samps);

private:
    Config cfg_;

    uhd::usrp::multi_usrp::sptr usrp_;
    uhd::tx_streamer::sptr tx_stream_;
    uhd::rx_streamer::sptr rx_stream_;

    std::atomic<bool> rx_stop_flag_{ false };
    std::atomic<bool> rx_running_{ false };
    std::atomic<size_t> rx_valid_samps_{ 0 };
    std::atomic<int> rx_overflow_count_{ 0 };
    std::thread rx_thread_;

    std::vector<std::complex<float>> rx_buffer_raw_;
    std::string effective_device_args_;
    double actual_master_clock_rate_ = 0.0;
    double actual_rx_rate_ = 0.0;
    double actual_tx_rate_ = 0.0;
};
