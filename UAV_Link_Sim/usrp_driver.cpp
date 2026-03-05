#include "usrp_driver.h"

#include <stdexcept>
#include <iostream>
#include <thread>
#include <chrono>

// UHD
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/types/tune_request.hpp>

USRPDriver::USRPDriver(const Config& cfg) : cfg_(cfg) {}
USRPDriver::~USRPDriver() {
    try { stop_rx(); }
    catch (...) {}
}

void USRPDriver::init() {
    // 1) 创建设备
    usrp_ = uhd::usrp::multi_usrp::make(cfg_.device_args);

    // 2) 基本参数：采样率、频点、增益、天线、带宽
    usrp_->set_tx_rate(cfg_.sample_rate, cfg_.tx_chan);
    usrp_->set_rx_rate(cfg_.sample_rate, cfg_.rx_chan);

    usrp_->set_tx_freq(uhd::tune_request_t(cfg_.center_freq), cfg_.tx_chan);
    usrp_->set_rx_freq(uhd::tune_request_t(cfg_.center_freq), cfg_.rx_chan);

    usrp_->set_tx_gain(cfg_.tx_gain, cfg_.tx_chan);
    usrp_->set_rx_gain(cfg_.rx_gain, cfg_.rx_chan);

    if (!cfg_.tx_antenna.empty()) usrp_->set_tx_antenna(cfg_.tx_antenna, cfg_.tx_chan);
    if (!cfg_.rx_antenna.empty()) usrp_->set_rx_antenna(cfg_.rx_antenna, cfg_.rx_chan);

    if (cfg_.bandwidth > 0) {
        usrp_->set_tx_bandwidth(cfg_.bandwidth, cfg_.tx_chan);
        usrp_->set_rx_bandwidth(cfg_.bandwidth, cfg_.rx_chan);
    }

    // 3) 等待PLL锁定等（经验上给一点时间更稳）
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 4) 创建 TX/RX streamer
    // CPU格式 fc32（complex<float>），OTW格式 sc16（节省带宽）
    uhd::stream_args_t tx_args("fc32", "sc16");
    tx_args.channels = { (size_t)cfg_.tx_chan };
    tx_stream_ = usrp_->get_tx_stream(tx_args);

    uhd::stream_args_t rx_args("fc32", "sc16");
    rx_args.channels = { (size_t)cfg_.rx_chan };
    rx_stream_ = usrp_->get_rx_stream(rx_args);

    // 5) 可选：打印设备信息（调试）
    std::cout << usrp_->get_pp_string() << std::endl;
}

std::string USRPDriver::get_pp_string() const {
    if (!usrp_) return "";
    return usrp_->get_pp_string();
}

void USRPDriver::start_rx_now() {
    if (!usrp_ || !rx_stream_) throw std::runtime_error("USRP not initialized");
    if (rx_running_) return;

    // 立即开始连续接收
    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    cmd.stream_now = true;
    cmd.num_samps = 0;
    rx_stream_->issue_stream_cmd(cmd);

    rx_running_ = true;
}

void USRPDriver::stop_rx() {
    if (!rx_stream_ || !rx_running_) return;

    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    cmd.stream_now = true;
    rx_stream_->issue_stream_cmd(cmd);

    rx_running_ = false;
}

size_t USRPDriver::send_burst(const VecComplex& samples) {
    if (!usrp_ || !tx_stream_) throw std::runtime_error("USRP not initialized");
    if (samples.empty()) return 0;

    // double -> float
    std::vector<std::complex<float>> buff(samples.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        buff[i] = std::complex<float>((float)samples[i].real(), (float)samples[i].imag());
    }

    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst = false;
    md.has_time_spec = false; // 先不做定时发送（后面要做可置true+time_spec）

    size_t sent_total = 0;
    const size_t max_samps = tx_stream_->get_max_num_samps();
    const double timeout = cfg_.send_timeout;

    while (sent_total < buff.size()) {
        size_t to_send = std::min(max_samps, buff.size() - sent_total);
        size_t n = tx_stream_->send(
            &buff[sent_total], to_send, md, timeout
        );
        sent_total += n;

        // 后续包不再标start_of_burst
        md.start_of_burst = false;
        md.has_time_spec = false;
    }

    // 发一个EOB（end_of_burst）
    md.end_of_burst = true;
    tx_stream_->send("", 0, md, timeout);

    return sent_total;
}

size_t USRPDriver::recv_samples(size_t nsamps, VecComplex& samples_out) {
    if (!usrp_ || !rx_stream_) throw std::runtime_error("USRP not initialized");
    if (!rx_running_) start_rx_now();

    samples_out.clear();
    samples_out.reserve(nsamps);

    const size_t max_samps = rx_stream_->get_max_num_samps();
    std::vector<std::complex<float>> buff(max_samps);

    uhd::rx_metadata_t md;
    size_t got_total = 0;
    const double timeout = cfg_.recv_timeout;

    while (got_total < nsamps) {
        size_t to_recv = std::min(max_samps, nsamps - got_total);
        size_t n = rx_stream_->recv(&buff.front(), to_recv, md, timeout);

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            // 超时就退出，避免死等
            break;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            throw std::runtime_error("RX error: " + md.strerror());
        }

        // float -> double
        for (size_t i = 0; i < n; ++i) {
            samples_out.emplace_back((double)buff[i].real(), (double)buff[i].imag());
        }
        got_total += n;
    }

    return got_total;
}