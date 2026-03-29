#include "usrp_driver.h"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <chrono>

USRPDriver::USRPDriver(const Config& cfg) : cfg_(cfg) {}

USRPDriver::~USRPDriver() {
    try {
        stop_rx_worker();
        wait_rx_worker();
    }
    catch (...) {
    }
}

void USRPDriver::init() {
    usrp_ = uhd::usrp::multi_usrp::make(cfg_.device_args);

    if (!cfg_.clock_source.empty()) {
        usrp_->set_clock_source(cfg_.clock_source);
    }
    if (!cfg_.time_source.empty()) {
        usrp_->set_time_source(cfg_.time_source);
    }
    if (!cfg_.tx_subdev.empty()) {
        usrp_->set_tx_subdev_spec(cfg_.tx_subdev);
    }
    if (!cfg_.rx_subdev.empty()) {
        usrp_->set_rx_subdev_spec(cfg_.rx_subdev);
    }

    usrp_->set_rx_rate(cfg_.sample_rate);
    usrp_->set_tx_rate(cfg_.sample_rate);

    usrp_->set_rx_freq(cfg_.center_freq);
    usrp_->set_tx_freq(cfg_.center_freq);

    usrp_->set_rx_gain(cfg_.rx_gain);
    usrp_->set_tx_gain(cfg_.tx_gain);

    if (!cfg_.rx_antenna.empty()) {
        usrp_->set_rx_antenna(cfg_.rx_antenna);
    }
    if (!cfg_.tx_antenna.empty()) {
        usrp_->set_tx_antenna(cfg_.tx_antenna);
    }

    uhd::stream_args_t tx_args("fc32", "sc16");
    uhd::stream_args_t rx_args("fc32", "sc16");

    tx_args.channels = { 0 };
    rx_args.channels = { 0 };

    tx_stream_ = usrp_->get_tx_stream(tx_args);
    rx_stream_ = usrp_->get_rx_stream(rx_args);
}

std::string USRPDriver::get_pp_string() const {
    if (!usrp_) return "USRP not initialized";
    return usrp_->get_pp_string();
}

size_t USRPDriver::send_burst(const VecComplex& samples) {
    if (!usrp_ || !tx_stream_) {
        throw std::runtime_error("USRP not initialized");
    }

    std::vector<std::complex<float>> buff(samples.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        buff[i] = std::complex<float>(
            static_cast<float>(samples[i].real()),
            static_cast<float>(samples[i].imag())
        );
    }

    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst = false;
    md.has_time_spec = false;

    const size_t max_samps = tx_stream_->get_max_num_samps();
    size_t sent_total = 0;

    while (sent_total < buff.size()) {
        size_t nsend = std::min(max_samps, buff.size() - sent_total);

        md.start_of_burst = (sent_total == 0);
        md.end_of_burst = false;

        size_t n = tx_stream_->send(&buff[sent_total], nsend, md);
        sent_total += n;
    }

    uhd::tx_metadata_t md_eob;
    md_eob.start_of_burst = false;
    md_eob.end_of_burst = true;
    md_eob.has_time_spec = false;
    tx_stream_->send("", 0, md_eob);

    return sent_total;
}

void USRPDriver::start_rx_worker(size_t target_samps) {
    if (!usrp_ || !rx_stream_) {
        throw std::runtime_error("USRP not initialized");
    }
    if (rx_running_) {
        throw std::runtime_error("RX worker already running");
    }

    {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        rx_buffer_.clear();
        rx_buffer_.reserve(target_samps);
    }

    rx_stop_flag_ = false;
    rx_running_ = true;
    rx_thread_ = std::thread(&USRPDriver::rx_worker_loop, this, target_samps);
}

void USRPDriver::stop_rx_worker() {
    rx_stop_flag_ = true;
}

void USRPDriver::wait_rx_worker() {
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
}

VecComplex USRPDriver::fetch_rx_buffer() {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    return rx_buffer_;
}

VecComplex USRPDriver::fetch_rx_samples_since(size_t start_index, size_t* out_total_samps) {
    std::lock_guard<std::mutex> lock(rx_mutex_);

    const size_t total = rx_buffer_.size();
    if (out_total_samps) {
        *out_total_samps = total;
    }

    if (start_index >= total) {
        return {};
    }

    return VecComplex(
        rx_buffer_.begin() + static_cast<long long>(start_index),
        rx_buffer_.end()
    );
}

size_t USRPDriver::rx_buffer_size() {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    return rx_buffer_.size();
}

void USRPDriver::rx_worker_loop(size_t target_samps) {
    try {
        uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        cmd.stream_now = true;
        cmd.num_samps = 0;
        rx_stream_->issue_stream_cmd(cmd);

        const size_t max_samps = rx_stream_->get_max_num_samps();
        std::vector<std::complex<float>> buff(max_samps);
        uhd::rx_metadata_t md;

        size_t recv_total = 0;
        int overflow_count = 0;

        while (!rx_stop_flag_ && recv_total < target_samps) {
            size_t want = std::min(max_samps, target_samps - recv_total);

            size_t n = rx_stream_->recv(buff.data(), want, md, cfg_.recv_timeout, false);

            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                continue;
            }

            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
                ++overflow_count;
                std::cerr << "[UHD][RX] Overflow count = " << overflow_count << "\n";
                if (overflow_count >= 5) {
                    std::cerr << "[UHD][RX] Too many overflows, stop RX\n";
                    break;
                }
                continue;
            }

            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                std::cerr << "[UHD][RX] Error: " << md.strerror() << "\n";
                break;
            }

            if (n > 0) {
                std::lock_guard<std::mutex> lock(rx_mutex_);
                const size_t old_size = rx_buffer_.size();
                rx_buffer_.resize(old_size + n);
                for (size_t i = 0; i < n; ++i) {
                    rx_buffer_[old_size + i] = Complex(
                        static_cast<double>(buff[i].real()),
                        static_cast<double>(buff[i].imag())
                    );
                }
                recv_total += n;
            }
        }

        uhd::stream_cmd_t stop_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
        rx_stream_->issue_stream_cmd(stop_cmd);
    }
    catch (const std::exception& e) {
        std::cerr << "[UHD][RX THREAD EXCEPTION] " << e.what() << "\n";
    }
    catch (...) {
        std::cerr << "[UHD][RX THREAD EXCEPTION] unknown\n";
    }

    rx_running_ = false;
}
