#include "usrp_driver.h"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

namespace {
bool has_device_arg_key(const std::string& args, const std::string& key)
{
    const std::string needle = key + "=";
    size_t pos = args.find(needle);
    while (pos != std::string::npos) {
        if (pos == 0 || args[pos - 1] == ',') {
            return true;
        }
        pos = args.find(needle, pos + needle.size());
    }
    return false;
}

void append_device_arg_if_missing(std::string& args, const std::string& key, const std::string& value)
{
    if (has_device_arg_key(args, key)) {
        return;
    }

    if (!args.empty() && args.back() != ',') {
        args.push_back(',');
    }
    args += key;
    args.push_back('=');
    args += value;
}

bool looks_like_b2xx_args(const std::string& args)
{
    return args.find("type=b200") != std::string::npos ||
        args.find("type=b210") != std::string::npos ||
        args.find("type=b200mini") != std::string::npos ||
        args.find("type=b205mini") != std::string::npos ||
        args.find("type=b206mini") != std::string::npos;
}

double choose_b2xx_master_clock_rate(double sample_rate)
{
    if (!std::isfinite(sample_rate) || sample_rate <= 0.0) {
        return 0.0;
    }

    static constexpr double kMinMcr = 5e6;
    static constexpr double kMaxRecommendedMcr = 56e6;
    static constexpr double kMaxAbsoluteMcr = 61.44e6;

    for (int mult = 128; mult >= 1; --mult) {
        const double cand = sample_rate * static_cast<double>(mult);
        if (cand >= kMinMcr && cand <= kMaxRecommendedMcr) {
            return cand;
        }
    }

    for (int mult = 128; mult >= 1; --mult) {
        const double cand = sample_rate * static_cast<double>(mult);
        if (cand >= kMinMcr && cand <= kMaxAbsoluteMcr) {
            return cand;
        }
    }

    return 0.0;
}
} // namespace

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
    effective_device_args_ = cfg_.device_args;
    const bool b2xx_hint = looks_like_b2xx_args(effective_device_args_);
    if (b2xx_hint) {
        append_device_arg_if_missing(effective_device_args_, "recv_frame_size", "1024");
    }

    usrp_ = uhd::usrp::multi_usrp::make(effective_device_args_);

    if (b2xx_hint && !has_device_arg_key(effective_device_args_, "master_clock_rate")) {
        const double mcr = choose_b2xx_master_clock_rate(cfg_.sample_rate);
        if (mcr > 0.0) {
            usrp_->set_master_clock_rate(mcr);
        }
    }

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

    actual_master_clock_rate_ = usrp_->get_master_clock_rate();
    actual_rx_rate_ = usrp_->get_rx_rate();
    actual_tx_rate_ = usrp_->get_tx_rate();

    uhd::stream_args_t tx_args("fc32", "sc16");
    uhd::stream_args_t rx_args("fc32", "sc16");

    tx_args.channels = { 0 };
    rx_args.channels = { 0 };

    tx_stream_ = usrp_->get_tx_stream(tx_args);
    rx_stream_ = usrp_->get_rx_stream(rx_args);
}

std::string USRPDriver::get_pp_string() const {
    if (!usrp_) return "USRP not initialized";

    std::ostringstream oss;
    oss << usrp_->get_pp_string();
    oss << "\n[UHD CFG] effective_device_args=" << effective_device_args_;
    oss << "\n[UHD CFG] actual_master_clock_rate=" << actual_master_clock_rate_;
    oss << "\n[UHD CFG] actual_rx_rate=" << actual_rx_rate_;
    oss << "\n[UHD CFG] actual_tx_rate=" << actual_tx_rate_;
    return oss.str();
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

    rx_stop_flag_ = false;
    rx_valid_samps_ = 0;
    rx_overflow_count_ = 0;
    rx_buffer_raw_.clear();
    rx_buffer_raw_.resize(target_samps);
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

bool USRPDriver::is_rx_running() const {
    return rx_running_.load(std::memory_order_acquire);
}

size_t USRPDriver::get_rx_sample_count() const {
    return rx_valid_samps_.load(std::memory_order_acquire);
}

VecComplex USRPDriver::fetch_rx_buffer() {
    const size_t valid_samps = rx_valid_samps_.load(std::memory_order_acquire);
    VecComplex out;
    out.reserve(valid_samps);

    for (size_t i = 0; i < valid_samps; ++i) {
        const auto& s = rx_buffer_raw_[i];
        out.emplace_back(
            static_cast<double>(s.real()),
            static_cast<double>(s.imag()));
    }

    return out;
}

void USRPDriver::rx_worker_loop(size_t target_samps) {
    try {
        uhd::set_thread_priority_safe();

        uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        cmd.stream_now = true;
        cmd.num_samps = 0;
        rx_stream_->issue_stream_cmd(cmd);

        const size_t max_samps = rx_stream_->get_max_num_samps();
        uhd::rx_metadata_t md;

        size_t recv_total = 0;

        while (!rx_stop_flag_ && recv_total < target_samps) {
            size_t want = std::min(max_samps, target_samps - recv_total);

            size_t n = rx_stream_->recv(
                rx_buffer_raw_.data() + static_cast<std::ptrdiff_t>(recv_total),
                want,
                md,
                cfg_.recv_timeout,
                false);

            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                continue;
            }

            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
                const int overflow_count =
                    rx_overflow_count_.fetch_add(1, std::memory_order_relaxed) + 1;
                if (overflow_count <= 5 || (overflow_count % 50) == 0) {
                    std::cerr << "[UHD][RX] Overflow count = " << overflow_count
                        << " recv_total=" << recv_total
                        << " target_samps=" << target_samps
                        << "\n";
                }
                continue;
            }

            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                std::cerr << "[UHD][RX] Error: " << md.strerror() << "\n";
                break;
            }

            if (n > 0) {
                recv_total += n;
                rx_valid_samps_.store(recv_total, std::memory_order_release);
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
