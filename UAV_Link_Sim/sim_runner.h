#pragma once

#include <string>
#include <vector>
#include <complex>

#include "transmitter.h"

enum class RunMode
{
    LOOPBACK,
    AWGN,
    USRP
};

enum class WaveformType
{
    REAL,
    ENVELOPE
};

struct TestResult
{
    double total_ber = 0.0;
    size_t total_bit_errors = 0;
    size_t total_compared_bits = 0;
    size_t decoded_frames = 0;

    // 发射端波形/频谱/时频图
    std::vector<double> waveform;
    WaveformType waveform_type = WaveformType::REAL;

    std::vector<double> spectrum_freq;
    std::vector<double> spectrum_mag;

    std::vector<double> spectrogram_data;
    int spectrogram_width = 0;
    int spectrogram_height = 0;
    double spectrogram_time_span = 0.0;
    double spectrogram_freq_min = 0.0;
    double spectrogram_freq_max = 0.0;

    // 接收端波形/频谱
    std::vector<double> rx_waveform;
    WaveformType rx_waveform_type = WaveformType::REAL;

    std::vector<double> rx_spectrum_freq;
    std::vector<double> rx_spectrum_mag;

    // ===== 新增：星座图 =====
    std::vector<double> constellation_i;
    std::vector<double> constellation_q;

    // 文件恢复
    bool file_saved = false;
    std::string saved_file_path;

    // 日志
    std::string log_text;
};

struct SweepPoint
{
    double snr_db = 0.0;
    double ber = 0.0;
    size_t decoded_frames = 0;
};

struct SweepResult
{
    std::vector<SweepPoint> points;
    std::string log_text;
};

std::string mode_to_string(RunMode mode);
std::string modulation_to_string(ModulationType m);

ModulationType parse_modulation(const std::string& s);
RunMode parse_mode(const std::string& s);

TestResult run_one_test(
    RunMode mode,
    double awgn_snr_db,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern
);

TestResult run_file_transfer_test(
    RunMode mode,
    double awgn_snr_db,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern,
    const std::string& input_file_path,
    const std::string& output_file_path
);

SweepResult run_awgn_sweep(
    double snr_start,
    double snr_end,
    double snr_step,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation
);