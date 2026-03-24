#pragma once

#include <string>
#include <vector>

#include "sim_runner.h"

struct TxOnlyResult {
    size_t tx_frame_count = 0;
    size_t tx_sample_count = 0;
    double fs = 0.0;

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

    std::string log_text;
};

struct RxOnlyResult {
    size_t rx_sample_count = 0;
    size_t decoded_bits_count = 0;
    size_t decoded_frames = 0;
    double fs = 0.0;

    std::vector<double> rx_waveform;
    WaveformType rx_waveform_type = WaveformType::REAL;

    std::vector<double> rx_spectrum_freq;
    std::vector<double> rx_spectrum_mag;

    std::vector<double> constellation_i;
    std::vector<double> constellation_q;

    std::string log_text;
};

struct RxFileResult {
    size_t rx_sample_count = 0;
    size_t decoded_bits_count = 0;
    size_t decoded_frames = 0;
    double fs = 0.0;

    bool file_saved = false;
    std::string recovered_filename;
    std::string saved_file_path;

    std::vector<double> constellation_i;
    std::vector<double> constellation_q;

    std::string log_text;
};

TxOnlyResult run_tx_role_once(
    const std::string& tx_device_args,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern,
    SourceMode source_mode,
    const std::string& input_file_path,
    int tx_repeat_frames);

RxOnlyResult run_rx_role_once(
    const std::string& rx_device_args,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern,
    int expected_frames);

RxFileResult run_rx_file_role_once(
    const std::string& rx_device_args,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern,
    const std::string& output_file_path,
    int max_frames);
