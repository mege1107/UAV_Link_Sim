#include "sim_runner.h"
#include "ofdm_link.h"
#include <random>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <ctime>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iterator>

#include "transmitter.h"
#include "receiver.h"
#include "channel.h"

#define WITH_UHD
#ifdef WITH_UHD
#include "usrp_driver.h"
#endif

static constexpr bool SHOW_PURE_MSK_ONLY = true;
static constexpr bool kEnableDebugLogs = true;

static std::string bits_to_string(const VecInt& bits, size_t n = 64)
{
    std::string s;
    size_t m = std::min(n, bits.size());
    for (size_t i = 0; i < m; ++i)
        s.push_back(bits[i] ? '1' : '0');
    return s;
}

static double compute_ber(const VecInt& tx, const VecInt& rx, size_t& bit_errors)
{
    size_t n = std::min(tx.size(), rx.size());
    bit_errors = 0;

    for (size_t i = 0; i < n; ++i)
        if ((tx[i] & 1) != (rx[i] & 1))
            bit_errors++;

    return n ? (double)bit_errors / (double)n : 0.0;
}

static double wrap_phase_pm_pi(double x)
{
    while (x > M_PI) x -= 2.0 * M_PI;
    while (x < -M_PI) x += 2.0 * M_PI;
    return x;
}

static int detect_ofdm_pss_refined(
    const VecComplex& rx,
    const VecComplex& pss,
    int refine_radius = 8)
{
    if (rx.size() < pss.size()) return -1;

    double best = -1.0;
    int coarse_pos = -1;
    for (size_t d = 0; d + pss.size() <= rx.size(); ++d) {
        Complex acc(0.0, 0.0);
        for (size_t k = 0; k < pss.size(); ++k) {
            acc += std::conj(pss[k]) * rx[d + k];
        }
        const double v = std::norm(acc);
        if (v > best) {
            best = v;
            coarse_pos = static_cast<int>(d);
        }
    }

    if (coarse_pos < 0) return -1;

    const int start = std::max(0, coarse_pos - refine_radius);
    const int end = std::min(
        static_cast<int>(rx.size() - pss.size()),
        coarse_pos + refine_radius);

    double refine_best = -1.0;
    int refine_pos = coarse_pos;
    for (int d = start; d <= end; ++d) {
        Complex acc(0.0, 0.0);
        double er = 0.0;
        double ep = 0.0;
        for (size_t k = 0; k < pss.size(); ++k) {
            acc += std::conj(pss[k]) * rx[static_cast<size_t>(d) + k];
            er += std::norm(rx[static_cast<size_t>(d) + k]);
            ep += std::norm(pss[k]);
        }
        if (er <= 1e-12 || ep <= 1e-12) continue;
        const double metric = std::norm(acc) / (er * ep);
        if (metric > refine_best) {
            refine_best = metric;
            refine_pos = d;
        }
    }

    return refine_pos;
}

static int detect_ofdm_pss_in_window(
    const VecComplex& rx,
    const VecComplex& pss,
    int center,
    int radius,
    double* out_metric = nullptr)
{
    if (out_metric) *out_metric = -1.0;
    if (rx.size() < pss.size()) return -1;

    const int max_start = static_cast<int>(rx.size() - pss.size());
    const int start = std::max(0, center - radius);
    const int end = std::min(max_start, center + radius);
    if (start > end) return -1;

    double best_metric = -1.0;
    int best_pos = -1;
    for (int d = start; d <= end; ++d) {
        Complex acc(0.0, 0.0);
        double er = 0.0;
        double ep = 0.0;
        for (size_t k = 0; k < pss.size(); ++k) {
            acc += std::conj(pss[k]) * rx[static_cast<size_t>(d) + k];
            er += std::norm(rx[static_cast<size_t>(d) + k]);
            ep += std::norm(pss[k]);
        }
        if (er <= 1e-12 || ep <= 1e-12) continue;
        const double metric = std::norm(acc) / (er * ep);
        if (metric > best_metric) {
            best_metric = metric;
            best_pos = d;
        }
    }

    if (out_metric) *out_metric = best_metric;
    return best_pos;
}

static int find_first_valid_ofdm_pss_in_window(
    const VecComplex& rx,
    const VecComplex& pss,
    int start_center,
    int radius,
    double threshold,
    double* out_metric = nullptr)
{
    if (out_metric) *out_metric = -1.0;
    if (rx.size() < pss.size()) return -1;

    const int max_start = static_cast<int>(rx.size() - pss.size());
    const int start = std::max(0, start_center - radius);
    const int end = std::min(max_start, start_center + radius);
    if (start > end) return -1;

    for (int d = start; d <= end; ++d) {
        Complex acc(0.0, 0.0);
        double er = 0.0;
        double ep = 0.0;
        for (size_t k = 0; k < pss.size(); ++k) {
            acc += std::conj(pss[k]) * rx[static_cast<size_t>(d) + k];
            er += std::norm(rx[static_cast<size_t>(d) + k]);
            ep += std::norm(pss[k]);
        }
        if (er <= 1e-12 || ep <= 1e-12) continue;
        const double metric = std::norm(acc) / (er * ep);
        if (metric >= threshold) {
            if (out_metric) *out_metric = metric;
            return d;
        }
    }

    return -1;
}

static double estimate_ofdm_cfo_from_cp(
    const VecComplex& rx,
    int pss_pos,
    const OFDMConfig& cfg)
{
    const int pss_len = static_cast<int>(OFDMUtils::makePSS(cfg.N_fft).size());
    const int cp_start = pss_pos + pss_len - 1;
    const int data_start = cp_start + cfg.N_cp;
    const int tail_start = data_start + cfg.N_fft - cfg.N_cp;
    const int need_end = data_start + cfg.N_fft;

    if (cp_start < 0 || need_end > static_cast<int>(rx.size())) return 0.0;

    Complex acc(0.0, 0.0);
    for (int n = 0; n < cfg.N_cp; ++n) {
        const Complex& a = rx[static_cast<size_t>(cp_start + n)];
        const Complex& b = rx[static_cast<size_t>(tail_start + n)];
        acc += std::conj(a) * b;
    }

    if (std::abs(acc) <= 1e-12) return 0.0;

    const double phase = std::atan2(acc.imag(), acc.real());
    return phase * cfg.sampleRate() / (2.0 * M_PI * static_cast<double>(cfg.N_fft));
}

static VecComplex apply_frequency_correction(
    const VecComplex& signal,
    double fs,
    double freq_hz)
{
    if (signal.empty()) return signal;
    if (!std::isfinite(fs) || fs <= 0.0) return signal;
    if (!std::isfinite(freq_hz) || std::abs(freq_hz) < 1e-12) return signal;

    VecComplex out(signal.size());
    for (size_t n = 0; n < signal.size(); ++n) {
        const double ang = 2.0 * M_PI * freq_hz * static_cast<double>(n) / fs;
        out[n] = signal[n] * std::conj(Complex(std::cos(ang), std::sin(ang)));
    }
    return out;
}

static void fit_phase_line(
    const std::vector<int>& x,
    const std::vector<double>& y,
    double& a,
    double& b)
{
    a = 0.0;
    b = 0.0;
    if (x.size() != y.size() || x.empty()) return;
    if (x.size() == 1) {
        b = y.front();
        return;
    }

    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    const double n = static_cast<double>(x.size());
    for (size_t i = 0; i < x.size(); ++i) {
        const double xd = static_cast<double>(x[i]);
        sx += xd;
        sy += y[i];
        sxx += xd * xd;
        sxy += xd * y[i];
    }

    const double denom = n * sxx - sx * sx;
    if (std::abs(denom) < 1e-12) {
        b = sy / n;
        return;
    }

    a = (n * sxy - sx * sy) / denom;
    b = (sy - a * sx) / n;
}

static bool extract_ofdm_symbols_from_pss(
    const VecComplex& rx,
    int pss_pos,
    const VecComplex& pss,
    const OFDMConfig& cfg,
    std::vector<VecComplex>& symbols_no_cp)
{
    symbols_no_cp.clear();

    const int pos0 = pss_pos + static_cast<int>(pss.size()) - 1;
    for (int i = 0; i < cfg.Nd; ++i) {
        const int cp_start = pos0 + i * cfg.N_symbol();
        const int data_start = cp_start + cfg.N_cp;
        const int data_end = data_start + cfg.N_fft;

        if (data_start < 0 || data_end > static_cast<int>(rx.size())) {
            symbols_no_cp.clear();
            return false;
        }

        symbols_no_cp.emplace_back(
            rx.begin() + data_start,
            rx.begin() + data_end);
    }

    return true;
}

static double score_ofdm_cfo_candidate(
    const VecComplex& rx,
    const VecComplex& pss,
    const OFDMConfig& cfg,
    double cfo_hz)
{
    const VecComplex rx_corr = apply_frequency_correction(rx, cfg.sampleRate(), cfo_hz);
    const int pss_pos = detect_ofdm_pss_refined(rx_corr, pss, 12);
    if (pss_pos < 0) return -1e300;

    std::vector<VecComplex> symbols_no_cp;
    if (!extract_ofdm_symbols_from_pss(rx_corr, pss_pos, pss, cfg, symbols_no_cp)) {
        return -1e300;
    }

    auto pilotPos = OFDMUtils::pilotPositions(cfg.N_sc, cfg.P_f_inter);
    const int pilotNum = static_cast<int>(pilotPos.size());
    if (pilotNum <= 0) return -1e300;

    std::vector<std::vector<Complex>> pilotSeq(
        pilotNum, std::vector<Complex>(cfg.Nd, Complex(1.0, 0.0)));
    {
        uint32_t s = 1u;
        auto rbit = [&]() {
            s = 1664525u * s + 1013904223u;
            return (s >> 31) & 1u;
        };
        for (int r = 0; r < pilotNum; ++r) {
            for (int c = 0; c < cfg.Nd; ++c) {
                pilotSeq[r][c] = Complex(rbit() ? 1.0 : -1.0, 0.0);
            }
        }
    }

    double score = 0.0;
    int score_terms = 0;

    const int eval_cols = std::min(cfg.Nd, 3);
    for (int col = 0; col < eval_cols; ++col) {
        VecComplex Y = OFDMUtils::fft(symbols_no_cp[col]);
        VecComplex used(cfg.N_sc);
        for (int k = 0; k < cfg.N_sc; ++k) {
            used[k] = Y[1 + k];
        }

        std::vector<double> phase(pilotNum, 0.0);
        double mag_sum = 0.0;
        for (int i = 0; i < pilotNum; ++i) {
            Complex h = (std::abs(pilotSeq[i][col]) > 1e-12)
                ? (used[pilotPos[i]] / pilotSeq[i][col])
                : Complex(1.0, 0.0);
            phase[i] = std::atan2(h.imag(), h.real());
            mag_sum += std::abs(h);
            if (i > 0) {
                phase[i] = phase[i - 1] + wrap_phase_pm_pi(phase[i] - phase[i - 1]);
            }
        }

        double a = 0.0, b = 0.0;
        fit_phase_line(pilotPos, phase, a, b);

        double residual_var = 0.0;
        for (int i = 0; i < pilotNum; ++i) {
            const double pred = a * static_cast<double>(pilotPos[i]) + b;
            const double err = wrap_phase_pm_pi(phase[i] - pred);
            residual_var += err * err;
        }
        residual_var /= static_cast<double>(pilotNum);

        const double mag_avg = mag_sum / static_cast<double>(pilotNum);
        score += mag_avg - 5.0 * residual_var;
        score_terms++;
    }

    if (score_terms <= 0) return -1e300;
    return score / static_cast<double>(score_terms);
}

static double resolve_ofdm_cfo_ambiguity(
    const VecComplex& rx,
    const VecComplex& pss,
    const OFDMConfig& cfg,
    double cfo_alias_hz)
{
    const double step_hz = cfg.sampleRate() / static_cast<double>(cfg.N_fft);
    double best_cfo = cfo_alias_hz;
    double best_score = -1e300;

    for (int k = -2; k <= 2; ++k) {
        const double cand = cfo_alias_hz + static_cast<double>(k) * step_hz;
        const double score = score_ofdm_cfo_candidate(rx, pss, cfg, cand);
        if (score > best_score) {
            best_score = score;
            best_cfo = cand;
        }
    }

    return best_cfo;
}

static std::vector<std::vector<Complex>> make_ofdm_pilot_sequence(const OFDMConfig& cfg)
{
    auto pilotPos = OFDMUtils::pilotPositions(cfg.N_sc, cfg.P_f_inter);
    const int pilotNum = static_cast<int>(pilotPos.size());
    std::vector<std::vector<Complex>> pilotSeq(
        pilotNum, std::vector<Complex>(cfg.Nd, Complex(1.0, 0.0)));

    uint32_t s = 1u;
    auto rbit = [&]() {
        s = 1664525u * s + 1013904223u;
        return (s >> 31) & 1u;
    };

    for (int r = 0; r < pilotNum; ++r) {
        for (int c = 0; c < cfg.Nd; ++c) {
            pilotSeq[r][c] = Complex(rbit() ? 1.0 : -1.0, 0.0);
        }
    }
    return pilotSeq;
}

static VecComplex build_ofdm_random_frame_signal(
    const OFDMConfig& cfg,
    const VecInt& tx_bits,
    const std::vector<int>& pilotPos,
    const std::vector<int>& dataPos,
    const std::vector<std::vector<Complex>>& pilotSeq,
    VecComplex* out_qam_syms = nullptr)
{
    VecInt codedBits = OFDMUtils::convEncode_171_133(tx_bits);
    VecComplex qamSyms = OFDMUtils::qamMod(codedBits, cfg.M);
    if (out_qam_syms) {
        *out_qam_syms = qamSyms;
    }

    std::vector<std::vector<Complex>> grid(
        cfg.N_sc, std::vector<Complex>(cfg.Nd, Complex(0.0, 0.0)));

    for (size_t r = 0; r < pilotPos.size(); ++r) {
        for (int c = 0; c < cfg.Nd; ++c) {
            grid[pilotPos[r]][c] = pilotSeq[r][c];
        }
    }

    size_t idx = 0;
    for (int c = 0; c < cfg.Nd; ++c) {
        for (size_t r = 0; r < dataPos.size(); ++r) {
            grid[dataPos[r]][c] = (idx < qamSyms.size()) ? qamSyms[idx++] : Complex(0.0, 0.0);
        }
    }

    VecComplex tx_data;
    for (int col = 0; col < cfg.Nd; ++col) {
        VecComplex X(cfg.N_fft, Complex(0.0, 0.0));
        for (int k = 0; k < cfg.N_sc; ++k) {
            X[1 + k] = grid[k][col];
        }
        VecComplex x = OFDMUtils::ifft(X);
        for (int i = cfg.N_fft - cfg.N_cp; i < cfg.N_fft; ++i) {
            tx_data.push_back(x[i]);
        }
        tx_data.insert(tx_data.end(), x.begin(), x.end());
    }

    VecComplex tx_sig;
    tx_sig.insert(tx_sig.end(), cfg.N_zeros, Complex(0.0, 0.0));
    VecComplex pss = OFDMUtils::makePSS(cfg.N_fft);
    tx_sig.insert(tx_sig.end(), pss.begin(), pss.end());
    tx_sig.insert(tx_sig.end(), tx_data.begin(), tx_data.end());
    return tx_sig;
}

static bool decode_ofdm_random_frame_signal(
    const VecComplex& rx_sig,
    const OFDMConfig& cfg,
    const std::vector<int>& pilotPos,
    const std::vector<int>& dataPos,
    const std::vector<std::vector<Complex>>& pilotSeq,
    int initial_pss_pos,
    int pss_refine_radius,
    bool use_cfo_ambiguity_resolve,
    VecInt& rx_bits,
    VecComplex* out_eqSyms = nullptr,
    double* out_cfo_hat_hz = nullptr,
    int* out_pss_pos = nullptr)
{
    rx_bits.clear();
    if (out_eqSyms) out_eqSyms->clear();
    if (out_cfo_hat_hz) *out_cfo_hat_hz = 0.0;
    if (out_pss_pos) *out_pss_pos = -1;

    const VecComplex pss = OFDMUtils::makePSS(cfg.N_fft);
    int pssPos = initial_pss_pos;
    if (pssPos < 0 || pssPos + static_cast<int>(pss.size()) > static_cast<int>(rx_sig.size())) {
        return false;
    }

    const double cfo_alias_hz = estimate_ofdm_cfo_from_cp(rx_sig, pssPos, cfg);
    const double cfo_hat_hz = use_cfo_ambiguity_resolve
        ? resolve_ofdm_cfo_ambiguity(rx_sig, pss, cfg, cfo_alias_hz)
        : cfo_alias_hz;

    VecComplex rx_work = rx_sig;
    if (std::isfinite(cfo_hat_hz) && std::abs(cfo_hat_hz) > 1e-9) {
        rx_work = apply_frequency_correction(rx_sig, cfg.sampleRate(), cfo_hat_hz);
        pssPos = detect_ofdm_pss_in_window(rx_work, pss, pssPos, pss_refine_radius, nullptr);
        if (pssPos < 0) {
            return false;
        }
    }

    std::vector<VecComplex> symbolsNoCP;
    if (!extract_ofdm_symbols_from_pss(rx_work, pssPos, pss, cfg, symbolsNoCP)) {
        return false;
    }

    VecComplex eqSyms;
    const int pilotNum = static_cast<int>(pilotPos.size());
    for (int col = 0; col < cfg.Nd; ++col) {
        VecComplex Y = OFDMUtils::fft(symbolsNoCP[col]);
        VecComplex used(cfg.N_sc);
        for (int k = 0; k < cfg.N_sc; ++k) {
            used[k] = Y[1 + k];
        }

        VecComplex rxPilot(pilotNum), txPilot(pilotNum);
        for (int i = 0; i < pilotNum; ++i) {
            rxPilot[i] = used[pilotPos[i]];
            txPilot[i] = pilotSeq[i][col];
        }

        std::vector<double> pilot_phase(pilotNum, 0.0);
        for (int i = 0; i < pilotNum; ++i) {
            Complex h = (std::abs(txPilot[i]) > 1e-12)
                ? (rxPilot[i] / txPilot[i])
                : Complex(1.0, 0.0);
            pilot_phase[i] = std::atan2(h.imag(), h.real());
            if (i > 0) {
                pilot_phase[i] = pilot_phase[i - 1] +
                    wrap_phase_pm_pi(pilot_phase[i] - pilot_phase[i - 1]);
            }
        }

        double phase_slope = 0.0;
        double phase_bias = 0.0;
        fit_phase_line(pilotPos, pilot_phase, phase_slope, phase_bias);

        VecComplex used_phase_corrected = used;
        for (int k = 0; k < cfg.N_sc; ++k) {
            const double ph = phase_slope * static_cast<double>(k) + phase_bias;
            used_phase_corrected[k] *= std::conj(Complex(std::cos(ph), std::sin(ph)));
        }

        VecComplex rxPilotCorrected(pilotNum);
        for (int i = 0; i < pilotNum; ++i) {
            rxPilotCorrected[i] = used_phase_corrected[pilotPos[i]];
        }

        auto H = OFDMUtils::linearInterpChannel(pilotPos, rxPilotCorrected, txPilot, dataPos);
        for (size_t i = 0; i < dataPos.size(); ++i) {
            Complex h = H[i];
            if (std::abs(h) < 1e-12) h = Complex(1.0, 0.0);
            eqSyms.push_back(used_phase_corrected[dataPos[i]] / h);
        }
    }

    VecInt rx_coded_bits = OFDMUtils::qamDemod(eqSyms, cfg.M);
    rx_bits = OFDMUtils::viterbiDecodeHard_171_133(rx_coded_bits, cfg.tblen);

    if (out_eqSyms) *out_eqSyms = eqSyms;
    if (out_cfo_hat_hz) *out_cfo_hat_hz = cfo_hat_hz;
    if (out_pss_pos) *out_pss_pos = pssPos;
    return !rx_bits.empty();
}

static bool decode_ofdm_random_frame_signal(
    const VecComplex& rx_sig,
    const OFDMConfig& cfg,
    const std::vector<int>& pilotPos,
    const std::vector<int>& dataPos,
    const std::vector<std::vector<Complex>>& pilotSeq,
    VecInt& rx_bits,
    VecComplex* out_eqSyms = nullptr,
    double* out_cfo_hat_hz = nullptr,
    int* out_pss_pos = nullptr)
{
    const VecComplex pss = OFDMUtils::makePSS(cfg.N_fft);
    int pssPos = detect_ofdm_pss_refined(rx_sig, pss, 12);
    if (pssPos < 0) {
        return false;
    }
    return decode_ofdm_random_frame_signal(
        rx_sig, cfg, pilotPos, dataPos, pilotSeq, pssPos, 12, true,
        rx_bits, out_eqSyms, out_cfo_hat_hz, out_pss_pos);
}

static std::vector<unsigned char> bits_to_bytes(const VecInt& bits)
{
    std::vector<unsigned char> bytes;

    if (bits.empty()) return bytes;

    const size_t nbytes = bits.size() / 8;
    bytes.reserve(nbytes);

    for (size_t i = 0; i < nbytes; ++i)
    {
        unsigned char b = 0;

        for (size_t j = 0; j < 8; ++j)
        {
            const int raw = bits[i * 8 + j];
            const int bit = raw & 1;
            b |= static_cast<unsigned char>(bit << (7 - j));
        }

        bytes.push_back(b);
    }

    return bytes;
}

static unsigned short read_u16_be(const std::vector<unsigned char>& data, size_t pos)
{
    return static_cast<unsigned short>(
        (static_cast<unsigned short>(data[pos]) << 8) |
        static_cast<unsigned short>(data[pos + 1])
        );
}

static unsigned long long read_u64_be(const std::vector<unsigned char>& data, size_t pos)
{
    unsigned long long v = 0;
    for (int i = 0; i < 8; ++i)
    {
        v = (v << 8) | static_cast<unsigned long long>(data[pos + i]);
    }
    return v;
}

static VecComplex transceive_usrp_burst(
    const std::string& device_args,
    double sample_rate,
    double center_freq_hz,
    const VecComplex& tx_burst,
    size_t rx_extra_samps = 0)
{
#ifdef WITH_UHD
    USRPDriver::Config uc;
    uc.device_args = device_args;
    uc.sample_rate = sample_rate;
    uc.center_freq = center_freq_hz;

    USRPDriver usrp(uc);
    usrp.init();
    usrp.start_rx_worker(tx_burst.size() + rx_extra_samps);
    usrp.send_burst(tx_burst);
    usrp.wait_rx_worker();
    return usrp.fetch_rx_buffer();
#else
    (void)device_args;
    (void)sample_rate;
    (void)center_freq_hz;
    (void)tx_burst;
    (void)rx_extra_samps;
    throw std::runtime_error("USRP mode requested, but WITH_UHD is not enabled.");
#endif
}

static bool recover_file_from_bits(
    const VecInt& rx_bits,
    const std::string& output_file_path,
    std::string& recovered_filename
)
{
    std::vector<unsigned char> bytes = bits_to_bytes(rx_bits);

    std::cout << "[FILE][RX] total bits = " << rx_bits.size() << "\n";
    std::cout << "[FILE][RX] total bytes = " << bytes.size() << "\n";

    std::cout << "[FILE][RX] first bytes = ";
    for (size_t i = 0; i < std::min<size_t>(32, bytes.size()); ++i) {
        std::cout << (int)bytes[i] << " ";
    }
    std::cout << "\n";

    if (bytes.size() < 16) {
        std::cout << "[FILE][RX] FAIL: bytes.size() < 16\n";
        return false;
    }

    std::cout << "[FILE][RX] magic chars = "
        << (char)bytes[0]
        << (char)bytes[1]
        << (char)bytes[2]
        << (char)bytes[3] << "\n";

    if (!(bytes[0] == 'U' && bytes[1] == 'A' && bytes[2] == 'V' && bytes[3] == 'F')) {
        std::cout << "[FILE][RX] FAIL: magic mismatch\n";
        return false;
    }

    const unsigned char version = bytes[4];
    const unsigned char file_type = bytes[5];
    const unsigned short filename_len = read_u16_be(bytes, 6);
    const unsigned long long file_size = read_u64_be(bytes, 8);

    std::cout << "[FILE][RX] version = " << (int)version << "\n";
    std::cout << "[FILE][RX] file_type = " << (int)file_type << "\n";
    std::cout << "[FILE][RX] filename_len = " << filename_len << "\n";
    std::cout << "[FILE][RX] file_size = " << file_size << "\n";

    if (version != 1) {
        std::cout << "[FILE][RX] FAIL: version != 1\n";
        return false;
    }

    const size_t header_size = 16ull + static_cast<size_t>(filename_len);

    std::cout << "[FILE][RX] header_size = " << header_size << "\n";

    if (bytes.size() < header_size) {
        std::cout << "[FILE][RX] FAIL: bytes.size() < header_size\n";
        return false;
    }

    if (bytes.size() < header_size + static_cast<size_t>(file_size)) {
        std::cout << "[FILE][RX] FAIL: bytes.size() < header_size + file_size\n";
        return false;
    }

    recovered_filename.assign(
        reinterpret_cast<const char*>(&bytes[16]),
        reinterpret_cast<const char*>(&bytes[16 + filename_len])
    );

    std::cout << "[FILE][RX] recovered filename = " << recovered_filename << "\n";

    std::ofstream ofs(output_file_path, std::ios::binary);
    if (!ofs) {
        std::cout << "[FILE][RX] FAIL: cannot open output path = " << output_file_path << "\n";
        throw std::runtime_error("Cannot open output file for writing: " + output_file_path);
    }

    if (file_size > 0) {
        ofs.write(
            reinterpret_cast<const char*>(&bytes[header_size]),
            static_cast<std::streamsize>(file_size)
        );
    }

    std::cout << "[FILE][RX] file saved OK: " << output_file_path << "\n";
    return true;
}

std::string mode_to_string(RunMode mode)
{
    switch (mode)
    {
    case RunMode::LOOPBACK: return "LOOPBACK";
    case RunMode::AWGN: return "AWGN";
    case RunMode::USRP: return "USRP";
    default: return "UNKNOWN";
    }
}

std::string modulation_to_string(ModulationType m)
{
    switch (m)
    {
    case ModulationType::BPSK: return "BPSK";
    case ModulationType::QPSK: return "QPSK";
    case ModulationType::QAM:  return "QAM";
    case ModulationType::OOK:  return "OOK";
    case ModulationType::FSK:  return "FSK";
    case ModulationType::FM:   return "FM";
    case ModulationType::MSK:  return "MSK";
    default: return "UNKNOWN";
    }
}

ModulationType parse_modulation(const std::string& s)
{
    std::string t = s;
    std::transform(t.begin(), t.end(), t.begin(), ::tolower);

    if (t == "bpsk") return ModulationType::BPSK;
    if (t == "qpsk") return ModulationType::QPSK;
    if (t == "qam")  return ModulationType::QAM;
    if (t == "ook")  return ModulationType::OOK;
    if (t == "fsk")  return ModulationType::FSK;
    if (t == "fm")   return ModulationType::FM;
    if (t == "msk")  return ModulationType::MSK;

    throw std::runtime_error("Unknown modulation");
}

RunMode parse_mode(const std::string& s)
{
    std::string t = s;
    std::transform(t.begin(), t.end(), t.begin(), ::tolower);

    if (t == "loopback") return RunMode::LOOPBACK;
    if (t == "awgn")     return RunMode::AWGN;
    if (t == "usrp")     return RunMode::USRP;

    throw std::runtime_error("Unknown mode");
}

static std::vector<std::complex<double>> compute_dft(
    const std::vector<std::complex<double>>& x)
{
    const size_t N = x.size();
    std::vector<std::complex<double>> X(N);

    for (size_t k = 0; k < N; ++k) {
        std::complex<double> sum = 0.0;
        for (size_t n = 0; n < N; ++n) {
            double angle = -2.0 * M_PI * static_cast<double>(k) * static_cast<double>(n) / static_cast<double>(N);
            sum += x[n] * std::complex<double>(std::cos(angle), std::sin(angle));
        }
        X[k] = sum;
    }
    return X;
}

static size_t get_center_start(size_t total_len, size_t want_len)
{
    if (want_len >= total_len) {
        return 0;
    }
    return (total_len - want_len) / 2;
}

static VecComplex extract_center_segment(const VecComplex& sig, size_t want_len)
{
    if (sig.empty()) {
        return {};
    }

    const size_t takeN = std::min(want_len, sig.size());
    const size_t start = get_center_start(sig.size(), takeN);

    return VecComplex(sig.begin() + static_cast<long long>(start),
        sig.begin() + static_cast<long long>(start + takeN));
}

static void build_waveform_from_signal(
    const VecComplex& sig_mid,
    ModulationType modulation,
    std::vector<double>& out_waveform,
    WaveformType& out_type)
{
    out_waveform.clear();

    if (sig_mid.empty()) {
        out_type = WaveformType::REAL;
        return;
    }

    bool useEnvelope = 0;
    //bool useEnvelope = (modulation == ModulationType::FM || modulation == ModulationType::MSK);
    out_type = useEnvelope ? WaveformType::ENVELOPE : WaveformType::REAL;

    out_waveform.reserve(sig_mid.size());

    if (useEnvelope)
    {
        std::vector<double> env;
        env.reserve(sig_mid.size());

        for (size_t i = 0; i < sig_mid.size(); ++i) {
            env.push_back(std::abs(sig_mid[i]));
        }

        const size_t win = 64;

        for (size_t i = 0; i < env.size(); ++i) {
            const size_t l = (i > win / 2) ? (i - win / 2) : 0;
            const size_t r = std::min(env.size(), i + win / 2 + 1);

            double sum = 0.0;
            for (size_t k = l; k < r; ++k) {
                sum += env[k];
            }

            out_waveform.push_back(sum / static_cast<double>(r - l));
        }
    }
    else
    {
        for (size_t i = 0; i < sig_mid.size(); ++i) {
            out_waveform.push_back(sig_mid[i].real());
        }
    }
}

static void build_spectrum_from_signal(
    const VecComplex& sig_mid,
    double fs,
    std::vector<double>& out_freq,
    std::vector<double>& out_mag)
{
    out_freq.clear();
    out_mag.clear();

    if (sig_mid.empty() || fs <= 0.0) {
        return;
    }

    const size_t N = sig_mid.size();
    if (N < 2) return;

    std::vector<std::complex<double>> x(N);

    for (size_t n = 0; n < N; ++n) {
        double w = 0.5 - 0.5 * std::cos(2.0 * M_PI * n / (N - 1));
        x[n] = sig_mid[n] * w;
    }

    std::vector<std::complex<double>> X(N);

    for (size_t k = 0; k < N; ++k) {
        std::complex<double> sum = 0.0;
        for (size_t n = 0; n < N; ++n) {
            double angle = -2.0 * M_PI * k * n / N;
            sum += x[n] * std::complex<double>(std::cos(angle), std::sin(angle));
        }
        X[k] = sum;
    }

    out_freq.reserve(N);
    out_mag.reserve(N);

    for (size_t k = 0; k < N; ++k) {
        size_t idx = (k + N / 2) % N;
        double f = (static_cast<double>(k) - static_cast<double>(N) / 2.0)
            * fs / static_cast<double>(N);

        double mag = std::abs(X[idx]) / static_cast<double>(N);
        double mag_db = 20.0 * std::log10(mag + 1e-12);

        out_freq.push_back(f);
        out_mag.push_back(mag_db);
    }
}

static void build_spectrogram(
    const VecComplex& sig_mid,
    double fs,
    TestResult& tr)
{
    tr.spectrogram_data.clear();
    tr.spectrogram_width = 0;
    tr.spectrogram_height = 0;
    tr.spectrogram_time_span = 0.0;
    tr.spectrogram_freq_min = -fs / 2.0;
    tr.spectrogram_freq_max = fs / 2.0;

    if (sig_mid.empty() || fs <= 0.0) {
        return;
    }

    const int fftSize = 256;
    const int hopSize = 64;
    const int windowSize = 256;

    if (sig_mid.size() < static_cast<size_t>(windowSize)) {
        return;
    }

    const int frameCount =
        1 + static_cast<int>((sig_mid.size() - static_cast<size_t>(windowSize)) / static_cast<size_t>(hopSize));
    const int freqBins = fftSize;

    tr.spectrogram_width = frameCount;
    tr.spectrogram_height = freqBins;
    tr.spectrogram_time_span = static_cast<double>(sig_mid.size()) / fs;
    tr.spectrogram_freq_min = -fs / 2.0;
    tr.spectrogram_freq_max = fs / 2.0;
    tr.spectrogram_data.assign(static_cast<size_t>(frameCount * freqBins), -120.0);

    double globalMaxDb = -1e100;

    for (int frame = 0; frame < frameCount; ++frame)
    {
        const int start = frame * hopSize;

        std::vector<std::complex<double>> x(fftSize, 0.0);

        for (int n = 0; n < windowSize; ++n) {
            double w = 0.5 - 0.5 * std::cos(2.0 * M_PI * n / (windowSize - 1));
            x[n] = sig_mid[static_cast<size_t>(start + n)] * w;
        }

        auto X = compute_dft(x);

        for (int k = 0; k < freqBins; ++k)
        {
            const int idx = (k + fftSize / 2) % fftSize;
            const double mag = std::abs(X[static_cast<size_t>(idx)]) / static_cast<double>(fftSize);
            const double db = 20.0 * std::log10(mag + 1e-12);

            tr.spectrogram_data[static_cast<size_t>(frame * freqBins + k)] = db;
            if (db > globalMaxDb) {
                globalMaxDb = db;
            }
        }
    }

    const double floorDb = globalMaxDb - 60.0;

    for (double& v : tr.spectrogram_data) {
        if (v < floorDb) v = floorDb;
        if (v > globalMaxDb) v = globalMaxDb;

        double norm = (v - floorDb) / (globalMaxDb - floorDb + 1e-12);
        v = std::clamp(norm, 0.0, 1.0);
    }
}

static void build_constellation_result(
    const VecComplex& pts,
    TestResult& tr)
{
    tr.constellation_i.clear();
    tr.constellation_q.clear();

    const size_t N = std::min<size_t>(pts.size(), 600);

    tr.constellation_i.reserve(N);
    tr.constellation_q.reserve(N);

    for (size_t i = 0; i < N; ++i) {
        tr.constellation_i.push_back(pts[i].real());
        tr.constellation_q.push_back(pts[i].imag());
    }
}

TestResult run_one_test(
    RunMode mode,
    double awgn_snr_db,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern
)
{
    std::ostringstream log;

    TransmitterConfig cfg;

    cfg.function =
        (modulation == ModulationType::MSK) ?
        FunctionType::RemoteControl :
        FunctionType::Telemetry;

    cfg.modulation = modulation;
    cfg.n = 10;
    cfg.frame_bit = 75;
    cfg.samp = 8;
    cfg.zp_sym = 33;
    cfg.Rb = info_rate_bps;
    cfg.hop_pattern = hop_pattern;
    cfg.connect = (mode == RunMode::USRP);

    cfg.source_mode = SourceMode::RandomBits;
    cfg.input_file_path.clear();
    cfg.file_bits.clear();
    cfg.file_bit_offset = 0;

    Transmitter tx(cfg);

    VecComplex one_frame_sig = tx.generateTransmitSignal();
    VecInt one_frame_bits = tx.getLastSourceBits();

    cfg.fs = tx.getFS();

    Receiver rx(cfg);

    log << "[MODE] " << mode_to_string(mode) << "\n";
    log << "[MOD] " << modulation_to_string(modulation) << "\n";
    log << "[SRC] RANDOM_BITS\n";
    log << "[CENTER FREQ] " << center_freq_hz << " Hz\n";
    log << "[INFO RATE] " << info_rate_bps << " bps\n";
    log << "[HOP PATTERN] " << hop_pattern << "\n";

    VecComplex tx_burst;

    for (int i = 0; i < tx_repeat_frames; ++i) {
        tx_burst.insert(tx_burst.end(), one_frame_sig.begin(), one_frame_sig.end());
    }

    VecComplex rx_sig;

    if (mode == RunMode::LOOPBACK)
    {
        rx_sig = tx_burst;
    }
    else if (mode == RunMode::AWGN)
    {
        rx_sig = Channel::awgn(tx_burst, awgn_snr_db);
    }
    else if (mode == RunMode::USRP)
    {
#ifdef WITH_UHD
        USRPDriver::Config uc;

        uc.device_args = "type=b200";
        uc.sample_rate = tx.getFS();
        uc.center_freq = center_freq_hz;

        USRPDriver usrp(uc);
        usrp.init();

        usrp.start_rx_worker(tx_burst.size());
        usrp.send_burst(tx_burst);
        usrp.wait_rx_worker();
        rx_sig = usrp.fetch_rx_buffer();
#else
        throw std::runtime_error("USRP mode requested, but WITH_UHD is not enabled.");
#endif
    }
    else
    {
        throw std::runtime_error("Unknown run mode.");
    }

    if (rx_sig.empty()) {
        throw std::runtime_error("rx_sig is empty before receiver processing.");
    }

    VecInt rx_bits = rx.receive(rx_sig);

    TestResult tr;
    build_constellation_result(rx.getLastConstellationPoints(), tr);

    const size_t bits_per_frame = one_frame_bits.size();
    const size_t decoded_frames = bits_per_frame > 0 ? (rx_bits.size() / bits_per_frame) : 0;

    size_t total_compared_bits = 0;
    size_t total_bit_errors = 0;

    for (size_t i = 0; i < decoded_frames; ++i)
    {
        VecInt rx_frame(
            rx_bits.begin() + static_cast<long long>(i * bits_per_frame),
            rx_bits.begin() + static_cast<long long>((i + 1) * bits_per_frame)
        );

        size_t frame_errors = 0;
        compute_ber(one_frame_bits, rx_frame, frame_errors);

        total_bit_errors += frame_errors;
        total_compared_bits += bits_per_frame;
    }

    tr.total_bit_errors = total_bit_errors;
    tr.total_compared_bits = total_compared_bits;
    tr.decoded_frames = decoded_frames;
    tr.total_ber =
        total_compared_bits ?
        static_cast<double>(total_bit_errors) / static_cast<double>(total_compared_bits) :
        0.0;

    {
        const VecComplex& tx_sig_full = tx.getLastPureModulatedSignal();

        if (tx_sig_full.empty()) {
            throw std::runtime_error("Pure modulated signal is empty");
        }

        const VecComplex tx_sig_wave_mid = extract_center_segment(tx_sig_full, 4000);
        const VecComplex tx_sig_spec_mid = extract_center_segment(tx_sig_full, 1024);
        const VecComplex tx_sig_tf_mid = extract_center_segment(tx_sig_full, 4096);

        build_waveform_from_signal(
            tx_sig_wave_mid,
            modulation,
            tr.waveform,
            tr.waveform_type
        );

        build_spectrum_from_signal(
            tx_sig_spec_mid,
            cfg.fs,
            tr.spectrum_freq,
            tr.spectrum_mag
        );

        build_spectrogram(tx_sig_tf_mid, cfg.fs, tr);

        log << "[TX MID SEGMENT] waveform = " << tx_sig_wave_mid.size()
            << ", spectrum = " << tx_sig_spec_mid.size()
            << ", spectrogram = " << tx_sig_tf_mid.size() << "\n";
        log << "[SPECTROGRAM] width = " << tr.spectrogram_width
            << ", height = " << tr.spectrogram_height << "\n";
    }

    {
        const VecComplex rx_sig_wave_mid = extract_center_segment(rx_sig, 4000);
        const VecComplex rx_sig_spec_mid = extract_center_segment(rx_sig, 1024);

        build_waveform_from_signal(
            rx_sig_wave_mid,
            modulation,
            tr.rx_waveform,
            tr.rx_waveform_type
        );

        build_spectrum_from_signal(
            rx_sig_spec_mid,
            cfg.fs,
            tr.rx_spectrum_freq,
            tr.rx_spectrum_mag
        );

        log << "[RX MID SEGMENT] waveform = " << rx_sig_wave_mid.size()
            << ", spectrum = " << rx_sig_spec_mid.size() << "\n";
    }

    log << "[CONSTELLATION POINTS] " << tr.constellation_i.size() << "\n";
    log << "[RX BITS] " << rx_bits.size() << "\n";
    log << "[DECODED FRAMES] " << decoded_frames << "\n";
    log << "BER = " << tr.total_ber << "\n";

    tr.log_text = log.str();
    return tr;
}

TestResult run_file_transfer_test(
    RunMode mode,
    double awgn_snr_db,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern,
    const std::string& input_file_path,
    const std::string& output_file_path
)
{
    std::ostringstream log;

    if (input_file_path.empty()) {
        throw std::runtime_error("input_file_path is empty");
    }

    if (output_file_path.empty()) {
        throw std::runtime_error("output_file_path is empty");
    }

    TransmitterConfig cfg;

    cfg.function =
        (modulation == ModulationType::MSK) ?
        FunctionType::RemoteControl :
        FunctionType::Telemetry;

    cfg.modulation = modulation;
    cfg.n = 10;
    cfg.frame_bit = 75;
    cfg.samp = 8;
    cfg.zp_sym = 33;
    cfg.Rb = info_rate_bps;
    cfg.hop_pattern = hop_pattern;
    cfg.connect = (mode == RunMode::USRP);

    cfg.source_mode = SourceMode::FileBits;
    cfg.input_file_path = input_file_path;
    cfg.file_bits.clear();
    cfg.file_bit_offset = 0;

    Transmitter tx(cfg);

    VecComplex first_frame_sig = tx.generateTransmitSignal();
    VecInt first_frame_bits = tx.getLastSourceBits();

    cfg.fs = tx.getFS();

    const TransmitterConfig& tx_cfg = tx.getConfig();
    const size_t total_file_bits = tx_cfg.file_bits.size();
    const size_t bits_per_frame = static_cast<size_t>(cfg.frame_bit * cfg.n);
    const size_t total_frames =
        bits_per_frame > 0 ?
        (total_file_bits + bits_per_frame - 1) / bits_per_frame :
        0;

    Receiver rx(cfg);

    log << "[MODE] " << mode_to_string(mode) << "\n";
    log << "[MOD] " << modulation_to_string(modulation) << "\n";
    log << "[SRC] FILE_BITS\n";
    log << "[CENTER FREQ] " << center_freq_hz << " Hz\n";
    log << "[INFO RATE] " << info_rate_bps << " bps\n";
    log << "[HOP PATTERN] " << hop_pattern << "\n";
    log << "[INPUT FILE] " << input_file_path << "\n";
    log << "[OUTPUT FILE] " << output_file_path << "\n";
    log << "[PACKED FILE BITS] " << total_file_bits << "\n";
    log << "[BITS PER FRAME] " << bits_per_frame << "\n";
    log << "[TOTAL FRAMES] " << total_frames << "\n";

    VecComplex tx_burst;
    VecInt tx_all_bits;

    tx_burst.insert(tx_burst.end(), first_frame_sig.begin(), first_frame_sig.end());
    tx_all_bits.insert(tx_all_bits.end(), first_frame_bits.begin(), first_frame_bits.end());

    for (size_t i = 1; i < total_frames; ++i)
    {
        VecComplex frame_sig = tx.generateTransmitSignal();
        VecInt frame_bits = tx.getLastSourceBits();

        tx_burst.insert(tx_burst.end(), frame_sig.begin(), frame_sig.end());
        tx_all_bits.insert(tx_all_bits.end(), frame_bits.begin(), frame_bits.end());
    }

    VecComplex rx_sig;

    if (mode == RunMode::LOOPBACK)
    {
        rx_sig = tx_burst;
    }
    else if (mode == RunMode::AWGN)
    {
        rx_sig = Channel::awgn(tx_burst, awgn_snr_db);
    }
    else if (mode == RunMode::USRP)
    {
#ifdef WITH_UHD
        USRPDriver::Config uc;

        uc.device_args = "type=b200";
        uc.sample_rate = tx.getFS();
        uc.center_freq = center_freq_hz;

        USRPDriver usrp(uc);
        usrp.init();

        usrp.start_rx_worker(tx_burst.size());
        usrp.send_burst(tx_burst);
        usrp.wait_rx_worker();
        rx_sig = usrp.fetch_rx_buffer();
#else
        throw std::runtime_error("USRP mode requested, but WITH_UHD is not enabled.");
#endif
    }
    else
    {
        throw std::runtime_error("Unknown run mode.");
    }

    if (rx_sig.empty()) {
        throw std::runtime_error("rx_sig is empty before receiver processing.");
    }

    VecInt rx_bits = rx.receive(rx_sig);

    TestResult tr;
    build_constellation_result(rx.getLastConstellationPoints(), tr);

    if (kEnableDebugLogs) {
        std::cout << "[DBG][TX bits first 80] ";
        for (size_t i = 0; i < std::min<size_t>(80, tx_all_bits.size()); ++i) {
            std::cout << tx_all_bits[i];
        }
        std::cout << "\n";

        std::cout << "[DBG][RX bits first 80] ";
        for (size_t i = 0; i < std::min<size_t>(80, rx_bits.size()); ++i) {
            std::cout << rx_bits[i];
        }
        std::cout << "\n";

        std::cout << "[DBG][BIT ERR IDX < 80] ";
        for (size_t i = 0; i < std::min<size_t>(80, std::min(tx_all_bits.size(), rx_bits.size())); ++i) {
            if (tx_all_bits[i] != rx_bits[i]) {
                std::cout << i << " ";
            }
        }
        std::cout << "\n";
    }

    size_t bit_errors = 0;
    size_t total_compared_bits = std::min(tx_all_bits.size(), rx_bits.size());
    double ber = compute_ber(tx_all_bits, rx_bits, bit_errors);

    tr.total_compared_bits = total_compared_bits;
    tr.total_bit_errors = bit_errors;
    tr.decoded_frames = bits_per_frame ? (rx_bits.size() / bits_per_frame) : 0;
    tr.total_ber = ber;

    std::string recovered_filename;
    std::vector<unsigned char> tx_bytes_dbg = bits_to_bytes(tx_all_bits);
    std::vector<unsigned char> rx_bytes_dbg = bits_to_bytes(rx_bits);

    if (kEnableDebugLogs) {
        std::cout << "[DBG][TX bytes from tx_all_bits] ";
        for (size_t i = 0; i < std::min<size_t>(32, tx_bytes_dbg.size()); ++i) {
            std::cout << static_cast<int>(tx_bytes_dbg[i]) << " ";
        }
        std::cout << "\n";

        std::cout << "[DBG][RX bytes from rx_bits] ";
        for (size_t i = 0; i < std::min<size_t>(32, rx_bytes_dbg.size()); ++i) {
            std::cout << static_cast<int>(rx_bytes_dbg[i]) << " ";
        }
        std::cout << "\n";
    }

    bool save_ok = recover_file_from_bits(rx_bits, output_file_path, recovered_filename);

    tr.file_saved = save_ok;
    tr.saved_file_path = save_ok ? output_file_path : "";

    log << "[CONSTELLATION POINTS] " << tr.constellation_i.size() << "\n";
    log << "[RX BITS] " << rx_bits.size() << "\n";
    log << "[DECODED FRAMES] " << tr.decoded_frames << "\n";
    log << "BER = " << tr.total_ber << "\n";
    log << "FILE RECOVERED = " << (save_ok ? "YES" : "NO") << "\n";

    if (save_ok) {
        log << "RECOVERED ORIGINAL NAME = " << recovered_filename << "\n";
        log << "SAVED TO = " << output_file_path << "\n";
    }

    {
        const VecComplex& tx_sig_full = tx.getLastPureModulatedSignal();

        if (!tx_sig_full.empty())
        {
            const VecComplex tx_sig_wave_mid = extract_center_segment(tx_sig_full, 4000);
            const VecComplex tx_sig_spec_mid = extract_center_segment(tx_sig_full, 1024);
            const VecComplex tx_sig_tf_mid = extract_center_segment(tx_sig_full, 4096);

            build_waveform_from_signal(
                tx_sig_wave_mid,
                modulation,
                tr.waveform,
                tr.waveform_type
            );

            build_spectrum_from_signal(
                tx_sig_spec_mid,
                cfg.fs,
                tr.spectrum_freq,
                tr.spectrum_mag
            );

            build_spectrogram(tx_sig_tf_mid, cfg.fs, tr);
        }
    }

    {
        const VecComplex rx_sig_wave_mid = extract_center_segment(rx_sig, 4000);
        const VecComplex rx_sig_spec_mid = extract_center_segment(rx_sig, 1024);

        build_waveform_from_signal(
            rx_sig_wave_mid,
            modulation,
            tr.rx_waveform,
            tr.rx_waveform_type
        );

        build_spectrum_from_signal(
            rx_sig_spec_mid,
            cfg.fs,
            tr.rx_spectrum_freq,
            tr.rx_spectrum_mag
        );
    }

    tr.log_text = log.str();
    return tr;
}

SweepResult run_awgn_sweep(
    double snr_start,
    double snr_end,
    double snr_step,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation
)
{
    SweepResult sr;
    std::ostringstream log;

    if (snr_step <= 0.0) {
        throw std::runtime_error("snr_step must be > 0");
    }

    if (snr_start > snr_end) {
        throw std::runtime_error("snr_start must be <= snr_end");
    }

    log << "[SWEEP] Mode = AWGN\n";
    log << "[SWEEP] Source = RANDOM_BITS\n";
    log << "[SWEEP] Modulation = " << modulation_to_string(modulation) << "\n";
    log << "[SWEEP] Range = " << snr_start << " dB -> "
        << snr_end << " dB, step = " << snr_step << " dB\n";

    for (double snr = snr_start; snr <= snr_end + 1e-9; snr += snr_step)
    {
        TestResult tr = run_one_test(
            RunMode::AWGN,
            snr,
            tx_repeat_frames,
            center_freq_hz,
            modulation,
            50000.0,
            1
        );

        SweepPoint pt;
        pt.snr_db = snr;
        pt.ber = tr.total_ber;
        pt.decoded_frames = tr.decoded_frames;
        sr.points.push_back(pt);

        log << "SNR = " << snr
            << " dB, BER = " << tr.total_ber
            << ", decoded_frames = " << tr.decoded_frames
            << "\n";
    }

    sr.log_text = log.str();
    return sr;
}


#include "ofdm_link.h"
#include <random>

TestResult run_ofdm_random_bit_test(
    RunMode mode,
    double awgn_snr_db,
    double center_freq_hz,
    const std::string& usrp_device_args,
    const ChannelConfig& ch_cfg)
{
    std::ostringstream log;

    // ===== OFDM 锟斤拷锟斤拷 =====
    OFDMConfig cfg;
    cfg.N_fft = 256;
    cfg.N_cp = 32;
    cfg.N_sc = 128;
    cfg.Nd = 10;
    cfg.N_frm = 1;
    cfg.N_zeros = 256;
    cfg.P_f_inter = 6;
    cfg.M = 4;            // 4=QPSK, 16=16QAM
    cfg.L = 7;
    cfg.tblen = 32;
    cfg.delta_f = 15e3;

    constexpr int kNumFrames = 10;
    constexpr bool kEnableOfdmUsrpBurstMode = false;

    TestResult tr;

    // ===== 每帧锟缴筹拷锟截碉拷原始 bit 锟斤拷 =====
    auto dataPos = OFDMUtils::dataPositions(cfg.N_sc, cfg.P_f_inter);
    const int dataRow = static_cast<int>(dataPos.size());
    const int codedBitsPerFrame = dataRow * cfg.Nd * static_cast<int>(std::log2(cfg.M));
    const int uncodedBitsPerFrame = codedBitsPerFrame / 2; // 1/2 锟斤拷锟斤拷锟?

    std::mt19937 rng(1);
    std::uniform_int_distribution<int> dist01(0, 1);
    auto pilotPos = OFDMUtils::pilotPositions(cfg.N_sc, cfg.P_f_inter);
    auto dataPos2 = OFDMUtils::dataPositions(cfg.N_sc, cfg.P_f_inter);
    std::vector<std::vector<Complex>> pilotSeqRef = make_ofdm_pilot_sequence(cfg);

    size_t total_bit_errors = 0;
    size_t total_compared_bits = 0;

    // 只锟斤拷锟斤拷锟斤拷锟揭恢★拷锟酵硷拷锟斤拷锟斤拷荩锟斤拷锟斤拷锟?200 帧全锟斤拷锟斤拷去太锟斤拷
    VecComplex last_eqSyms;
    VecComplex last_tx_sig;
    VecComplex last_rx_sig;

    if (mode == RunMode::USRP && kEnableOfdmUsrpBurstMode)
    {
        const VecComplex pss = OFDMUtils::makePSS(cfg.N_fft);
        const size_t ofdm_frame_span =
            static_cast<size_t>(cfg.N_zeros) +
            pss.size() +
            static_cast<size_t>(cfg.Nd) * static_cast<size_t>(cfg.N_symbol());

        std::vector<VecInt> tx_bits_frames;
        std::vector<VecComplex> tx_sig_frames;
        tx_bits_frames.reserve(kNumFrames);
        tx_sig_frames.reserve(kNumFrames);

        for (int frm = 0; frm < kNumFrames; ++frm)
        {
            VecInt tx_bits;
            tx_bits.reserve(uncodedBitsPerFrame);
            for (int i = 0; i < uncodedBitsPerFrame; ++i) {
                tx_bits.push_back(dist01(rng));
            }
            VecComplex tx_sig = build_ofdm_random_frame_signal(
                cfg, tx_bits, pilotPos, dataPos2, pilotSeqRef);

            tx_bits_frames.push_back(std::move(tx_bits));
            tx_sig_frames.push_back(std::move(tx_sig));
        }

        const size_t burst_guard_samps = 2 * ofdm_frame_span;
        VecComplex tx_burst;
        tx_burst.reserve(
            burst_guard_samps +
            static_cast<size_t>(kNumFrames) * ofdm_frame_span +
            burst_guard_samps);
        tx_burst.insert(tx_burst.end(), burst_guard_samps, Complex(0.0, 0.0));
        for (const auto& frame_sig : tx_sig_frames) {
            tx_burst.insert(tx_burst.end(), frame_sig.begin(), frame_sig.end());
        }
        tx_burst.insert(tx_burst.end(), burst_guard_samps, Complex(0.0, 0.0));

        VecComplex rx_burst = transceive_usrp_burst(
            usrp_device_args,
            cfg.sampleRate(),
            center_freq_hz,
            tx_burst,
            2 * ofdm_frame_span);

        double first_metric = -1.0;
        int first_pss = find_first_valid_ofdm_pss_in_window(
            rx_burst,
            pss,
            static_cast<int>(burst_guard_samps + cfg.N_zeros),
            static_cast<int>(2 * ofdm_frame_span),
            0.70,
            &first_metric);
        if (first_pss < 0) {
            first_pss = detect_ofdm_pss_refined(rx_burst, pss, 12);
        }
        if (first_pss < 0) {
            throw std::runtime_error("OFDM PSS detect failed in USRP burst.");
        }

        log << "[USRP OFDM] one-shot burst mode\n";
        log << "[USRP OFDM] tx_burst_samples=" << tx_burst.size() << "\n";
        log << "[USRP OFDM] rx_burst_samples=" << rx_burst.size() << "\n";
        log << "[USRP OFDM] first_pss=" << first_pss << "\n";
        log << "[USRP OFDM] first_metric=" << first_metric << "\n";

        for (int frm = 0; frm < kNumFrames; ++frm)
        {
            const int expected_pss = first_pss + frm * static_cast<int>(ofdm_frame_span);
            const int search_radius = std::max(64, cfg.N_cp * 4);
            const int pssPos = detect_ofdm_pss_in_window(
                rx_burst, pss, expected_pss, search_radius, nullptr);

            if (pssPos < 0) {
                throw std::runtime_error("OFDM local PSS detect failed in USRP burst.");
            }

            // Give the shared single-frame decoder some slack around the nominal frame
            // so refined PSS/CFO steps do not fail due to a too-tight crop.
            const int frame_pad = std::max(64, cfg.N_cp * 4);
            const int frame_base_nominal = pssPos - cfg.N_zeros;
            const int frame_base = std::max(0, frame_base_nominal - frame_pad);
            const int frame_end = std::min(
                static_cast<int>(rx_burst.size()),
                frame_base_nominal + static_cast<int>(ofdm_frame_span) + frame_pad);
            if (frame_base >= frame_end) {
                throw std::runtime_error("OFDM frame window out of range in USRP burst.");
            }
            VecComplex rx_frame(
                rx_burst.begin() + static_cast<std::ptrdiff_t>(frame_base),
                rx_burst.begin() + static_cast<std::ptrdiff_t>(frame_end));

            VecComplex eqSyms;
            VecInt rx_bits;
            double cfo_hat_hz = 0.0;
            int refined_pss = -1;
            const int local_pss = pssPos - frame_base;
            if (!decode_ofdm_random_frame_signal(
                rx_frame, cfg, pilotPos, dataPos2, pilotSeqRef, local_pss, search_radius, false,
                rx_bits, &eqSyms, &cfo_hat_hz, &refined_pss)) {
                log << "[USRP OFDM] frame_decode_failed"
                    << " frm=" << frm
                    << " expected_pss=" << expected_pss
                    << " pss=" << pssPos
                    << " local_pss=" << local_pss
                    << " frame_base_nominal=" << frame_base_nominal
                    << " frame_base=" << frame_base
                    << " frame_end=" << frame_end
                    << " frame_len=" << (frame_end - frame_base)
                    << "\n";
                throw std::runtime_error("OFDM frame decode failed in USRP burst.");
            }
            if (rx_bits.size() > tx_bits_frames[frm].size()) {
                rx_bits.resize(tx_bits_frames[frm].size());
            }

            size_t bit_errors = 0;
            double ber = compute_ber(tx_bits_frames[frm], rx_bits, bit_errors);
            total_bit_errors += bit_errors;
            total_compared_bits += std::min(tx_bits_frames[frm].size(), rx_bits.size());

            if (frm == kNumFrames - 1) {
                last_eqSyms = eqSyms;
                last_tx_sig = tx_sig_frames[frm];
                last_rx_sig = rx_frame;
            }

            log << "[Frame " << (frm + 1) << "/" << kNumFrames << "] "
                << "bits=" << std::min(tx_bits_frames[frm].size(), rx_bits.size())
                << ", errors=" << bit_errors
                << ", ber=" << ber
                << ", pss=" << pssPos
                << ", refined_pss=" << refined_pss
                << ", cfo=" << cfo_hat_hz
                << "\n";
        }

        tr.total_bit_errors = total_bit_errors;
        tr.total_compared_bits = total_compared_bits;
        tr.total_ber = (total_compared_bits > 0)
            ? static_cast<double>(total_bit_errors) / static_cast<double>(total_compared_bits)
            : 0.0;
        tr.decoded_frames = kNumFrames;

        build_constellation_result(last_eqSyms, tr);

        const VecComplex tx_sig_wave_mid = extract_center_segment(last_tx_sig, 4000);
        const VecComplex tx_sig_spec_mid = extract_center_segment(last_tx_sig, 1024);
        const VecComplex tx_sig_tf_mid = extract_center_segment(last_tx_sig, 4096);
        build_waveform_from_signal(tx_sig_wave_mid, ModulationType::QPSK, tr.waveform, tr.waveform_type);
        build_spectrum_from_signal(tx_sig_spec_mid, cfg.sampleRate(), tr.spectrum_freq, tr.spectrum_mag);
        build_spectrogram(tx_sig_tf_mid, cfg.sampleRate(), tr);

        const VecComplex rx_sig_wave_mid = extract_center_segment(last_rx_sig, 4000);
        const VecComplex rx_sig_spec_mid = extract_center_segment(last_rx_sig, 1024);
        build_waveform_from_signal(rx_sig_wave_mid, ModulationType::QPSK, tr.rx_waveform, tr.rx_waveform_type);
        build_spectrum_from_signal(rx_sig_spec_mid, cfg.sampleRate(), tr.rx_spectrum_freq, tr.rx_spectrum_mag);

        log << "========== OFDM RANDOM BIT TEST ==========\n";
        log << "[MODE] " << mode_to_string(mode) << "\n";
        log << "[CENTER FREQ] " << center_freq_hz << "\n";
        log << "[Frames] " << kNumFrames << "\n";
        log << "[fs] " << cfg.sampleRate() << "\n";
        log << "[Total compared bits] " << total_compared_bits << "\n";
        log << "[Total bit errors] " << total_bit_errors << "\n";
        log << "BER = " << tr.total_ber << "\n";
        tr.log_text = log.str();
        return tr;
    }

    for (int frm = 0; frm < kNumFrames; ++frm)
    {
        VecInt tx_bits;
        tx_bits.reserve(uncodedBitsPerFrame);
        for (int i = 0; i < uncodedBitsPerFrame; ++i) {
            tx_bits.push_back(dist01(rng));
        }
        VecComplex tx_sig = build_ofdm_random_frame_signal(
            cfg, tx_bits, pilotPos, dataPos2, pilotSeqRef);
        VecComplex pss = OFDMUtils::makePSS(cfg.N_fft);

        const size_t ofdm_frame_span =
            static_cast<size_t>(cfg.N_zeros) +
            pss.size() +
            static_cast<size_t>(cfg.Nd) * static_cast<size_t>(cfg.N_symbol());

        VecComplex rx_sig;
        if (mode == RunMode::LOOPBACK) {
            rx_sig = tx_sig;
        }
        else if (mode == RunMode::AWGN) {
            ChannelConfig cfg_local = ch_cfg;
            cfg_local.snr_dB = awgn_snr_db;

            if (cfg_local.enable_sto && cfg_local.sto_samp > 0) {
                const size_t tail_guard =
                    static_cast<size_t>(cfg_local.sto_samp) + (size_t)cfg.N_symbol();
                tx_sig.insert(tx_sig.end(), tail_guard, Complex(0.0, 0.0));
            }

            rx_sig = Channel::process(tx_sig, cfg_local, cfg.sampleRate());
        }
        else if (mode == RunMode::USRP) {
            VecComplex tx_usrp;
            const size_t guard_samps = 2 * ofdm_frame_span;
            tx_usrp.reserve(tx_sig.size() + 2 * guard_samps);
            tx_usrp.insert(tx_usrp.end(), guard_samps, Complex(0.0, 0.0));
            tx_usrp.insert(tx_usrp.end(), tx_sig.begin(), tx_sig.end());
            tx_usrp.insert(tx_usrp.end(), guard_samps, Complex(0.0, 0.0));

            rx_sig = transceive_usrp_burst(
                usrp_device_args,
                cfg.sampleRate(),
                center_freq_hz,
                tx_usrp,
                2 * ofdm_frame_span);
        }
        else {
            throw std::runtime_error("Unknown OFDM run mode.");
        }

        VecComplex eqSyms;
        VecInt rx_bits;
        double cfo_hat_hz = 0.0;
        int refined_pss = -1;
        if (mode == RunMode::USRP) {
            const int expected_pss =
                static_cast<int>(2 * ofdm_frame_span) + cfg.N_zeros;
            const int search_radius = static_cast<int>(2 * ofdm_frame_span);
            const int local_pss = detect_ofdm_pss_in_window(
                rx_sig, pss, expected_pss, search_radius, nullptr);
            if (local_pss < 0) {
                throw std::runtime_error("OFDM local PSS detect failed.");
            }
            if (!decode_ofdm_random_frame_signal(
                rx_sig, cfg, pilotPos, dataPos2, pilotSeqRef, local_pss, 64, false,
                rx_bits, &eqSyms, &cfo_hat_hz, &refined_pss)) {
                throw std::runtime_error("OFDM frame decode failed.");
            }
        }
        else if (!decode_ofdm_random_frame_signal(
            rx_sig, cfg, pilotPos, dataPos2, pilotSeqRef,
            rx_bits, &eqSyms, &cfo_hat_hz, &refined_pss)) {
            throw std::runtime_error("OFDM frame decode failed.");
        }

        if (rx_bits.size() > tx_bits.size()) {
            rx_bits.resize(tx_bits.size());
        }

        size_t bit_errors = 0;
        double ber = compute_ber(tx_bits, rx_bits, bit_errors);

        total_bit_errors += bit_errors;
        total_compared_bits += std::min(tx_bits.size(), rx_bits.size());

        // 锟斤拷锟斤拷锟斤拷锟揭恢★拷锟斤拷锟角帮拷锟斤拷锟绞?
        if (frm == kNumFrames - 1) {
            last_eqSyms = eqSyms;
            last_tx_sig = tx_sig;
            last_rx_sig = rx_sig;
        }

        log << "[Frame " << (frm + 1) << "/" << kNumFrames << "] "
            << "bits=" << std::min(tx_bits.size(), rx_bits.size())
            << ", errors=" << bit_errors
            << ", ber=" << ber
            << ", refined_pss=" << refined_pss
            << ", cfo=" << cfo_hat_hz
            << "\n";
    }

    tr.total_bit_errors = total_bit_errors;
    tr.total_compared_bits = total_compared_bits;
    tr.total_ber = (total_compared_bits > 0)
        ? static_cast<double>(total_bit_errors) / static_cast<double>(total_compared_bits)
        : 0.0;
    tr.decoded_frames = kNumFrames;

    // ===== 图锟斤拷锟斤拷锟捷ｏ拷只锟斤拷示锟斤拷锟揭恢?=====
    build_constellation_result(last_eqSyms, tr);

    const VecComplex tx_sig_wave_mid = extract_center_segment(last_tx_sig, 4000);
    const VecComplex tx_sig_spec_mid = extract_center_segment(last_tx_sig, 1024);
    const VecComplex tx_sig_tf_mid = extract_center_segment(last_tx_sig, 4096);

    build_waveform_from_signal(
        tx_sig_wave_mid,
        ModulationType::QPSK,
        tr.waveform,
        tr.waveform_type
    );

    build_spectrum_from_signal(
        tx_sig_spec_mid,
        cfg.sampleRate(),
        tr.spectrum_freq,
        tr.spectrum_mag
    );

    build_spectrogram(tx_sig_tf_mid, cfg.sampleRate(), tr);

    const VecComplex rx_sig_wave_mid = extract_center_segment(last_rx_sig, 4000);
    const VecComplex rx_sig_spec_mid = extract_center_segment(last_rx_sig, 1024);

    build_waveform_from_signal(
        rx_sig_wave_mid,
        ModulationType::QPSK,
        tr.rx_waveform,
        tr.rx_waveform_type
    );

    build_spectrum_from_signal(
        rx_sig_spec_mid,
        cfg.sampleRate(),
        tr.rx_spectrum_freq,
        tr.rx_spectrum_mag
    );

    log << "========== OFDM RANDOM BIT TEST ==========\n";
    log << "[MODE] " << mode_to_string(mode) << "\n";
    log << "[CENTER FREQ] " << center_freq_hz << "\n";
    log << "[CHANNEL] STO=" << (ch_cfg.enable_sto ? "ON" : "OFF")
        << " CFO=" << (ch_cfg.enable_cfo ? "ON" : "OFF")
        << " SFO=" << (ch_cfg.enable_sfo ? "ON" : "OFF")
        << " AWGN=" << (ch_cfg.enable_awgn ? "ON" : "OFF") << "\n";
    log << "[STO samp] " << ch_cfg.sto_samp << "\n";
    log << "[CFO Hz] " << ch_cfg.cfo_hz << "\n";
    log << "[SFO ppm] " << ch_cfg.sfo_ppm << "\n";
    log << "[Frames] " << kNumFrames << "\n";
    log << "[N_fft] " << cfg.N_fft << "\n";
    log << "[N_cp] " << cfg.N_cp << "\n";
    log << "[N_sc] " << cfg.N_sc << "\n";
    log << "[Nd] " << cfg.Nd << "\n";
    log << "[P_f_inter] " << cfg.P_f_inter << "\n";
    log << "[M] " << cfg.M << "\n";
    log << "[delta_f] " << cfg.delta_f << "\n";
    log << "[fs] " << cfg.sampleRate() << "\n";
    log << "[OFDM anti-STO] PSS coarse/refined sync\n";
    log << "[OFDM anti-CFO] CP-based coarse CFO + ambiguity resolution\n";
    log << "[OFDM anti-SFO] pilot affine phase correction\n";
    log << "[Total compared bits] " << total_compared_bits << "\n";
    log << "[Total bit errors] " << total_bit_errors << "\n";
    log << "BER = " << tr.total_ber << "\n";

    tr.log_text = log.str();
    return tr;
}

TestResult run_ofdm_file_transfer_test(
    RunMode mode,
    double awgn_snr_db,
    double center_freq_hz,
    const std::string& input_file_path,
    const std::string& output_file_path,
    const std::string& usrp_device_args,
    const ChannelConfig& ch_cfg)
{
    std::ostringstream log;

    if (input_file_path.empty()) {
        throw std::runtime_error("OFDM input_file_path is empty");
    }
    if (output_file_path.empty()) {
        throw std::runtime_error("OFDM output_file_path is empty");
    }

    OFDMConfig cfg;
    cfg.N_fft = 256;
    cfg.N_cp = 32;
    cfg.N_sc = 128;
    cfg.Nd = 10;
    cfg.N_frm = 1;
    cfg.N_zeros = 256;
    cfg.P_f_inter = 6;
    cfg.M = 4;
    cfg.L = 7;
    cfg.tblen = 32;
    cfg.delta_f = 15e3;

    std::ifstream ifs(input_file_path, std::ios::binary);
    if (!ifs) {
        throw std::runtime_error("Cannot open OFDM input file: " + input_file_path);
    }
    std::vector<uint8_t> file_bytes{
        std::istreambuf_iterator<char>(ifs),
        std::istreambuf_iterator<char>() };
    if (file_bytes.empty()) {
        throw std::runtime_error("OFDM input file is empty: " + input_file_path);
    }

    std::string filename_only = input_file_path;
    const size_t slash = filename_only.find_last_of("/\\");
    if (slash != std::string::npos) {
        filename_only = filename_only.substr(slash + 1);
    }

    OFDMImageTransmitter tx(cfg);
    OFDMImageReceiver rx(cfg);

    std::vector<VecComplex> tx_frames = tx.buildFileFrames(filename_only, file_bytes);
    VecComplex tx_sig = tx.buildFileSignal(filename_only, file_bytes);
    const size_t pss_len = OFDMUtils::makePSS(cfg.N_fft).size();
    const size_t ofdm_frame_span =
        static_cast<size_t>(cfg.N_zeros) +
        pss_len +
        static_cast<size_t>(cfg.Nd) * static_cast<size_t>(cfg.N_symbol());

    VecComplex rx_sig;
    if (mode == RunMode::LOOPBACK) {
        rx_sig = tx_sig;
    }
    else if (mode == RunMode::AWGN) {
        ChannelConfig cfg_local = ch_cfg;
        cfg_local.snr_dB = awgn_snr_db;
        if (cfg_local.enable_sto && cfg_local.sto_samp > 0) {
            const size_t tail_guard =
                static_cast<size_t>(cfg_local.sto_samp) + static_cast<size_t>(cfg.N_symbol());
            tx_sig.insert(tx_sig.end(), tail_guard, Complex(0.0, 0.0));
        }
        rx_sig = Channel::process(tx_sig, cfg_local, cfg.sampleRate());
    }
    else if (mode == RunMode::USRP) {
        const size_t guard_samps = 2 * ofdm_frame_span;
        VecComplex tx_usrp;
        tx_usrp.reserve(tx_sig.size() + 2 * guard_samps);
        tx_usrp.insert(tx_usrp.end(), guard_samps, Complex(0.0, 0.0));
        tx_usrp.insert(tx_usrp.end(), tx_sig.begin(), tx_sig.end());
        tx_usrp.insert(tx_usrp.end(), guard_samps, Complex(0.0, 0.0));

        rx_sig = transceive_usrp_burst(
            usrp_device_args,
            cfg.sampleRate(),
            center_freq_hz,
            tx_usrp,
            2 * ofdm_frame_span);
    }
    else {
        throw std::runtime_error("Unknown OFDM file run mode.");
    }

    std::string rx_name;
    std::vector<uint8_t> rx_file_bytes;
    const bool ok = rx.receiveFileSignal(rx_sig, rx_name, rx_file_bytes);

    TestResult tr;
    tr.decoded_frames = tx_frames.size();
    tr.total_compared_bits = std::min(file_bytes.size(), rx_file_bytes.size()) * 8;
    tr.file_saved = false;

    size_t byte_errors = 0;
    const size_t n = std::min(file_bytes.size(), rx_file_bytes.size());
    for (size_t i = 0; i < n; ++i) {
        if (file_bytes[i] != rx_file_bytes[i]) {
            byte_errors++;
        }
    }
    tr.total_bit_errors = byte_errors * 8;
    tr.total_ber = tr.total_compared_bits
        ? static_cast<double>(tr.total_bit_errors) / static_cast<double>(tr.total_compared_bits)
        : 0.0;

    if (ok) {
        std::ofstream ofs(output_file_path, std::ios::binary);
        if (!ofs) {
            throw std::runtime_error("Cannot open OFDM output file: " + output_file_path);
        }
        if (!rx_file_bytes.empty()) {
            ofs.write(
                reinterpret_cast<const char*>(rx_file_bytes.data()),
                static_cast<std::streamsize>(rx_file_bytes.size()));
        }
        tr.file_saved = true;
        tr.saved_file_path = output_file_path;
    }

    const VecComplex tx_sig_wave_mid = extract_center_segment(tx_sig, 4000);
    const VecComplex tx_sig_spec_mid = extract_center_segment(tx_sig, 1024);
    const VecComplex tx_sig_tf_mid = extract_center_segment(tx_sig, 4096);
    build_waveform_from_signal(tx_sig_wave_mid, ModulationType::QPSK, tr.waveform, tr.waveform_type);
    build_spectrum_from_signal(tx_sig_spec_mid, cfg.sampleRate(), tr.spectrum_freq, tr.spectrum_mag);
    build_spectrogram(tx_sig_tf_mid, cfg.sampleRate(), tr);

    const VecComplex rx_sig_wave_mid = extract_center_segment(rx_sig, 4000);
    const VecComplex rx_sig_spec_mid = extract_center_segment(rx_sig, 1024);
    build_waveform_from_signal(rx_sig_wave_mid, ModulationType::QPSK, tr.rx_waveform, tr.rx_waveform_type);
    build_spectrum_from_signal(rx_sig_spec_mid, cfg.sampleRate(), tr.rx_spectrum_freq, tr.rx_spectrum_mag);

    log << "========== OFDM FILE TRANSFER TEST ==========\n";
    log << "[MODE] " << mode_to_string(mode) << "\n";
    log << "[CENTER FREQ] " << center_freq_hz << "\n";
    log << "[INPUT FILE] " << input_file_path << "\n";
    log << "[OUTPUT FILE] " << output_file_path << "\n";
    log << "[FILE BYTES] " << file_bytes.size() << "\n";
    log << "[TX FRAMES] " << tx_frames.size() << "\n";
    log << "[CHANNEL] STO=" << (ch_cfg.enable_sto ? "ON" : "OFF")
        << " CFO=" << (ch_cfg.enable_cfo ? "ON" : "OFF")
        << " SFO=" << (ch_cfg.enable_sfo ? "ON" : "OFF")
        << " AWGN=" << (ch_cfg.enable_awgn ? "ON" : "OFF") << "\n";
    log << "[STO samp] " << ch_cfg.sto_samp << "\n";
    log << "[CFO Hz] " << ch_cfg.cfo_hz << "\n";
    log << "[SFO ppm] " << ch_cfg.sfo_ppm << "\n";
    log << "[RX OK] " << (ok ? 1 : 0) << "\n";
    log << "[RX NAME] " << rx_name << "\n";
    log << "[RX BYTES] " << rx_file_bytes.size() << "\n";
    log << "[BYTE ERRORS] " << byte_errors << "\n";
    log << "BER = " << tr.total_ber << "\n";

    tr.log_text = log.str();
    return tr;
}

TestResult run_channel_test(
    double snr_db,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation,
    double info_rate_bps,
    int hop_pattern,
    const ChannelConfig& ch_cfg
)
{
    std::ostringstream log;

    // ===== 锟斤拷锟斤拷锟斤拷锟斤拷锟?=====
    TransmitterConfig cfg;

    cfg.function =
        (modulation == ModulationType::MSK) ?
        FunctionType::RemoteControl :
        FunctionType::Telemetry;

    cfg.modulation = modulation;
    cfg.n = 10;
    cfg.frame_bit = 75;
    cfg.samp = 8;
    cfg.zp_sym = 33;
    cfg.Rb = info_rate_bps;
    cfg.hop_pattern = hop_pattern;
    cfg.connect = false;

    cfg.source_mode = SourceMode::RandomBits;

    Transmitter tx(cfg);

    VecComplex one_frame_sig = tx.generateTransmitSignal();
    VecInt one_frame_bits = tx.getLastSourceBits();

    cfg.fs = tx.getFS();

    Receiver rx(cfg);

    // ===== 锟斤拷锟斤拷 burst =====
    VecComplex tx_burst;
    for (int i = 0; i < tx_repeat_frames; ++i) {
        tx_burst.insert(tx_burst.end(), one_frame_sig.begin(), one_frame_sig.end());
    }

    // ===== 锟斤拷锟脚碉拷锟斤拷锟?=====
    // 锟斤拷锟斤拷 STO 锟斤拷锟斤拷追锟斤拷尾锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟睫筹拷 burst 锟斤拷前锟斤拷锟斤拷锟截讹拷锟斤拷锟斤拷锟斤拷锟街?
    if (ch_cfg.enable_sto && ch_cfg.sto_samp > 0) {
        const size_t tail_guard = static_cast<size_t>(ch_cfg.sto_samp) + one_frame_sig.size();
        tx_burst.insert(tx_burst.end(), tail_guard, Complex(0.0, 0.0));
    }

    ChannelConfig cfg_local = ch_cfg;
    cfg_local.snr_dB = snr_db;

    VecComplex rx_sig = Channel::process(tx_burst, cfg_local, cfg.fs);

    if (rx_sig.empty()) {
        throw std::runtime_error("rx_sig is empty.");
    }

    // ===== 锟斤拷锟斤拷 =====
    VecInt rx_bits = rx.receive(rx_sig);

    TestResult tr;
    build_constellation_result(rx.getLastConstellationPoints(), tr);

    // ===== BER =====
    const size_t bits_per_frame = one_frame_bits.size();
    const size_t decoded_frames =
        bits_per_frame > 0 ? (rx_bits.size() / bits_per_frame) : 0;

    size_t total_compared_bits = 0;
    size_t total_bit_errors = 0;

    for (size_t i = 0; i < decoded_frames; ++i)
    {
        VecInt rx_frame(
            rx_bits.begin() + static_cast<long long>(i * bits_per_frame),
            rx_bits.begin() + static_cast<long long>((i + 1) * bits_per_frame)
        );

        size_t frame_errors = 0;
        compute_ber(one_frame_bits, rx_frame, frame_errors);

        total_bit_errors += frame_errors;
        total_compared_bits += bits_per_frame;
    }

    tr.total_bit_errors = total_bit_errors;
    tr.total_compared_bits = total_compared_bits;
    tr.decoded_frames = decoded_frames;

    tr.total_ber =
        total_compared_bits ?
        (double)total_bit_errors / (double)total_compared_bits :
        0.0;

    // ===== 锟斤拷锟斤拷/频锟斤拷 =====
    {
        const VecComplex& tx_sig_full = tx.getLastPureModulatedSignal();

        const VecComplex tx_mid = extract_center_segment(tx_sig_full, 1024);

        build_waveform_from_signal(tx_mid, modulation,
            tr.waveform, tr.waveform_type);

        build_spectrum_from_signal(tx_mid, cfg.fs,
            tr.spectrum_freq, tr.spectrum_mag);
    }

    {
        const VecComplex rx_mid = extract_center_segment(rx_sig, 1024);

        build_waveform_from_signal(rx_mid, modulation,
            tr.rx_waveform, tr.rx_waveform_type);

        build_spectrum_from_signal(rx_mid, cfg.fs,
            tr.rx_spectrum_freq, tr.rx_spectrum_mag);
    }

    // ===== 锟斤拷志 =====
    log << "========== CUSTOM CHANNEL TEST ==========\n";
    log << "[MOD] " << modulation_to_string(modulation) << "\n";
    log << "[SNR] " << snr_db << " dB\n";

    log << "[STO] " << (ch_cfg.enable_sto ? "ON" : "OFF")
        << " samp=" << ch_cfg.sto_samp << "\n";

    log << "[CFO] " << (ch_cfg.enable_cfo ? "ON" : "OFF")
        << " Hz=" << ch_cfg.cfo_hz << "\n";

    log << "[SFO] " << (ch_cfg.enable_sfo ? "ON" : "OFF")
        << " ppm=" << ch_cfg.sfo_ppm << "\n";

    log << "[AWGN] " << (ch_cfg.enable_awgn ? "ON" : "OFF") << "\n";

    log << "[DECODED FRAMES] " << decoded_frames << "\n";
    log << "BER = " << tr.total_ber << "\n";

    tr.log_text = log.str();

    return tr;
}
