#include "ofdm_link.h"
#include "file_transfer.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <numeric>
#include <limits>
#include <iostream>

namespace {
    constexpr double PI = 3.14159265358979323846;
    constexpr size_t kFileHeaderBytes = 16;
    constexpr bool kEnableDebugLogs = true;

    inline int normalizeBit(int v) {
        return v ? 1 : 0;
    }

    std::vector<uint8_t> bitsToBytesNormalized(const VecInt& bits) {
        std::vector<uint8_t> out;
        const size_t n = bits.size() / 8;
        out.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            unsigned int v = 0;
            for (int k = 0; k < 8; ++k) {
                v = (v << 1) | static_cast<unsigned int>(normalizeBit(bits[i * 8 + k]));
            }
            out.push_back(static_cast<uint8_t>(v & 0xFFu));
        }
        return out;
    }

    inline Complex cexpj(double x) {
        return Complex(std::cos(x), std::sin(x));
    }

    int log2Int(int x) {
        int r = 0;
        while ((1 << r) < x) ++r;
        return r;
    }

    int binVecToInt(const VecInt& b, size_t st, int n) {
        int v = 0;
        for (int i = 0; i < n; ++i) {
            v = (v << 1) | ((st + i < b.size()) ? normalizeBit(b[st + i]) : 0);
        }
        return v;
    }

    VecInt intToBinVec(int v, int n) {
        VecInt out(n, 0);
        for (int i = n - 1; i >= 0; --i) {
            out[i] = (v & 1);
            v >>= 1;
        }
        return out;
    }

    // 线性同余伪随机，仅用于补齐 bit，模仿 MATLAB randi 的“补随机”
    int prbsBit() {
        static uint32_t s = 1u;
        s = 1664525u * s + 1013904223u;
        return (s >> 31) & 1u;
    }

    void appendUint16Bytes(std::vector<uint8_t>& out, uint16_t v) {
        out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
        out.push_back(static_cast<uint8_t>(v & 0xFF));
    }

    void appendUint64Bytes(std::vector<uint8_t>& out, uint64_t v) {
        for (int i = 7; i >= 0; --i) {
            out.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
        }
    }

    uint16_t readUint16Bytes(const std::vector<uint8_t>& data, size_t pos) {
        return static_cast<uint16_t>(
            (static_cast<uint16_t>(data[pos]) << 8) |
            static_cast<uint16_t>(data[pos + 1]));
    }

    uint64_t readUint64Bytes(const std::vector<uint8_t>& data, size_t pos) {
        uint64_t v = 0;
        for (int i = 0; i < 8; ++i) {
            v = (v << 8) | static_cast<uint64_t>(data[pos + i]);
        }
        return v;
    }

    std::string toLowerAscii(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
            if (c >= 'A' && c <= 'Z') return static_cast<char>(c - 'A' + 'a');
            return static_cast<char>(c);
        });
        return s;
    }

    uint8_t fileTypeFromFilename(const std::string& filename) {
        const std::string lower = toLowerAscii(filename);
        const size_t dot = lower.find_last_of('.');
        const std::string ext = (dot == std::string::npos) ? std::string() : lower.substr(dot);
        if (ext == ".txt") return 1;
        if (ext == ".jpg" || ext == ".jpeg") return 2;
        if (ext == ".mp4") return 3;
        return 255;
    }

    std::vector<uint8_t> buildFilePacketBytes(
        const std::string& filename,
        const std::vector<uint8_t>& fileBytes,
        uint8_t fileType) {
        if (filename.size() > 65535u) {
            throw std::runtime_error("Filename too long");
        }

        std::vector<uint8_t> packet;
        packet.reserve(kFileHeaderBytes + filename.size() + fileBytes.size());
        packet.push_back('U');
        packet.push_back('A');
        packet.push_back('V');
        packet.push_back('F');
        packet.push_back(1);
        packet.push_back(fileType);
        appendUint16Bytes(packet, static_cast<uint16_t>(filename.size()));
        appendUint64Bytes(packet, static_cast<uint64_t>(fileBytes.size()));
        packet.insert(packet.end(), filename.begin(), filename.end());
        packet.insert(packet.end(), fileBytes.begin(), fileBytes.end());
        return packet;
    }

    bool recoverFileFromPacketBits(
        const VecInt& bits,
        std::string& filename,
        std::vector<uint8_t>& fileBytes) {
        filename.clear();
        fileBytes.clear();

        std::vector<uint8_t> bytes = bitsToBytesNormalized(bits);
        if (kEnableDebugLogs) {
            std::cout << "[DBG][OFDM FILE RX] total_bits=" << bits.size()
                << " total_bytes=" << bytes.size() << "\n";
        }
        if (bytes.size() < kFileHeaderBytes) return false;
        if (!(bytes[0] == 'U' && bytes[1] == 'A' && bytes[2] == 'V' && bytes[3] == 'F')) {
            if (kEnableDebugLogs) {
                std::cout << "[DBG][OFDM FILE RX] magic mismatch: "
                    << static_cast<int>(bytes[0]) << " "
                    << static_cast<int>(bytes[1]) << " "
                    << static_cast<int>(bytes[2]) << " "
                    << static_cast<int>(bytes[3]) << "\n";
            }
            return false;
        }
        if (bytes[4] != 1) return false;

        const uint16_t filenameLen = readUint16Bytes(bytes, 6);
        const uint64_t fileSize = readUint64Bytes(bytes, 8);
        const size_t payloadOffset = kFileHeaderBytes + static_cast<size_t>(filenameLen);
        if (kEnableDebugLogs) {
            std::cout << "[DBG][OFDM FILE RX] version=" << static_cast<int>(bytes[4])
                << " file_type=" << static_cast<int>(bytes[5])
                << " filenameLen=" << filenameLen
                << " fileSize=" << fileSize
                << " payloadOffset=" << payloadOffset << "\n";
        }
        if (bytes.size() < payloadOffset + static_cast<size_t>(fileSize)) return false;

        filename.assign(
            bytes.begin() + static_cast<std::ptrdiff_t>(kFileHeaderBytes),
            bytes.begin() + static_cast<std::ptrdiff_t>(payloadOffset));
        fileBytes.assign(
            bytes.begin() + static_cast<std::ptrdiff_t>(payloadOffset),
            bytes.begin() + static_cast<std::ptrdiff_t>(payloadOffset + static_cast<size_t>(fileSize)));
        return true;
    }

    double wrap_phase_pm_pi(double x) {
        while (x > PI) x -= 2.0 * PI;
        while (x < -PI) x += 2.0 * PI;
        return x;
    }

    double estimateOfdmCfoFromCp(const VecComplex& rx, int pssPos, const OFDMConfig& cfg) {
        const int pssLen = static_cast<int>(OFDMUtils::makePSS(cfg.N_fft).size());
        const int cpStart = pssPos + pssLen;
        const int dataStart = cpStart + cfg.N_cp;
        const int tailStart = dataStart + cfg.N_fft - cfg.N_cp;
        const int needEnd = dataStart + cfg.N_fft;

        if (cpStart < 0 || needEnd > static_cast<int>(rx.size())) return 0.0;

        Complex acc(0.0, 0.0);
        for (int n = 0; n < cfg.N_cp; ++n) {
            acc += std::conj(rx[static_cast<size_t>(cpStart + n)]) * rx[static_cast<size_t>(tailStart + n)];
        }
        if (std::abs(acc) <= 1e-12) return 0.0;

        const double phase = std::atan2(acc.imag(), acc.real());
        return phase * cfg.sampleRate() / (2.0 * PI * static_cast<double>(cfg.N_fft));
    }

    VecComplex applyFrequencyCorrection(const VecComplex& signal, double fs, double freqHz) {
        if (signal.empty()) return signal;
        if (!std::isfinite(fs) || fs <= 0.0) return signal;
        if (!std::isfinite(freqHz) || std::abs(freqHz) < 1e-12) return signal;

        VecComplex out(signal.size());
        for (size_t n = 0; n < signal.size(); ++n) {
            const double ang = 2.0 * PI * freqHz * static_cast<double>(n) / fs;
            out[n] = signal[n] * std::conj(Complex(std::cos(ang), std::sin(ang)));
        }
        return out;
    }

    void fit_phase_line(const std::vector<int>& x, const std::vector<double>& y, double& a, double& b) {
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

    std::string bitsToString(const VecInt& bits, size_t n = 64) {
        std::string s;
        const size_t m = std::min(n, bits.size());
        s.reserve(m);
        for (size_t i = 0; i < m; ++i) {
            s.push_back(bits[i] ? '1' : '0');
        }
        return s;
    }

    size_t frameSpanSamples(const OFDMConfig& cfg) {
        return static_cast<size_t>(cfg.N_zeros)
            + OFDMUtils::makePSS(cfg.N_fft).size()
            + static_cast<size_t>(cfg.Nd) * cfg.N_symbol();
    }

    int detectPssInWindow(
        const VecComplex& rx,
        const VecComplex& pss,
        int center,
        int radius,
        double* outMetric = nullptr)
    {
        if (outMetric) *outMetric = -1.0;
        if (rx.size() < pss.size()) return -1;

        const int maxStart = static_cast<int>(rx.size() - pss.size());
        const int start = std::max(0, center - radius);
        const int end = std::min(maxStart, center + radius);
        if (start > end) return -1;

        double bestMetric = -1.0;
        int bestPos = -1;
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
            if (metric > bestMetric) {
                bestMetric = metric;
                bestPos = d;
            }
        }

        if (outMetric) *outMetric = bestMetric;
        return bestPos;
    }

    int findFirstValidPss(
        const VecComplex& rx,
        const VecComplex& pss,
        double threshold,
        double* outMetric = nullptr)
    {
        if (outMetric) *outMetric = -1.0;
        if (rx.size() < pss.size()) return -1;

        const int maxStart = static_cast<int>(rx.size() - pss.size());
        for (int d = 0; d <= maxStart; ++d) {
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
                double refinedMetric = metric;
                const int refined = detectPssInWindow(rx, pss, d, 12, &refinedMetric);
                if (outMetric) *outMetric = refinedMetric;
                return (refined >= 0) ? refined : d;
            }
        }

        return -1;
    }

    bool extractOfdmSymbolsFromPss(
        const VecComplex& rx,
        int pssPos,
        const OFDMConfig& cfg,
        std::vector<VecComplex>& symbolsNoCP)
    {
        symbolsNoCP.clear();
        const int pssLen = static_cast<int>(OFDMUtils::makePSS(cfg.N_fft).size());
        const int pos0 = pssPos + pssLen - 1;

        for (int i = 0; i < cfg.Nd; ++i) {
            const int cpStart = pos0 + i * cfg.N_symbol();
            const int dataStart = cpStart + cfg.N_cp;
            const int dataEnd = dataStart + cfg.N_fft;
            if (dataStart < 0 || dataEnd > static_cast<int>(rx.size())) {
                symbolsNoCP.clear();
                return false;
            }
            symbolsNoCP.emplace_back(
                rx.begin() + static_cast<std::ptrdiff_t>(dataStart),
                rx.begin() + static_cast<std::ptrdiff_t>(dataEnd));
        }
        return true;
    }

    double scoreOfdmCfoCandidate(
        const VecComplex& rx,
        const VecComplex& pss,
        const OFDMConfig& cfg,
        double cfoHz)
    {
        const VecComplex rxCorr = applyFrequencyCorrection(rx, cfg.sampleRate(), cfoHz);
        const int pssPos = detectPssInWindow(
            rxCorr, pss, cfg.N_zeros, std::max(24, cfg.N_cp), nullptr);
        if (pssPos < 0) return -1e300;

        std::vector<VecComplex> symbolsNoCP;
        if (!extractOfdmSymbolsFromPss(rxCorr, pssPos, cfg, symbolsNoCP)) {
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
        int scoreTerms = 0;
        const int evalCols = std::min(cfg.Nd, 3);
        for (int col = 0; col < evalCols; ++col) {
            VecComplex Y = OFDMUtils::fft(symbolsNoCP[col]);
            VecComplex used(cfg.N_sc);
            for (int k = 0; k < cfg.N_sc; ++k) {
                used[k] = Y[1 + k];
            }

            std::vector<double> phase(pilotNum, 0.0);
            double magSum = 0.0;
            for (int i = 0; i < pilotNum; ++i) {
                Complex h = (std::abs(pilotSeq[i][col]) > 1e-12)
                    ? (used[pilotPos[i]] / pilotSeq[i][col])
                    : Complex(1.0, 0.0);
                phase[i] = std::atan2(h.imag(), h.real());
                magSum += std::abs(h);
                if (i > 0) {
                    phase[i] = phase[i - 1] + wrap_phase_pm_pi(phase[i] - phase[i - 1]);
                }
            }

            double a = 0.0, b = 0.0;
            fit_phase_line(pilotPos, phase, a, b);

            double residualVar = 0.0;
            for (int i = 0; i < pilotNum; ++i) {
                const double pred = a * static_cast<double>(pilotPos[i]) + b;
                const double err = wrap_phase_pm_pi(phase[i] - pred);
                residualVar += err * err;
            }
            residualVar /= static_cast<double>(pilotNum);

            const double magAvg = magSum / static_cast<double>(pilotNum);
            score += magAvg - 5.0 * residualVar;
            scoreTerms++;
        }

        if (scoreTerms <= 0) return -1e300;
        return score / static_cast<double>(scoreTerms);
    }

    double resolveOfdmCfoAmbiguity(
        const VecComplex& rx,
        const VecComplex& pss,
        const OFDMConfig& cfg,
        double cfoAliasHz)
    {
        const double stepHz = cfg.sampleRate() / static_cast<double>(cfg.N_fft);
        double bestCfo = cfoAliasHz;
        double bestScore = -1e300;

        for (int k = -2; k <= 2; ++k) {
            const double cand = cfoAliasHz + static_cast<double>(k) * stepHz;
            const double score = scoreOfdmCfoCandidate(rx, pss, cfg, cand);
            if (score > bestScore) {
                bestScore = score;
                bestCfo = cand;
            }
        }
        return bestCfo;
    }

}

// =============================
// OFDMUtils
// =============================

VecInt OFDMUtils::bytesToBits(const std::vector<uint8_t>& bytes)
{
    VecInt bits;
    bits.reserve(bytes.size() * 8);
    for (uint8_t b : bytes) {
        for (int i = 7; i >= 0; --i) {
            bits.push_back((b >> i) & 1);
        }
    }
    return bits;
}

std::vector<uint8_t> OFDMUtils::bitsToBytes(const VecInt& bits)
{
    std::vector<uint8_t> out;
    size_t n = bits.size() / 8;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        uint8_t v = 0;
        for (int k = 0; k < 8; ++k) {
            v = static_cast<uint8_t>((v << 1) | normalizeBit(bits[i * 8 + k]));
        }
        out.push_back(v);
    }
    return out;
}

VecInt OFDMUtils::uint16ToBits(uint16_t v)
{
    VecInt bits(16, 0);
    for (int i = 15; i >= 0; --i) {
        bits[15 - i] = (v >> i) & 1;
    }
    return bits;
}

uint16_t OFDMUtils::bitsToUint16(const VecInt& bits, size_t start)
{
    uint16_t v = 0;
    for (int i = 0; i < 16; ++i) {
        v = static_cast<uint16_t>(
            (v << 1) | ((start + i < bits.size()) ? normalizeBit(bits[start + i]) : 0));
    }
    return v;
}

VecComplex OFDMUtils::fft(const VecComplex& x)
{
    const size_t N = x.size();
    VecComplex X(N, Complex(0.0, 0.0));
    for (size_t k = 0; k < N; ++k) {
        Complex s(0.0, 0.0);
        for (size_t n = 0; n < N; ++n) {
            s += x[n] * cexpj(-2.0 * PI * k * n / static_cast<double>(N));
        }
        X[k] = s;
    }
    return X;
}

VecComplex OFDMUtils::ifft(const VecComplex& X)
{
    const size_t N = X.size();
    VecComplex x(N, Complex(0.0, 0.0));
    for (size_t n = 0; n < N; ++n) {
        Complex s(0.0, 0.0);
        for (size_t k = 0; k < N; ++k) {
            s += X[k] * cexpj(2.0 * PI * k * n / static_cast<double>(N));
        }
        x[n] = s / static_cast<double>(N);
    }
    return x;
}

VecInt OFDMUtils::mseq(const std::vector<int>& seed)
{
    // 简化版 LFSR；这里只需要和发端/收端一致
    std::vector<int> reg = seed;
    if (reg.empty()) return {};

    const int m = static_cast<int>(reg.size());
    const int len = (1 << m) - 1;
    VecInt seq;
    seq.reserve(len);

    for (int i = 0; i < len; ++i) {
        int out = reg.back();
        seq.push_back(out);

        int fb = reg[m - 1] ^ reg[m - 2];
        for (int k = m - 1; k > 0; --k) reg[k] = reg[k - 1];
        reg[0] = fb;
    }
    return seq;
}

VecComplex OFDMUtils::makePSS(int N_fft)
{
    VecInt pss = mseq({ 1,1,1,0,1,1,0 });
    for (auto& b : pss) b = 2 * b - 1; // -> ±1

    // MATLAB: circshift(...,43)
    std::rotate(pss.rbegin(), pss.rbegin() + (43 % pss.size()), pss.rend());

    VecComplex out;
    out.reserve(pss.size());
    const double scale = std::sqrt(1.0 / static_cast<double>(N_fft));
    for (int v : pss) out.emplace_back(scale * v, 0.0);
    return out;
}

std::vector<int> OFDMUtils::pilotPositions(int N_sc, int P_f_inter)
{
    std::vector<int> pos;
    for (int i = 0; i < N_sc; i += P_f_inter) pos.push_back(i);
    return pos;
}

std::vector<int> OFDMUtils::dataPositions(int N_sc, int P_f_inter)
{
    std::vector<int> pos;
    for (int i = 0; i < N_sc; ++i) {
        if (i % P_f_inter != 0) pos.push_back(i);
    }
    return pos;
}

VecComplex OFDMUtils::qamMod(const VecInt& bits, int M)
{
    const int k = log2Int(M);
    if ((1 << k) != M) throw std::runtime_error("M must be power of 2");

    VecComplex syms;
    syms.reserve((bits.size() + k - 1) / k);

    if (M == 4) {
        // QPSK / 4QAM
        for (size_t i = 0; i < bits.size(); i += 2) {
            int b0 = (i < bits.size()) ? bits[i] : 0;
            int b1 = (i + 1 < bits.size()) ? bits[i + 1] : 0;
            double I = (b0 == 0) ? -1.0 : 1.0;
            double Q = (b1 == 0) ? -1.0 : 1.0;
            syms.emplace_back(I, Q);
        }
        for (auto& s : syms) s /= std::sqrt(2.0);
        return syms;
    }

    if (M == 16) {
        for (size_t i = 0; i < bits.size(); i += 4) {
            int b0 = (i < bits.size()) ? bits[i] : 0;
            int b1 = (i + 1 < bits.size()) ? bits[i + 1] : 0;
            int b2 = (i + 2 < bits.size()) ? bits[i + 2] : 0;
            int b3 = (i + 3 < bits.size()) ? bits[i + 3] : 0;

            auto map2 = [](int a, int b) {
                int v = (a << 1) | b;
                switch (v) {
                case 0: return -3.0;
                case 1: return -1.0;
                case 3: return  1.0;
                case 2: return  3.0;
                }
                return -3.0;
                };

            double I = map2(b0, b1);
            double Q = map2(b2, b3);
            syms.emplace_back(I, Q);
        }
        for (auto& s : syms) s /= std::sqrt(10.0);
        return syms;
    }

    throw std::runtime_error("Only M=4/16 currently implemented");
}

VecInt OFDMUtils::qamDemod(const VecComplex& syms, int M)
{
    VecInt bits;

    if (M == 4) {
        bits.reserve(syms.size() * 2);
        for (const auto& s : syms) {
            bits.push_back(s.real() >= 0 ? 1 : 0);
            bits.push_back(s.imag() >= 0 ? 1 : 0);
        }
        return bits;
    }

    if (M == 16) {
        bits.reserve(syms.size() * 4);

        auto dec2 = [](double x) {
            if (x < -2.0) return std::pair<int, int>{0, 0};
            if (x < 0.0) return std::pair<int, int>{0, 1};
            if (x < 2.0) return std::pair<int, int>{1, 1};
            return std::pair<int, int>{1, 0};
            };

        for (auto s : syms) {
            s *= std::sqrt(10.0);
            auto [b0, b1] = dec2(s.real());
            auto [b2, b3] = dec2(s.imag());
            bits.push_back(b0); bits.push_back(b1);
            bits.push_back(b2); bits.push_back(b3);
        }
        return bits;
    }

    throw std::runtime_error("Only M=4/16 currently implemented");
}

VecInt OFDMUtils::convEncode_171_133(const VecInt& bits)
{
    // rate 1/2, K=7, g0=171(oct), g1=133(oct)
    const int g0 = 0x79;
    const int g1 = 0x5B;
    int state = 0;
    VecInt out;
    out.reserve(bits.size() * 2);

    for (int b : bits) {
        state = ((state << 1) | (b & 1)) & 0x7F;

        auto parity = [](int x) {
            int p = 0;
            while (x) { p ^= (x & 1); x >>= 1; }
            return p;
            };

        out.push_back(parity(state & g0));
        out.push_back(parity(state & g1));
    }
    return out;
}

VecInt OFDMUtils::viterbiDecodeHard_171_133(const VecInt& bits, int tblen)
{
    // 简化硬判决 Viterbi
    if (bits.size() % 2 != 0) return {};

    const int numStates = 64;
    const int INF = 1e9;
    const size_t T = bits.size() / 2;

    std::vector<std::vector<int>> metric(T + 1, std::vector<int>(numStates, INF));
    std::vector<std::vector<int>> prevSt(T + 1, std::vector<int>(numStates, -1));
    std::vector<std::vector<int>> prevBit(T + 1, std::vector<int>(numStates, 0));

    metric[0][0] = 0;

    auto parity = [](int x) {
        int p = 0;
        while (x) { p ^= (x & 1); x >>= 1; }
        return p;
        };

    const int g0 = 0x79;
    const int g1 = 0x5B;

    for (size_t t = 0; t < T; ++t) {
        int r0 = bits[2 * t];
        int r1 = bits[2 * t + 1];

        for (int st = 0; st < numStates; ++st) {
            if (metric[t][st] >= INF) continue;

            for (int inb = 0; inb <= 1; ++inb) {
                int reg = ((st << 1) | inb) & 0x7F;
                int ns = reg & 0x3F;
                int c0 = parity(reg & g0);
                int c1 = parity(reg & g1);
                int dist = (c0 != r0) + (c1 != r1);
                int cand = metric[t][st] + dist;

                if (cand < metric[t + 1][ns]) {
                    metric[t + 1][ns] = cand;
                    prevSt[t + 1][ns] = st;
                    prevBit[t + 1][ns] = inb;
                }
            }
        }
    }

    int bestState = 0;
    int bestMetric = INF;
    for (int st = 0; st < numStates; ++st) {
        if (metric[T][st] < bestMetric) {
            bestMetric = metric[T][st];
            bestState = st;
        }
    }

    VecInt decoded(T, 0);
    int st = bestState;
    for (size_t t = T; t > 0; --t) {
        decoded[t - 1] = prevBit[t][st];
        st = prevSt[t][st];
        if (st < 0) break;
    }

    if (tblen > 0 && static_cast<int>(decoded.size()) > tblen) {
        // MATLAB vitdec(...,'trunc','hard') 更接近整段输出；
        // 这里不截尾，只保留完整长度
    }

    return decoded;
}

std::vector<Complex> OFDMUtils::linearInterpChannel(
    const std::vector<int>& pilotPos,
    const VecComplex& pilotRxCol,
    const VecComplex& pilotTxCol,
    const std::vector<int>& dataPos)
{
    std::vector<Complex> H(dataPos.size(), Complex(1.0, 0.0));
    if (pilotPos.empty() || pilotRxCol.size() != pilotTxCol.size() || pilotRxCol.empty()) {
        return H;
    }

    std::vector<Complex> hp(pilotPos.size(), Complex(1.0, 0.0));
    for (size_t i = 0; i < pilotPos.size(); ++i) {
        if (std::abs(pilotTxCol[i]) > 1e-12) hp[i] = pilotRxCol[i] / pilotTxCol[i];
    }

    for (size_t i = 0; i < dataPos.size(); ++i) {
        int x = dataPos[i];

        if (x <= pilotPos.front()) {
            H[i] = hp.front();
            continue;
        }
        if (x >= pilotPos.back()) {
            H[i] = hp.back();
            continue;
        }

        for (size_t k = 0; k + 1 < pilotPos.size(); ++k) {
            int x0 = pilotPos[k];
            int x1 = pilotPos[k + 1];
            if (x0 <= x && x <= x1) {
                double a = (x - x0) / static_cast<double>(x1 - x0);
                H[i] = hp[k] * (1.0 - a) + hp[k + 1] * a;
                break;
            }
        }
    }

    return H;
}

GrayImage OFDMUtils::rebuildImageFromBits(const VecInt& decodedBits)
{
    GrayImage img;
    if (decodedBits.size() < 32) return img;

    img.height = bitsToUint16(decodedBits, 0);
    img.width = bitsToUint16(decodedBits, 16);

    if (img.width <= 0 || img.height <= 0) {
        img.width = 0;
        img.height = 0;
        return img;
    }

    const size_t needBits = static_cast<size_t>(img.width) * img.height * 8;
    if (decodedBits.size() < 32 + needBits) {
        img.width = 0;
        img.height = 0;
        return img;
    }

    VecInt payload(decodedBits.begin() + 32, decodedBits.begin() + 32 + needBits);
    img.pixels = bitsToBytes(payload);
    return img;
}

// =============================
// OFDMImageTransmitter
// =============================

OFDMImageTransmitter::OFDMImageTransmitter(const OFDMConfig& cfg)
    : cfg_(cfg)
{
}

std::vector<VecComplex> OFDMImageTransmitter::buildFramesFromBitstream(const VecInt& bitstream) const
{
    const int payloadBitsPerFrame = calcPayloadBitsPerFrame();
    if (payloadBitsPerFrame <= 0) {
        throw std::runtime_error("OFDM frame payload too small");
    }

    const size_t totalFrameCount =
        (bitstream.size() + static_cast<size_t>(payloadBitsPerFrame) - 1) / static_cast<size_t>(payloadBitsPerFrame);
    if (totalFrameCount == 0) {
        return {};
    }

    std::vector<VecComplex> frames;
    frames.reserve(totalFrameCount);

    for (size_t i = 0; i < totalFrameCount; ++i) {
        const size_t bitStart = i * static_cast<size_t>(payloadBitsPerFrame);
        const size_t bitEnd = std::min(bitStart + static_cast<size_t>(payloadBitsPerFrame), bitstream.size());
        VecInt payloadBits(
            bitstream.begin() + static_cast<std::ptrdiff_t>(bitStart),
            bitstream.begin() + static_cast<std::ptrdiff_t>(bitEnd));
        while (static_cast<int>(payloadBits.size()) < payloadBitsPerFrame) {
            payloadBits.push_back(0);
        }

        VecInt coded = OFDMUtils::convEncode_171_133(payloadBits);
        VecComplex qam = OFDMUtils::qamMod(coded, cfg_.M);

        auto grid = mapToResourceGrid(qam);
        VecComplex txData = buildTimeDomainSignal(grid);

        VecComplex out;
        out.reserve(cfg_.N_zeros + OFDMUtils::makePSS(cfg_.N_fft).size() + txData.size());
        out.insert(out.end(), cfg_.N_zeros, Complex(0.0, 0.0));

        VecComplex pss = OFDMUtils::makePSS(cfg_.N_fft);
        out.insert(out.end(), pss.begin(), pss.end());
        out.insert(out.end(), txData.begin(), txData.end());
        frames.push_back(std::move(out));
    }

    return frames;
}

std::vector<VecComplex> OFDMImageTransmitter::buildFileFrames(
    const std::string& filename,
    const std::vector<uint8_t>& fileBytes)
{
    const std::vector<uint8_t> packetBytes =
        buildFilePacketBytes(filename, fileBytes, fileTypeFromFilename(filename));
    const VecInt packetBits = OFDMUtils::bytesToBits(packetBytes);

    if (kEnableDebugLogs) {
        std::cout << "[DBG][OFDM FILE TX] filename=" << filename
            << " fileBytes=" << fileBytes.size()
            << " packetBytes=" << packetBytes.size()
            << " packetBits=" << packetBits.size() << "\n";
        std::cout << "[DBG][OFDM FILE TX] first_bytes=";
        for (size_t i = 0; i < std::min<size_t>(16, packetBytes.size()); ++i) {
            std::cout << " " << static_cast<int>(packetBytes[i]);
        }
        std::cout << "\n";
        std::cout << "[DBG][OFDM FILE TX] first_bits=" << bitsToString(packetBits, 64) << "\n";
    }

    return buildFramesFromBitstream(packetBits);
}

VecComplex OFDMImageTransmitter::buildFileSignal(
    const std::string& filename,
    const std::vector<uint8_t>& fileBytes)
{
    std::vector<VecComplex> frames = buildFileFrames(filename, fileBytes);
    VecComplex total;
    size_t totalSamples = 0;
    for (const auto& frame : frames) totalSamples += frame.size();
    total.reserve(totalSamples);

    for (const auto& frame : frames) {
        total.insert(total.end(), frame.begin(), frame.end());
    }
    return total;
}

int OFDMImageTransmitter::calcPayloadBitsPerFrame() const
{
    auto dataPos = OFDMUtils::dataPositions(cfg_.N_sc, cfg_.P_f_inter);
    const int dataRow = static_cast<int>(dataPos.size());
    const int N_RE_data = dataRow * cfg_.Nd;
    return static_cast<int>(N_RE_data * cfg_.channelCodingRate() * log2Int(cfg_.M));
}

int OFDMImageTransmitter::calcImageChunkBytesPerFrame() const
{
    const int payloadBits = calcPayloadBitsPerFrame();
    return payloadBits / 8;
}

VecInt OFDMImageTransmitter::imageToPayloadBits(const GrayImage& image) const
{
    if (image.width <= 0 || image.height <= 0 || image.pixels.empty()) {
        throw std::runtime_error("Invalid image");
    }

    VecInt bits;
    auto hbits = OFDMUtils::uint16ToBits(static_cast<uint16_t>(image.height));
    auto wbits = OFDMUtils::uint16ToBits(static_cast<uint16_t>(image.width));
    bits.insert(bits.end(), hbits.begin(), hbits.end());
    bits.insert(bits.end(), wbits.begin(), wbits.end());

    auto pixBits = OFDMUtils::bytesToBits(image.pixels);
    bits.insert(bits.end(), pixBits.begin(), pixBits.end());

    const int maxBits = calcPayloadBitsPerFrame();
    if (static_cast<int>(bits.size()) > maxBits) {
        throw std::runtime_error("Image too large for one OFDM frame payload");
    }

    while (static_cast<int>(bits.size()) < maxBits) {
        bits.push_back(prbsBit());
    }

    return bits;
}

VecInt OFDMImageTransmitter::imageChunkToPayloadBits(
    const GrayImage& image,
    uint16_t chunkIndex,
    uint16_t chunkCount,
    size_t byteOffset,
    size_t chunkBytes) const
{
    if (image.width <= 0 || image.height <= 0 || image.pixels.empty()) {
        throw std::runtime_error("Invalid image");
    }
    (void)image;
    (void)chunkIndex;
    (void)chunkCount;
    (void)byteOffset;
    (void)chunkBytes;
    return {};
}

std::vector<std::vector<Complex>> OFDMImageTransmitter::mapToResourceGrid(const VecComplex& qamSyms) const
{
    auto pilotPos = OFDMUtils::pilotPositions(cfg_.N_sc, cfg_.P_f_inter);
    auto dataPos = OFDMUtils::dataPositions(cfg_.N_sc, cfg_.P_f_inter);

    const int pilotNum = static_cast<int>(pilotPos.size());
    const int dataRow = static_cast<int>(dataPos.size());

    // grid: N_sc x Nd
    std::vector<std::vector<Complex>> grid(cfg_.N_sc, std::vector<Complex>(cfg_.Nd, Complex(0.0, 0.0)));

    // MATLAB: rand('seed',1); pilot_seq = 2*(randi([0 1],pilot_num,data_col))-1;
    std::vector<std::vector<Complex>> pilotSeq(pilotNum, std::vector<Complex>(cfg_.Nd, Complex(1.0, 0.0)));
    uint32_t s = 1u;
    auto rbit = [&]() {
        s = 1664525u * s + 1013904223u;
        return (s >> 31) & 1u;
        };

    for (int r = 0; r < pilotNum; ++r) {
        for (int c = 0; c < cfg_.Nd; ++c) {
            pilotSeq[r][c] = Complex(rbit() ? 1.0 : -1.0, 0.0);
            grid[pilotPos[r]][c] = pilotSeq[r][c];
        }
    }

    size_t idx = 0;
    for (int c = 0; c < cfg_.Nd; ++c) {
        for (int r = 0; r < dataRow; ++r) {
            grid[dataPos[r]][c] = (idx < qamSyms.size()) ? qamSyms[idx++] : Complex(0.0, 0.0);
        }
    }

    return grid;
}

VecComplex OFDMImageTransmitter::buildTimeDomainSignal(
    const std::vector<std::vector<Complex>>& grid) const
{
    VecComplex tx;

    // 每列一个 OFDM symbol
    for (int col = 0; col < cfg_.Nd; ++col) {
        VecComplex X(cfg_.N_fft, Complex(0.0, 0.0));

        // MATLAB:
        // data = [zeros(1,Nd); data];
        // data3=[data; zeros(N_fft-N_sc-1,Nd)];
        // 即 bin0 为 DC=0，bin1..binN_sc 为有效载波
        for (int k = 0; k < cfg_.N_sc; ++k) {
            X[1 + k] = grid[k][col];
        }

        VecComplex x = OFDMUtils::ifft(X);

        // 加 CP
        VecComplex one;
        one.reserve(cfg_.N_cp + cfg_.N_fft);
        for (int i = cfg_.N_fft - cfg_.N_cp; i < cfg_.N_fft; ++i) one.push_back(x[i]);
        one.insert(one.end(), x.begin(), x.end());

        tx.insert(tx.end(), one.begin(), one.end());
    }

    return tx;
}

VecComplex OFDMImageTransmitter::buildSingleImageFrame(const GrayImage& image)
{
    VecInt payloadBits = imageToPayloadBits(image);
    VecInt coded = OFDMUtils::convEncode_171_133(payloadBits);
    VecComplex qam = OFDMUtils::qamMod(coded, cfg_.M);

    auto grid = mapToResourceGrid(qam);
    VecComplex txData = buildTimeDomainSignal(grid);

    VecComplex out;
    out.reserve(cfg_.N_zeros + OFDMUtils::makePSS(cfg_.N_fft).size() + txData.size());

    out.insert(out.end(), cfg_.N_zeros, Complex(0.0, 0.0));

    VecComplex pss = OFDMUtils::makePSS(cfg_.N_fft);
    out.insert(out.end(), pss.begin(), pss.end());

    out.insert(out.end(), txData.begin(), txData.end());
    return out;
}

std::vector<VecComplex> OFDMImageTransmitter::buildImageFrames(const GrayImage& image)
{
    if (image.width <= 0 || image.height <= 0 || image.pixels.empty()) {
        throw std::runtime_error("Invalid image");
    }
    const std::string pseudoName = "image.gray";
    return buildFileFrames(pseudoName, image.pixels);
}

VecComplex OFDMImageTransmitter::buildImageSignal(const GrayImage& image)
{
    const std::string pseudoName = "image.gray";
    return buildFileSignal(pseudoName, image.pixels);
}

VecComplex OFDMImageTransmitter::buildMultiImageSignal(const std::vector<GrayImage>& images)
{
    VecComplex total;
    for (const auto& img : images) {
        VecComplex one = buildImageSignal(img);
        total.insert(total.end(), one.begin(), one.end());
    }
    return total;
}

// =============================
// OFDMImageReceiver
// =============================

OFDMImageReceiver::OFDMImageReceiver(const OFDMConfig& cfg)
    : cfg_(cfg)
{
}

int OFDMImageReceiver::detectPSS(const VecComplex& rx, const VecComplex& pss) const
{
    if (rx.size() < pss.size()) return -1;

    // In the multi-frame file path, each chunk is already cut to one nominal frame.
    // Searching only around the known zero-prefix boundary is much more stable than
    // globally correlating over the whole chunk, which can occasionally lock onto
    // a strong data-region sidelobe.
    if (rx.size() == frameSpanSamples(cfg_)) {
        double metric = -1.0;
        const int pos = detectPssInWindow(rx, pss, cfg_.N_zeros, 24, &metric);
        if (kEnableDebugLogs) {
            std::cout << "[DBG][OFDM PSS] windowed_center=" << cfg_.N_zeros
                << " refined=" << pos
                << " metric=" << metric << "\n";
        }
        if (pos >= 0) {
            return pos;
        }
    }

    double best = -1.0;
    int coarsePos = -1;
    for (size_t d = 0; d + pss.size() <= rx.size(); ++d) {
        Complex acc(0.0, 0.0);
        for (size_t k = 0; k < pss.size(); ++k) {
            acc += std::conj(pss[k]) * rx[d + k];
        }
        const double v = std::norm(acc);
        if (v > best) {
            best = v;
            coarsePos = static_cast<int>(d);
        }
    }

    if (coarsePos < 0) return -1;

    const int refineRadius = 12;
    const int start = std::max(0, coarsePos - refineRadius);
    const int end = std::min(static_cast<int>(rx.size() - pss.size()), coarsePos + refineRadius);
    double refineBest = -1.0;
    int refinePos = coarsePos;

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
        if (metric > refineBest) {
            refineBest = metric;
            refinePos = d;
        }
    }

    if (kEnableDebugLogs) {
        std::cout << "[DBG][OFDM PSS] coarse=" << coarsePos
            << " refined=" << refinePos
            << " metric=" << refineBest << "\n";
    }
    return refinePos;
}

bool OFDMImageReceiver::extractOFDMSymbols(
    const VecComplex& rx,
    int startPos,
    std::vector<VecComplex>& symbolsNoCP) const
{
    return extractOfdmSymbolsFromPss(rx, startPos, cfg_, symbolsNoCP);
}

void OFDMImageReceiver::cfoCompensateSimple(std::vector<VecComplex>& symbolsNoCP) const
{
    // 先放一个占位函数
    // 你 MATLAB 里调的是 CFO_compensate(...)
    // 真正完全对齐它，需要你把这个函数源码也贴出来
}

bool OFDMImageReceiver::equalizeAndDemod(
    const std::vector<VecComplex>& symbolsNoCP,
    VecInt& hardBitsOut) const
{
    hardBitsOut.clear();

    auto pilotPos = OFDMUtils::pilotPositions(cfg_.N_sc, cfg_.P_f_inter);
    auto dataPos = OFDMUtils::dataPositions(cfg_.N_sc, cfg_.P_f_inter);

    const int pilotNum = static_cast<int>(pilotPos.size());

    // 重建与发端一致的 pilot_seq
    std::vector<std::vector<Complex>> pilotSeq(pilotNum, std::vector<Complex>(cfg_.Nd, Complex(1.0, 0.0)));
    uint32_t s = 1u;
    auto rbit = [&]() {
        s = 1664525u * s + 1013904223u;
        return (s >> 31) & 1u;
        };
    for (int r = 0; r < pilotNum; ++r) {
        for (int c = 0; c < cfg_.Nd; ++c) {
            pilotSeq[r][c] = Complex(rbit() ? 1.0 : -1.0, 0.0);
        }
    }

    VecComplex allEqSyms;

    for (int col = 0; col < cfg_.Nd; ++col) {
        VecComplex Y = OFDMUtils::fft(symbolsNoCP[col]);

        // 只取 [1 ... N_sc]
        VecComplex used(cfg_.N_sc);
        for (int k = 0; k < cfg_.N_sc; ++k) {
            used[k] = Y[1 + k];
        }

        VecComplex rxPilot(pilotNum);
        for (int i = 0; i < pilotNum; ++i) rxPilot[i] = used[pilotPos[i]];

        VecComplex txPilot(pilotNum);
        for (int i = 0; i < pilotNum; ++i) txPilot[i] = pilotSeq[i][col];

        std::vector<double> pilotPhase(pilotNum, 0.0);
        for (int i = 0; i < pilotNum; ++i) {
            Complex h = (std::abs(txPilot[i]) > 1e-12)
                ? (rxPilot[i] / txPilot[i])
                : Complex(1.0, 0.0);
            pilotPhase[i] = std::atan2(h.imag(), h.real());
            if (i > 0) {
                pilotPhase[i] = pilotPhase[i - 1] +
                    wrap_phase_pm_pi(pilotPhase[i] - pilotPhase[i - 1]);
            }
        }

        double phaseSlope = 0.0;
        double phaseBias = 0.0;
        fit_phase_line(pilotPos, pilotPhase, phaseSlope, phaseBias);

        VecComplex usedPhaseCorrected = used;
        for (int k = 0; k < cfg_.N_sc; ++k) {
            const double ph = phaseSlope * static_cast<double>(k) + phaseBias;
            usedPhaseCorrected[k] *= std::conj(Complex(std::cos(ph), std::sin(ph)));
        }

        VecComplex rxPilotCorrected(pilotNum);
        for (int i = 0; i < pilotNum; ++i) {
            rxPilotCorrected[i] = usedPhaseCorrected[pilotPos[i]];
        }

        auto H = OFDMUtils::linearInterpChannel(pilotPos, rxPilotCorrected, txPilot, dataPos);

        for (size_t i = 0; i < dataPos.size(); ++i) {
            Complex h = H[i];
            if (std::abs(h) < 1e-12) h = Complex(1.0, 0.0);
            allEqSyms.push_back(usedPhaseCorrected[dataPos[i]] / h);
        }
    }

    hardBitsOut = OFDMUtils::qamDemod(allEqSyms, cfg_.M);
    return !hardBitsOut.empty();
}

bool OFDMImageReceiver::decodeFramePayloadAtPss(
    const VecComplex& rx,
    int pssPos,
    VecInt& decodedBits,
    int& refinedPssPos) const
{
    decodedBits.clear();
    refinedPssPos = pssPos;
    if (pssPos < 0 || static_cast<size_t>(pssPos) >= rx.size()) return false;

    const VecComplex pss = OFDMUtils::makePSS(cfg_.N_fft);
    const double cfoAliasHz = estimateOfdmCfoFromCp(rx, pssPos, cfg_);
    const double cfoHatHz = resolveOfdmCfoAmbiguity(rx, pss, cfg_, cfoAliasHz);

    if (kEnableDebugLogs) {
        std::cout << "[DBG][OFDM FRAME] pssPos=" << pssPos
            << " cfoAliasHz=" << cfoAliasHz
            << " cfoHatHz=" << cfoHatHz << "\n";
    }

    VecComplex rxWork = rx;
    if (std::isfinite(cfoHatHz) && std::abs(cfoHatHz) > 1e-9) {
        rxWork = applyFrequencyCorrection(rx, cfg_.sampleRate(), cfoHatHz);
    }

    const int refined = detectPssInWindow(
        rxWork, pss, pssPos, std::max(24, cfg_.N_cp), nullptr);
    if (refined >= 0) {
        refinedPssPos = refined;
    }

    if (kEnableDebugLogs) {
        std::cout << "[DBG][OFDM FRAME] pssPosAfterCfo=" << refinedPssPos << "\n";
    }

    std::vector<VecComplex> symbolsNoCP;
    if (!extractOfdmSymbolsFromPss(rxWork, refinedPssPos, cfg_, symbolsNoCP)) {
        return false;
    }

    VecInt demodBits;
    if (!equalizeAndDemod(symbolsNoCP, demodBits)) {
        return false;
    }

    decodedBits = OFDMUtils::viterbiDecodeHard_171_133(demodBits, cfg_.tblen);
    if (kEnableDebugLogs) {
        std::cout << "[DBG][OFDM FRAME] demodBits=" << demodBits.size()
            << " decodedBits=" << decodedBits.size() << "\n";
        if (!decodedBits.empty()) {
            const std::vector<uint8_t> firstBytes =
                bitsToBytesNormalized(VecInt(
                    decodedBits.begin(),
                    decodedBits.begin() + static_cast<std::ptrdiff_t>(std::min<size_t>(decodedBits.size(), 64))));
            std::cout << "[DBG][OFDM FRAME] first_bits=" << bitsToString(decodedBits, 32) << "\n";
            std::cout << "[DBG][OFDM FRAME] first_bytes=";
            for (size_t i = 0; i < std::min<size_t>(8, firstBytes.size()); ++i) {
                std::cout << " " << static_cast<int>(firstBytes[i]);
            }
            std::cout << "\n";
        }
    }

    return !decodedBits.empty();
}

bool OFDMImageReceiver::decodeFramePayload(
    const VecComplex& rx,
    size_t searchStart,
    VecInt& decodedBits,
    size_t& frameStartPos) const
{
    decodedBits.clear();
    frameStartPos = 0;
    if (searchStart >= rx.size()) return false;

    VecComplex search(rx.begin() + static_cast<std::ptrdiff_t>(searchStart), rx.end());
    VecComplex pss = OFDMUtils::makePSS(cfg_.N_fft);
    int p = detectPSS(search, pss);
    if (p < 0) return false;

    frameStartPos = searchStart + static_cast<size_t>(p);
    int refinedPssPos = static_cast<int>(frameStartPos);
    const bool ok = decodeFramePayloadAtPss(rx, static_cast<int>(frameStartPos), decodedBits, refinedPssPos);
    frameStartPos = static_cast<size_t>(std::max(refinedPssPos, 0));
    return ok;
}

bool OFDMImageReceiver::receiveBitstreamFromSignal(const VecComplex& rx, VecInt& bitsOut) const
{
    bitsOut.clear();

    const size_t span = frameSpanSamples(cfg_);
    const size_t frameCount = rx.size() / span;
    const int payloadBitsPerFrame = OFDMImageTransmitter(cfg_).calcPayloadBitsPerFrame();
    if (payloadBitsPerFrame <= 0) return false;

    bitsOut.reserve(frameCount * static_cast<size_t>(payloadBitsPerFrame));

    const VecComplex pss = OFDMUtils::makePSS(cfg_.N_fft);
    double firstMetric = -1.0;
    int firstPssPos = findFirstValidPss(rx, pss, 0.4, &firstMetric);
    if (firstPssPos < 0) {
        firstPssPos = detectPSS(rx, pss);
    }
    if (firstPssPos < 0) {
        if (kEnableDebugLogs) {
            std::cout << "[DBG][OFDM FILE RX] first frame PSS detect failed\n";
        }
        return false;
    }

    const int firstFrameBase = firstPssPos - cfg_.N_zeros;
    if (kEnableDebugLogs) {
        std::cout << "[DBG][OFDM FILE RX] firstPssPos=" << firstPssPos
            << " firstFrameBase=" << firstFrameBase
            << " firstMetric=" << firstMetric
            << " frameSpan=" << span << "\n";
    }

    for (size_t frameIdx = 0; frameIdx < frameCount; ++frameIdx) {
        VecInt decoded;
        const int expectedPssPos =
            firstPssPos + static_cast<int>(frameIdx * span);
        const int localRadius = std::max(24, cfg_.N_cp * 2);
        double metric = -1.0;
        const int pssPos = detectPssInWindow(rx, pss, expectedPssPos, localRadius, &metric);
        if (pssPos < 0) {
            if (kEnableDebugLogs) {
                std::cout << "[DBG][OFDM FILE RX] frame " << frameIdx
                    << " local PSS detect failed expected=" << expectedPssPos << "\n";
            }
            return false;
        }
        int refinedPssPos = pssPos;
        if (!decodeFramePayloadAtPss(rx, pssPos, decoded, refinedPssPos)) {
            if (kEnableDebugLogs) {
                std::cout << "[DBG][OFDM FILE RX] frame " << frameIdx << " decode failed\n";
            }
            return false;
        }
        if (decoded.size() < static_cast<size_t>(payloadBitsPerFrame)) return false;
        if (kEnableDebugLogs) {
            std::cout << "[DBG][OFDM FILE RX] frame " << frameIdx
                << " payloadBits=" << payloadBitsPerFrame
                << " expectedPssPos=" << expectedPssPos
                << " pssPos=" << pssPos
                << " refinedPssPos=" << refinedPssPos
                << " metric=" << metric << "\n";
        }

        bitsOut.insert(
            bitsOut.end(),
            decoded.begin(),
            decoded.begin() + static_cast<std::ptrdiff_t>(payloadBitsPerFrame));
    }

    return true;
}

bool OFDMImageReceiver::receiveFileSignal(
    const VecComplex& rx,
    std::string& filename,
    std::vector<uint8_t>& fileBytes)
{
    VecInt bits;
    if (!receiveBitstreamFromSignal(rx, bits)) return false;
    return recoverFileFromPacketBits(bits, filename, fileBytes);
}

bool OFDMImageReceiver::receiveOneFrame(const VecComplex& rx, GrayImage& outImg)
{
    outImg = {};
    VecInt decoded;
    size_t frameStartPos = 0;
    if (!decodeFramePayload(rx, 0, decoded, frameStartPos)) return false;

    std::string filename;
    std::vector<uint8_t> fileBytes;
    if (recoverFileFromPacketBits(decoded, filename, fileBytes) && !fileBytes.empty()) {
        outImg.width = static_cast<int>(fileBytes.size());
        outImg.height = 1;
        outImg.pixels = std::move(fileBytes);
        return true;
    }

    outImg = OFDMUtils::rebuildImageFromBits(decoded);
    return (outImg.width > 0 && outImg.height > 0 && !outImg.pixels.empty());
}

bool OFDMImageReceiver::receiveImageSignal(const VecComplex& rx, GrayImage& outImg)
{
    outImg = {};
    std::string filename;
    std::vector<uint8_t> fileBytes;
    if (!receiveFileSignal(rx, filename, fileBytes)) return false;
    if (fileBytes.empty()) return false;

    outImg.width = static_cast<int>(fileBytes.size());
    outImg.height = 1;
    outImg.pixels = std::move(fileBytes);
    return true;
}
