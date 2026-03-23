#include "ofdm_link.h"

#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <numeric>
#include <limits>
#include <iostream>

namespace {
    constexpr double PI = 3.14159265358979323846;

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
            v = (v << 1) | ((st + i < b.size()) ? b[st + i] : 0);
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
            v = static_cast<uint8_t>((v << 1) | (bits[i * 8 + k] & 1));
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
        v = static_cast<uint16_t>((v << 1) | ((start + i < bits.size()) ? bits[start + i] : 0));
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

int OFDMImageTransmitter::calcPayloadBitsPerFrame() const
{
    auto dataPos = OFDMUtils::dataPositions(cfg_.N_sc, cfg_.P_f_inter);
    const int dataRow = static_cast<int>(dataPos.size());
    const int N_RE_data = dataRow * cfg_.Nd;
    return static_cast<int>(N_RE_data * cfg_.channelCodingRate() * log2Int(cfg_.M));
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

VecComplex OFDMImageTransmitter::buildMultiImageSignal(const std::vector<GrayImage>& images)
{
    VecComplex total;
    for (const auto& img : images) {
        VecComplex one = buildSingleImageFrame(img);
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

    double best = -1.0;
    int bestPos = -1;

    for (size_t d = 0; d + pss.size() <= rx.size(); ++d) {
        Complex acc(0.0, 0.0);
        for (size_t k = 0; k < pss.size(); ++k) {
            acc += std::conj(pss[k]) * rx[d + k];
        }
        double v = std::abs(acc);
        if (v > best) {
            best = v;
            bestPos = static_cast<int>(d);
        }
    }

    return bestPos;
}

bool OFDMImageReceiver::extractOFDMSymbols(
    const VecComplex& rx,
    int startPos,
    std::vector<VecComplex>& symbolsNoCP) const
{
    symbolsNoCP.clear();

    const int pssLen = static_cast<int>(OFDMUtils::makePSS(cfg_.N_fft).size());
    int pos0 = startPos + pssLen - 1;

    for (int i = 0; i < cfg_.Nd; ++i) {
        int cpStart = pos0 + i * cfg_.N_symbol();
        int dataStart = cpStart + cfg_.N_cp;
        int dataEnd = dataStart + cfg_.N_fft;

        if (dataStart < 0 || dataEnd > static_cast<int>(rx.size())) return false;

        symbolsNoCP.emplace_back(rx.begin() + dataStart, rx.begin() + dataEnd);
    }
    return true;
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

        auto H = OFDMUtils::linearInterpChannel(pilotPos, rxPilot, txPilot, dataPos);

        for (size_t i = 0; i < dataPos.size(); ++i) {
            Complex h = H[i];
            if (std::abs(h) < 1e-12) h = Complex(1.0, 0.0);
            allEqSyms.push_back(used[dataPos[i]] / h);
        }
    }

    hardBitsOut = OFDMUtils::qamDemod(allEqSyms, cfg_.M);
    return !hardBitsOut.empty();
}

bool OFDMImageReceiver::receiveOneFrame(const VecComplex& rx, GrayImage& outImg)
{
    outImg = {};

    VecComplex pss = OFDMUtils::makePSS(cfg_.N_fft);
    int p = detectPSS(rx, pss);
    if (p < 0) return false;

    std::vector<VecComplex> symbolsNoCP;
    if (!extractOFDMSymbols(rx, p, symbolsNoCP)) {
        return false;
    }

    cfoCompensateSimple(symbolsNoCP);

    VecInt demodBits;
    if (!equalizeAndDemod(symbolsNoCP, demodBits)) {
        return false;
    }

    VecInt decoded = OFDMUtils::viterbiDecodeHard_171_133(demodBits, cfg_.tblen);
    if (decoded.empty()) return false;

    outImg = OFDMUtils::rebuildImageFromBits(decoded);
    return (outImg.width > 0 && outImg.height > 0 && !outImg.pixels.empty());
}