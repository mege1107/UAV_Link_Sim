#pragma once

#include <vector>
#include <complex>
#include <string>
#include <cstdint>

using Complex = std::complex<double>;
using VecComplex = std::vector<Complex>;
using VecInt = std::vector<int>;
using VecDouble = std::vector<double>;

struct OFDMConfig
{
    // ===== 来自 parameter.mat / SystemSetup =====
    int N_fft = 256;          // IFFT/FFT 点数
    int N_cp = 32;            // 循环前缀长度
    int N_sc = 128;           // 实际使用的有效子载波数（不含 DC）
    int Nd = 10;              // 一帧内 OFDM 符号数
    int N_frm = 1;            // 接收端里有用到
    int N_zeros = 256;        // 发射前导零
    int P_f_inter = 6;        // 导频间隔
    int M = 4;                // QAM 阶数，如 4/16
    int L = 7;                // 卷积码约束长度
    int tblen = 32;           // 维特比 traceback 长度
    double delta_f = 15e3;    // 子载波间隔

    // ===== 派生参数 =====
    int N_symbol() const { return N_cp + N_fft; }
    double sampleRate() const { return N_fft * delta_f; }
    double channelCodingRate() const { return 0.5; }
};

struct GrayImage
{
    int width = 0;
    int height = 0;
    std::vector<uint8_t> pixels; // row-major, grayscale
};

struct OFDMDebugInfo
{
    std::vector<double> pilot_avg_phase;
    std::vector<double> pilot_residual_rms;
};

class OFDMUtils
{
public:
    static VecInt bytesToBits(const std::vector<uint8_t>& bytes);
    static std::vector<uint8_t> bitsToBytes(const VecInt& bits);

    static VecInt uint16ToBits(uint16_t v);
    static uint16_t bitsToUint16(const VecInt& bits, size_t start);

    static VecComplex fft(const VecComplex& x);
    static VecComplex ifft(const VecComplex& x);

    static VecInt mseq(const std::vector<int>& seed);
    static VecComplex makePSS(int N_fft);

    static std::vector<int> pilotPositions(int N_sc, int P_f_inter);
    static std::vector<int> dataPositions(int N_sc, int P_f_inter);

    static VecComplex qamMod(const VecInt& bits, int M);
    static VecInt qamDemod(const VecComplex& syms, int M);

    static VecInt convEncode_171_133(const VecInt& bits);
    static VecInt viterbiDecodeHard_171_133(const VecInt& bits, int tblen);

    static std::vector<Complex> linearInterpChannel(
        const std::vector<int>& pilotPos,
        const VecComplex& pilotRxCol,
        const VecComplex& pilotTxCol,
        const std::vector<int>& dataPos);

    static GrayImage rebuildImageFromBits(const VecInt& decodedBits);
};

class OFDMImageTransmitter
{
public:
    explicit OFDMImageTransmitter(const OFDMConfig& cfg);

    VecComplex buildBitFrame(const VecInt& bits);
    VecComplex buildBitSignal(const VecInt& bitstream);
    std::vector<VecComplex> buildFileFrames(const std::string& filename, const std::vector<uint8_t>& fileBytes);
    VecComplex buildFileSignal(const std::string& filename, const std::vector<uint8_t>& fileBytes);
    VecComplex buildSingleImageFrame(const GrayImage& image);
    std::vector<VecComplex> buildImageFrames(const GrayImage& image);
    VecComplex buildImageSignal(const GrayImage& image);
    VecComplex buildMultiImageSignal(const std::vector<GrayImage>& images);

    int calcPayloadBitsPerFrame() const;
    int calcImageChunkBytesPerFrame() const;

private:
    OFDMConfig cfg_;

    std::vector<VecComplex> buildFramesFromBitstream(const VecInt& bitstream) const;
    VecInt imageToPayloadBits(const GrayImage& image) const;
    VecInt imageChunkToPayloadBits(
        const GrayImage& image,
        uint16_t chunkIndex,
        uint16_t chunkCount,
        size_t byteOffset,
        size_t chunkBytes) const;
    std::vector<std::vector<Complex>> mapToResourceGrid(const VecComplex& qamSyms) const;
    VecComplex buildTimeDomainSignal(const std::vector<std::vector<Complex>>& grid) const;
};

class OFDMImageReceiver
{
public:
    explicit OFDMImageReceiver(const OFDMConfig& cfg);

    bool receiveFrameBits(const VecComplex& rx, VecInt& bitsOut, VecComplex* eqSymsOut = nullptr);
    bool receiveFrameBitsAtPss(
        const VecComplex& rx,
        int initialPssPos,
        VecInt& bitsOut,
        VecComplex* eqSymsOut = nullptr,
        double* cfoAliasHzOut = nullptr,
        int fftWindowDelta = 0,
        OFDMDebugInfo* debugOut = nullptr);
    bool receiveBitSignal(const VecComplex& rx, VecInt& bitsOut);
    bool receiveFileSignal(const VecComplex& rx, std::string& filename, std::vector<uint8_t>& fileBytes);
    bool receiveOneFrame(const VecComplex& rx, GrayImage& outImg);
    bool receiveImageSignal(const VecComplex& rx, GrayImage& outImg);

private:
    OFDMConfig cfg_;

    int detectPSS(const VecComplex& rx, const VecComplex& pss) const;
    bool extractOFDMSymbols(
        const VecComplex& rx,
        int startPos,
        std::vector<VecComplex>& symbolsNoCP) const;

    void cfoCompensateSimple(std::vector<VecComplex>& symbolsNoCP) const;

    bool equalizeAndDemod(
        const std::vector<VecComplex>& symbolsNoCP,
        VecInt& hardBitsOut,
        VecComplex* eqSymsOut = nullptr,
        OFDMDebugInfo* debugOut = nullptr) const;
    bool decodeFramePayloadAtPss(
        const VecComplex& rx,
        int pssPos,
        VecInt& decodedBits,
        int& refinedPssPos) const;
    bool receiveBitstreamFromSignal(const VecComplex& rx, VecInt& bitsOut) const;
    bool decodeFramePayload(
        const VecComplex& rx,
        size_t searchStart,
        VecInt& decodedBits,
        size_t& frameStartPos) const;
};
