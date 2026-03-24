$path = 'D:\linksim\UAV_Link_Sim\UAV_Link_Sim\ofdm_link.cpp'
$content = Get-Content -Raw -Path $path

$old1 = @"
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
"@
$new1 = @"
namespace {
    constexpr double PI = 3.14159265358979323846;
    constexpr uint16_t kImageChunkMagic = 0x4F49; // \"OI\"
    constexpr size_t kImageChunkHeaderBits = 128;

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

    struct ImageChunkHeader {
        uint16_t magic = 0;
        uint16_t width = 0;
        uint16_t height = 0;
        uint32_t totalBytes = 0;
        uint16_t chunkIndex = 0;
        uint16_t chunkCount = 0;
        uint16_t chunkBytes = 0;
    };

    void appendUint32Bits(VecInt& bits, uint32_t v) {
        for (int i = 31; i >= 0; --i) {
            bits.push_back((v >> i) & 1u);
        }
    }

    uint32_t bitsToUint32(const VecInt& bits, size_t start) {
        uint32_t v = 0;
        for (int i = 0; i < 32; ++i) {
            v = (v << 1) | ((start + i < bits.size()) ? (bits[start + i] & 1u) : 0u);
        }
        return v;
    }

    VecInt chunkHeaderToBits(const ImageChunkHeader& header) {
        VecInt bits;
        bits.reserve(kImageChunkHeaderBits);

        VecInt magicBits = OFDMUtils::uint16ToBits(header.magic);
        VecInt widthBits = OFDMUtils::uint16ToBits(header.width);
        VecInt heightBits = OFDMUtils::uint16ToBits(header.height);
        VecInt chunkIndexBits = OFDMUtils::uint16ToBits(header.chunkIndex);
        VecInt chunkCountBits = OFDMUtils::uint16ToBits(header.chunkCount);
        VecInt chunkBytesBits = OFDMUtils::uint16ToBits(header.chunkBytes);

        bits.insert(bits.end(), magicBits.begin(), magicBits.end());
        bits.insert(bits.end(), widthBits.begin(), widthBits.end());
        bits.insert(bits.end(), heightBits.begin(), heightBits.end());
        appendUint32Bits(bits, header.totalBytes);
        bits.insert(bits.end(), chunkIndexBits.begin(), chunkIndexBits.end());
        bits.insert(bits.end(), chunkCountBits.begin(), chunkCountBits.end());
        bits.insert(bits.end(), chunkBytesBits.begin(), chunkBytesBits.end());
        return bits;
    }

    bool parseChunkHeader(const VecInt& bits, ImageChunkHeader& header) {
        if (bits.size() < kImageChunkHeaderBits) return false;

        header.magic = OFDMUtils::bitsToUint16(bits, 0);
        header.width = OFDMUtils::bitsToUint16(bits, 16);
        header.height = OFDMUtils::bitsToUint16(bits, 32);
        header.totalBytes = bitsToUint32(bits, 48);
        header.chunkIndex = OFDMUtils::bitsToUint16(bits, 80);
        header.chunkCount = OFDMUtils::bitsToUint16(bits, 96);
        header.chunkBytes = OFDMUtils::bitsToUint16(bits, 112);
        return true;
    }

    size_t ofdmFrameSpanSamples(const OFDMConfig& cfg) {
        return static_cast<size_t>(cfg.N_zeros)
            + OFDMUtils::makePSS(cfg.N_fft).size()
            + static_cast<size_t>(cfg.Nd) * cfg.N_symbol();
    }
}
"@
if (-not $content.Contains($old1)) { throw 'old1 not found' }
$content = $content.Replace($old1, $new1)

$old2 = @"
int OFDMImageTransmitter::calcPayloadBitsPerFrame() const
{
    auto dataPos = OFDMUtils::dataPositions(cfg_.N_sc, cfg_.P_f_inter);
    const int dataRow = static_cast<int>(dataPos.size());
    const int N_RE_data = dataRow * cfg_.Nd;
    return static_cast<int>(N_RE_data * cfg_.channelCodingRate() * log2Int(cfg_.M));
}
"@
$new2 = @"
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
    if (payloadBits <= static_cast<int>(kImageChunkHeaderBits)) {
        return 0;
    }
    return (payloadBits - static_cast<int>(kImageChunkHeaderBits)) / 8;
}
"@
if (-not $content.Contains($old2)) { throw 'old2 not found' }
$content = $content.Replace($old2, $new2)

$old3 = @"
VecInt OFDMImageTransmitter::imageToPayloadBits(const GrayImage& image) const
{
    if (image.width <= 0 || image.height <= 0 || image.pixels.empty()) {
        throw std::runtime_error(\"Invalid image\");
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
        throw std::runtime_error(\"Image too large for one OFDM frame payload\");
    }

    while (static_cast<int>(bits.size()) < maxBits) {
        bits.push_back(prbsBit());
    }

    return bits;
}
"@
$new3 = @"
VecInt OFDMImageTransmitter::imageToPayloadBits(const GrayImage& image) const
{
    if (image.width <= 0 || image.height <= 0 || image.pixels.empty()) {
        throw std::runtime_error(\"Invalid image\");
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
        throw std::runtime_error(\"Image too large for one OFDM frame payload\");
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
        throw std::runtime_error(\"Invalid image\");
    }

    if (byteOffset + chunkBytes > image.pixels.size()) {
        throw std::runtime_error(\"Image chunk range out of bounds\");
    }

    const int payloadBitsPerFrame = calcPayloadBitsPerFrame();
    if (payloadBitsPerFrame <= static_cast<int>(kImageChunkHeaderBits)) {
        throw std::runtime_error(\"OFDM payload too small for image chunk header\");
    }

    ImageChunkHeader header;
    header.magic = kImageChunkMagic;
    header.width = static_cast<uint16_t>(image.width);
    header.height = static_cast<uint16_t>(image.height);
    header.totalBytes = static_cast<uint32_t>(image.pixels.size());
    header.chunkIndex = chunkIndex;
    header.chunkCount = chunkCount;
    header.chunkBytes = static_cast<uint16_t>(chunkBytes);

    VecInt bits = chunkHeaderToBits(header);
    std::vector<uint8_t> chunk(
        image.pixels.begin() + static_cast<std::ptrdiff_t>(byteOffset),
        image.pixels.begin() + static_cast<std::ptrdiff_t>(byteOffset + chunkBytes));
    VecInt chunkBits = OFDMUtils::bytesToBits(chunk);
    bits.insert(bits.end(), chunkBits.begin(), chunkBits.end());

    while (static_cast<int>(bits.size()) < payloadBitsPerFrame) {
        bits.push_back(prbsBit());
    }

    return bits;
}
"@
if (-not $content.Contains($old3)) { throw 'old3 not found' }
$content = $content.Replace($old3, $new3)

$old4 = @"
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
"@
$new4 = @"
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
        throw std::runtime_error(\"Invalid image\");
    }

    const int chunkCapacity = calcImageChunkBytesPerFrame();
    if (chunkCapacity <= 0) {
        throw std::runtime_error(\"OFDM frame payload too small for chunked image transfer\");
    }

    const size_t totalBytes = image.pixels.size();
    const size_t chunkCountSz = (totalBytes + static_cast<size_t>(chunkCapacity) - 1)
        / static_cast<size_t>(chunkCapacity);
    if (chunkCountSz == 0 || chunkCountSz > std::numeric_limits<uint16_t>::max()) {
        throw std::runtime_error(\"Image requires too many OFDM chunks\");
    }

    std::vector<VecComplex> frames;
    frames.reserve(chunkCountSz);

    for (size_t i = 0; i < chunkCountSz; ++i) {
        const size_t byteOffset = i * static_cast<size_t>(chunkCapacity);
        const size_t chunkBytes = std::min(
            static_cast<size_t>(chunkCapacity),
            totalBytes - byteOffset);

        VecInt payloadBits = imageChunkToPayloadBits(
            image,
            static_cast<uint16_t>(i),
            static_cast<uint16_t>(chunkCountSz),
            byteOffset,
            chunkBytes);
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

VecComplex OFDMImageTransmitter::buildImageSignal(const GrayImage& image)
{
    std::vector<VecComplex> frames = buildImageFrames(image);
    VecComplex total;
    size_t totalSamples = 0;
    for (const auto& frame : frames) totalSamples += frame.size();
    total.reserve(totalSamples);

    for (const auto& frame : frames) {
        total.insert(total.end(), frame.begin(), frame.end());
    }
    return total;
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
"@
if (-not $content.Contains($old4)) { throw 'old4 not found' }
$content = $content.Replace($old4, $new4)

$old5 = @"
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
"@
$new5 = @"
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

    std::vector<VecComplex> symbolsNoCP;
    if (!extractOFDMSymbols(rx, static_cast<int>(frameStartPos), symbolsNoCP)) {
        return false;
    }

    cfoCompensateSimple(symbolsNoCP);

    VecInt demodBits;
    if (!equalizeAndDemod(symbolsNoCP, demodBits)) {
        return false;
    }

    decodedBits = OFDMUtils::viterbiDecodeHard_171_133(demodBits, cfg_.tblen);
    return !decodedBits.empty();
}

bool OFDMImageReceiver::receiveOneFrame(const VecComplex& rx, GrayImage& outImg)
{
    outImg = {};

    VecInt decoded;
    size_t frameStartPos = 0;
    if (!decodeFramePayload(rx, 0, decoded, frameStartPos)) return false;

    ImageChunkHeader header;
    if (parseChunkHeader(decoded, header) &&
        header.magic == kImageChunkMagic &&
        header.chunkCount == 1 &&
        header.chunkIndex == 0 &&
        header.chunkBytes > 0) {
        const size_t payloadStart = kImageChunkHeaderBits;
        const size_t payloadBits = static_cast<size_t>(header.chunkBytes) * 8;
        if (decoded.size() < payloadStart + payloadBits) return false;

        VecInt chunkBits(
            decoded.begin() + static_cast<std::ptrdiff_t>(payloadStart),
            decoded.begin() + static_cast<std::ptrdiff_t>(payloadStart + payloadBits));
        std::vector<uint8_t> bytes = OFDMUtils::bitsToBytes(chunkBits);

        if (bytes.size() != header.totalBytes) return false;
        if (static_cast<size_t>(header.width) * header.height != header.totalBytes) return false;

        outImg.width = header.width;
        outImg.height = header.height;
        outImg.pixels = std::move(bytes);
        return true;
    }

    GrayImage legacy = OFDMUtils::rebuildImageFromBits(decoded);
    if (legacy.width > 0 && legacy.height > 0 && !legacy.pixels.empty()) {
        outImg = std::move(legacy);
        return true;
    }

    return false;
}

bool OFDMImageReceiver::receiveImageSignal(const VecComplex& rx, GrayImage& outImg)
{
    outImg = {};

    const int chunkCapacity = OFDMImageTransmitter(cfg_).calcImageChunkBytesPerFrame();
    if (chunkCapacity <= 0) return false;

    const size_t frameSpan = ofdmFrameSpanSamples(cfg_);
    size_t cursor = 0;
    bool started = false;
    uint16_t expectedWidth = 0;
    uint16_t expectedHeight = 0;
    uint16_t expectedChunkCount = 0;
    uint32_t expectedTotalBytes = 0;
    std::vector<uint8_t> imageBytes;
    std::vector<bool> receivedChunks;

    while (cursor < rx.size()) {
        VecInt decoded;
        size_t frameStartPos = 0;
        if (!decodeFramePayload(rx, cursor, decoded, frameStartPos)) {
            break;
        }

        ImageChunkHeader header;
        if (!parseChunkHeader(decoded, header) || header.magic != kImageChunkMagic) {
            if (!started) {
                return receiveOneFrame(rx, outImg);
            }
            return false;
        }

        if (!started) {
            if (header.width == 0 || header.height == 0 || header.chunkCount == 0) return false;
            if (static_cast<size_t>(header.width) * header.height != header.totalBytes) return false;

            expectedWidth = header.width;
            expectedHeight = header.height;
            expectedChunkCount = header.chunkCount;
            expectedTotalBytes = header.totalBytes;
            imageBytes.assign(expectedTotalBytes, 0);
            receivedChunks.assign(expectedChunkCount, false);
            started = true;
        }

        if (header.width != expectedWidth ||
            header.height != expectedHeight ||
            header.chunkCount != expectedChunkCount ||
            header.totalBytes != expectedTotalBytes) {
            return false;
        }

        if (header.chunkIndex >= expectedChunkCount) return false;
        if (header.chunkBytes == 0 || header.chunkBytes > chunkCapacity) return false;

        const size_t byteOffset = static_cast<size_t>(header.chunkIndex) * static_cast<size_t>(chunkCapacity);
        if (byteOffset + header.chunkBytes > imageBytes.size()) return false;

        const size_t payloadStart = kImageChunkHeaderBits;
        const size_t payloadBits = static_cast<size_t>(header.chunkBytes) * 8;
        if (decoded.size() < payloadStart + payloadBits) return false;

        VecInt chunkBits(
            decoded.begin() + static_cast<std::ptrdiff_t>(payloadStart),
            decoded.begin() + static_cast<std::ptrdiff_t>(payloadStart + payloadBits));
        std::vector<uint8_t> bytes = OFDMUtils::bitsToBytes(chunkBits);
        if (bytes.size() != header.chunkBytes) return false;

        std::copy(bytes.begin(), bytes.end(), imageBytes.begin() + static_cast<std::ptrdiff_t>(byteOffset));
        receivedChunks[header.chunkIndex] = true;

        bool allReceived = std::all_of(receivedChunks.begin(), receivedChunks.end(), [](bool v) { return v; });
        if (allReceived) {
            outImg.width = expectedWidth;
            outImg.height = expectedHeight;
            outImg.pixels = std::move(imageBytes);
            return true;
        }

        cursor = frameStartPos + frameSpan;
    }

    return false;
}
"@
if (-not $content.Contains($old5)) { throw 'old5 not found' }
$content = $content.Replace($old5, $new5)

Set-Content -Path $path -Value $content -Encoding UTF8
