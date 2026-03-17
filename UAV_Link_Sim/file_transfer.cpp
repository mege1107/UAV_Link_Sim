#include "file_transfer.h"
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <filesystem>

static void append_u16(ByteVec& out, uint16_t v)
{
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(v & 0xFF));
}

static void append_u64(ByteVec& out, uint64_t v)
{
    for (int i = 7; i >= 0; --i) {
        out.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
    }
}

static uint16_t read_u16(const ByteVec& in, size_t pos)
{
    return (static_cast<uint16_t>(in[pos]) << 8) |
        (static_cast<uint16_t>(in[pos + 1]));
}

static uint64_t read_u64(const ByteVec& in, size_t pos)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) {
        v = (v << 8) | static_cast<uint64_t>(in[pos + i]);
    }
    return v;
}

static std::string to_lower_str(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}

FileType detect_file_type(const std::string& path)
{
    std::filesystem::path p(path);
    std::string ext = to_lower_str(p.extension().string());

    if (ext == ".txt") return FileType::TXT;
    if (ext == ".jpg" || ext == ".jpeg") return FileType::JPG;
    if (ext == ".mp4") return FileType::MP4;
    return FileType::OTHER;
}

ByteVec read_binary_file(const std::string& path)
{
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {
        throw std::runtime_error("Cannot open file for reading: " + path);
    }

    ifs.seekg(0, std::ios::end);
    std::streamsize sz = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    if (sz < 0) {
        throw std::runtime_error("Invalid file size: " + path);
    }

    ByteVec data(static_cast<size_t>(sz));
    if (sz > 0) {
        ifs.read(reinterpret_cast<char*>(data.data()), sz);
    }
    return data;
}

void write_binary_file(const std::string& path, const ByteVec& data)
{
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) {
        throw std::runtime_error("Cannot open file for writing: " + path);
    }

    if (!data.empty()) {
        ofs.write(reinterpret_cast<const char*>(data.data()),
            static_cast<std::streamsize>(data.size()));
    }
}

VecInt bytes_to_bits(const ByteVec& bytes)
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

ByteVec bits_to_bytes(const VecInt& bits)
{
    ByteVec bytes;
    if (bits.empty()) return bytes;

    const size_t nbytes = bits.size() / 8;
    bytes.reserve(nbytes);

    for (size_t i = 0; i < nbytes; ++i) {
        uint8_t b = 0;
        for (int j = 0; j < 8; ++j) {
            b = static_cast<uint8_t>((b << 1) | (bits[i * 8 + j] & 1));
        }
        bytes.push_back(b);
    }

    return bytes;
}

ByteVec pack_file_packet(const FilePacket& pkt)
{
    ByteVec out;

    const std::string magic = "UAVF";
    out.insert(out.end(), magic.begin(), magic.end());

    out.push_back(1); // version
    out.push_back(static_cast<uint8_t>(pkt.file_type));

    if (pkt.filename.size() > 65535) {
        throw std::runtime_error("Filename too long");
    }

    append_u16(out, static_cast<uint16_t>(pkt.filename.size()));
    append_u64(out, static_cast<uint64_t>(pkt.payload.size()));

    out.insert(out.end(), pkt.filename.begin(), pkt.filename.end());
    out.insert(out.end(), pkt.payload.begin(), pkt.payload.end());

    return out;
}

bool unpack_file_packet(const ByteVec& bytes, FilePacket& pkt)
{
    if (bytes.size() < 16) return false;

    if (!(bytes[0] == 'U' && bytes[1] == 'A' && bytes[2] == 'V' && bytes[3] == 'F')) {
        return false;
    }

    uint8_t version = bytes[4];
    if (version != 1) return false;

    pkt.file_type = static_cast<FileType>(bytes[5]);
    uint16_t filename_len = read_u16(bytes, 6);
    uint64_t file_size = read_u64(bytes, 8);

    const size_t header_size = 16 + static_cast<size_t>(filename_len);
    if (bytes.size() < header_size) return false;
    if (bytes.size() < header_size + static_cast<size_t>(file_size)) return false;

    pkt.filename.assign(
        reinterpret_cast<const char*>(bytes.data() + 16),
        reinterpret_cast<const char*>(bytes.data() + 16 + filename_len)
    );

    pkt.payload.assign(
        bytes.begin() + static_cast<long long>(header_size),
        bytes.begin() + static_cast<long long>(header_size + static_cast<size_t>(file_size))
    );

    return true;
}

VecInt file_to_bits_with_header(const std::string& filepath)
{
    std::filesystem::path p(filepath);

    FilePacket pkt;
    pkt.filename = p.filename().string();
    pkt.file_type = detect_file_type(filepath);
    pkt.payload = read_binary_file(filepath);

    ByteVec packed = pack_file_packet(pkt);
    return bytes_to_bits(packed);
}

bool bits_to_file_from_header(const VecInt& bits, const std::string& output_dir, std::string& saved_path)
{
    ByteVec bytes = bits_to_bytes(bits);

    FilePacket pkt;
    if (!unpack_file_packet(bytes, pkt)) {
        return false;
    }

    std::filesystem::create_directories(output_dir);
    std::filesystem::path out = std::filesystem::path(output_dir) / pkt.filename;

    write_binary_file(out.string(), pkt.payload);
    saved_path = out.string();
    return true;
}