#pragma once
#include <string>
#include <vector>
#include <cstdint>

using ByteVec = std::vector<uint8_t>;
using VecInt = std::vector<int>;

enum class FileType : uint8_t {
    TXT = 1,
    JPG = 2,
    MP4 = 3,
    OTHER = 255
};

struct FilePacket {
    std::string filename;
    FileType file_type = FileType::OTHER;
    ByteVec payload;
};

FileType detect_file_type(const std::string& path);

ByteVec read_binary_file(const std::string& path);
void write_binary_file(const std::string& path, const ByteVec& data);

VecInt bytes_to_bits(const ByteVec& bytes);
ByteVec bits_to_bytes(const VecInt& bits);

ByteVec pack_file_packet(const FilePacket& pkt);
bool unpack_file_packet(const ByteVec& bytes, FilePacket& pkt);

VecInt file_to_bits_with_header(const std::string& filepath);
bool bits_to_file_from_header(const VecInt& bits, const std::string& output_dir, std::string& saved_path);
