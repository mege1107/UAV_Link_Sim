#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

#include "sim_runner.h"
#include "role_runner.h"

namespace {

enum class EntryMode {
    UnifiedLinkTest,
    RoleTxOnly,
    RoleRxOnly,
    RoleRxFile
};

enum class TestChain {
    SingleCarrierRandomBits,
    SingleCarrierFileTransfer,
    OfdmRandomBits,
    OfdmFileTransfer
};

const char* test_chain_to_string(TestChain chain)
{
    switch (chain) {
    case TestChain::SingleCarrierRandomBits: return "SingleCarrier RandomBits";
    case TestChain::SingleCarrierFileTransfer: return "SingleCarrier FileTransfer";
    case TestChain::OfdmRandomBits:          return "OFDM RandomBits";
    case TestChain::OfdmFileTransfer:        return "OFDM FileTransfer";
    default:                                 return "Unknown";
    }
}

const char* entry_mode_to_string(EntryMode mode)
{
    switch (mode) {
    case EntryMode::UnifiedLinkTest: return "UnifiedLinkTest";
    case EntryMode::RoleTxOnly:      return "RoleTxOnly";
    case EntryMode::RoleRxOnly:      return "RoleRxOnly";
    case EntryMode::RoleRxFile:      return "RoleRxFile";
    default:                         return "Unknown";
    }
}

void print_result_summary(const TestResult& tr)
{
    std::cout << std::scientific << std::setprecision(6)
        << "BER=" << tr.total_ber
        << std::defaultfloat
        << " | bit_errors=" << tr.total_bit_errors
        << " | compared_bits=" << tr.total_compared_bits
        << " | decoded_frames=" << tr.decoded_frames
        << "\n";

    if (!tr.saved_file_path.empty()) {
        std::cout << "file_saved=" << (tr.file_saved ? 1 : 0)
            << " | saved_path=" << tr.saved_file_path << "\n";
    }
}

void print_tx_role_summary(const TxOnlyResult& tr)
{
    std::cout << "tx_frames=" << tr.tx_frame_count
        << " | tx_samples=" << tr.tx_sample_count
        << " | fs=" << tr.fs
        << "\n";
}

void print_rx_role_summary(const RxOnlyResult& tr)
{
    std::cout << std::scientific << std::setprecision(6)
        << "BER=" << tr.total_ber
        << std::defaultfloat
        << " | bit_errors=" << tr.total_bit_errors
        << " | compared_bits=" << tr.total_compared_bits
        << " | decoded_frames=" << tr.decoded_frames
        << " | decoded_bits=" << tr.decoded_bits_count
        << " | rx_samples=" << tr.rx_sample_count
        << " | fs=" << tr.fs
        << "\n";
}

void print_rx_file_role_summary(const RxFileResult& tr)
{
    std::cout << "decoded_frames=" << tr.decoded_frames
        << " | decoded_bits=" << tr.decoded_bits_count
        << " | rx_samples=" << tr.rx_sample_count
        << " | fs=" << tr.fs
        << "\n";

    if (!tr.saved_file_path.empty()) {
        std::cout << "file_saved=" << (tr.file_saved ? 1 : 0)
            << " | recovered_filename=" << tr.recovered_filename
            << " | saved_path=" << tr.saved_file_path
            << "\n";
    }
}

} // namespace

int main()
{
    try
    {
        // =========================
        // Unified test entry config
        // =========================
        constexpr EntryMode kEntryMode = EntryMode::RoleTxOnly;
        constexpr TestChain kChain = TestChain::SingleCarrierRandomBits;
        constexpr RunMode kMode = RunMode::USRP;   // AWGN / USRP / LOOPBACK
        constexpr bool kPrintLogText = true;

        constexpr double kAwgnSnrDb = 20.0;
        constexpr double kCenterFreqHz = 433e6;
        const std::string kUsrpDeviceArgs = "type=b200";
        const std::string kRxUsrpDeviceArgs = "type=b200";
        const std::string kTxUsrpDeviceArgs = "type=b200";

        // Single-carrier parameters
        constexpr ModulationType kSingleCarrierModulation = ModulationType::FSK;
        constexpr int kTxRepeatFrames = 5000;
        constexpr double kInfoRateBps = 64000.0;
        constexpr int kHopPattern = 0;
        constexpr SourceMode kRoleSourceMode = SourceMode::RandomBits;
        constexpr int kRoleExpectedFrames = 120;
        constexpr int kRoleMaxFrames = 200;

        // File paths
        const std::string kInputFilePath =  "D:\\linksim\\test_files\\test.txt";
        const std::string kOutputFilePath = "D:\\linksim\\test_files\\out_main.txt";

        // OFDM channel impairments used in AWGN mode
        ChannelConfig ofdmChannelCfg;
        ofdmChannelCfg.enable_awgn = true;
        ofdmChannelCfg.snr_dB = kAwgnSnrDb;
        ofdmChannelCfg.seed = 123;
        ofdmChannelCfg.enable_sto = false;
        ofdmChannelCfg.sto_samp = 10000;
        ofdmChannelCfg.enable_cfo = false;
        ofdmChannelCfg.cfo_hz = 6000.0;
        ofdmChannelCfg.enable_sfo = false;
        ofdmChannelCfg.sfo_ppm = 10.0;

        std::cout << "============================\n";
        std::cout << "  UNIFIED LINK TEST ENTRY\n";
        std::cout << "============================\n";
        std::cout << "entry_mode=" << entry_mode_to_string(kEntryMode)
            << " | center_freq_hz=" << kCenterFreqHz
            << " | modulation=" << modulation_to_string(kSingleCarrierModulation)
            << " | info_rate_bps=" << kInfoRateBps
            << " | hop_pattern=" << kHopPattern
            << "\n";

        switch (kEntryMode) {
        case EntryMode::UnifiedLinkTest:
        {
            TestResult tr;

            switch (kChain) {
            case TestChain::SingleCarrierRandomBits:
                tr = run_one_test(
                    kMode,
                    kAwgnSnrDb,
                    kTxRepeatFrames,
                    kCenterFreqHz,
                    kSingleCarrierModulation,
                    kInfoRateBps,
                    kHopPattern);
                break;

            case TestChain::SingleCarrierFileTransfer:
                tr = run_file_transfer_test(
                    kMode,
                    kAwgnSnrDb,
                    kCenterFreqHz,
                    kSingleCarrierModulation,
                    kInfoRateBps,
                    kHopPattern,
                    kInputFilePath,
                    kOutputFilePath);
                break;

            case TestChain::OfdmRandomBits:
                tr = run_ofdm_random_bit_test(
                    kMode,
                    kAwgnSnrDb,
                    kCenterFreqHz,
                    kUsrpDeviceArgs,
                    ofdmChannelCfg);
                break;

            case TestChain::OfdmFileTransfer:
                tr = run_ofdm_file_transfer_test(
                    kMode,
                    kAwgnSnrDb,
                    kCenterFreqHz,
                    kInputFilePath,
                    kOutputFilePath,
                    kUsrpDeviceArgs,
                    ofdmChannelCfg);
                break;

            default:
                throw std::runtime_error("Unsupported test chain");
            }

            std::cout << "chain=" << test_chain_to_string(kChain)
                << " | mode=" << mode_to_string(kMode)
                << " | snr_dB=" << std::fixed << std::setprecision(3) << kAwgnSnrDb
                << "\n";

            if (kChain == TestChain::SingleCarrierFileTransfer ||
                kChain == TestChain::OfdmFileTransfer) {
                std::cout << "input_file=" << kInputFilePath
                    << " | output_file=" << kOutputFilePath
                    << "\n";
            }

            if (kChain == TestChain::OfdmRandomBits ||
                kChain == TestChain::OfdmFileTransfer) {
                std::cout << "ofdm_sto=" << ofdmChannelCfg.sto_samp
                    << " | ofdm_cfo_hz=" << ofdmChannelCfg.cfo_hz
                    << " | ofdm_sfo_ppm=" << ofdmChannelCfg.sfo_ppm
                    << "\n";
            }

            print_result_summary(tr);

            if (kPrintLogText && !tr.log_text.empty()) {
                std::cout << "\n----- LOG TEXT -----\n";
                std::cout << tr.log_text;
            }
            break;
        }

        case EntryMode::RoleTxOnly:
        {
            std::cout << "role=TX"
                << " | tx_device_args=" << kTxUsrpDeviceArgs
                << " | source_mode=" << (kRoleSourceMode == SourceMode::FileBits ? "FileBits" : "RandomBits")
                << " | tx_repeat_frames=" << kTxRepeatFrames
                << "\n";

            if (kRoleSourceMode == SourceMode::FileBits) {
                std::cout << "input_file=" << kInputFilePath << "\n";
            }

            TxOnlyResult tr = run_tx_role_once(
                kTxUsrpDeviceArgs,
                kCenterFreqHz,
                kSingleCarrierModulation,
                kInfoRateBps,
                kHopPattern,
                kRoleSourceMode,
                kInputFilePath,
                kTxRepeatFrames);

            print_tx_role_summary(tr);

            if (kPrintLogText && !tr.log_text.empty()) {
                std::cout << "\n----- LOG TEXT -----\n";
                std::cout << tr.log_text;
            }
            break;
        }

        case EntryMode::RoleRxOnly:
        {
            std::cout << "role=RX"
                << " | rx_device_args=" << kRxUsrpDeviceArgs
                << " | expected_frames=" << kRoleExpectedFrames
                << "\n";

            RxOnlyResult tr = run_rx_role_once(
                kRxUsrpDeviceArgs,
                kCenterFreqHz,
                kSingleCarrierModulation,
                kInfoRateBps,
                kHopPattern,
                kRoleExpectedFrames);

            print_rx_role_summary(tr);

            if (kPrintLogText && !tr.log_text.empty()) {
                std::cout << "\n----- LOG TEXT -----\n";
                std::cout << tr.log_text;
            }
            break;
        }

        case EntryMode::RoleRxFile:
        {
            std::cout << "role=RX_FILE"
                << " | rx_device_args=" << kRxUsrpDeviceArgs
                << " | max_frames=" << kRoleMaxFrames
                << " | output_file=" << kOutputFilePath
                << "\n";

            RxFileResult tr = run_rx_file_role_once(
                kRxUsrpDeviceArgs,
                kCenterFreqHz,
                kSingleCarrierModulation,
                kInfoRateBps,
                kHopPattern,
                kOutputFilePath,
                kRoleMaxFrames);

            print_rx_file_role_summary(tr);

            if (kPrintLogText && !tr.log_text.empty()) {
                std::cout << "\n----- LOG TEXT -----\n";
                std::cout << tr.log_text;
            }
            break;
        }

        default:
            throw std::runtime_error("Unsupported entry mode");
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
