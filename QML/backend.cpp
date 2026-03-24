#include "backend.h"
#include "sim_runner.h"

#include <QtConcurrent/QtConcurrent>
#include <QFileInfo>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QCoreApplication>
#include <QStringConverter>
#include <algorithm>
#include <exception>
#include <stdexcept>

Backend::Backend(QObject *parent)
    : QObject(parent),
    m_logText("绯荤粺灏辩华銆俓n"),
    m_berText("N/A"),
    m_selectedMode("MSK"),
    m_centerFreq("400"),
    m_bandwidth("1"),
    m_infoRate("4"),
    m_hopPattern("1"),
    m_inputText(""),
    m_inputFilePath(""),
    m_transferSource("RANDOM")
{
    connect(&m_simWatcher, &QFutureWatcher<TestResult>::finished,
            this, &Backend::handleSimulationFinished);
}

QString Backend::logText() const
{
    return m_logText;
}

QString Backend::berText() const
{
    return m_berText;
}

QVariantList Backend::waveform() const
{
    return m_waveform;
}

double Backend::waveformMin() const
{
    return m_waveformMin;
}

double Backend::waveformMax() const
{
    return m_waveformMax;
}

QVariantList Backend::spectrumFreq() const
{
    return m_spectrumFreq;
}

QVariantList Backend::spectrumMag() const
{
    return m_spectrumMag;
}

QVariantList Backend::spectrogramData() const
{
    return m_spectrogramData;
}

int Backend::spectrogramWidth() const
{
    return m_spectrogramWidth;
}

int Backend::spectrogramHeight() const
{
    return m_spectrogramHeight;
}

double Backend::spectrogramTimeSpan() const
{
    return m_spectrogramTimeSpan;
}

double Backend::spectrogramFreqMin() const
{
    return m_spectrogramFreqMin;
}

double Backend::spectrogramFreqMax() const
{
    return m_spectrogramFreqMax;
}

bool Backend::usrpConnected() const
{
    return m_usrpConnected;
}

bool Backend::sending() const
{
    return m_sending;
}

QString Backend::selectedMode() const
{
    return m_selectedMode;
}

void Backend::setSelectedMode(const QString &value)
{
    if (m_selectedMode == value) return;
    m_selectedMode = value;
    emit selectedModeChanged();
}

QString Backend::centerFreq() const
{
    return m_centerFreq;
}

void Backend::setCenterFreq(const QString &value)
{
    if (m_centerFreq == value) return;
    m_centerFreq = value;
    emit centerFreqChanged();
}

QString Backend::bandwidth() const
{
    return m_bandwidth;
}

void Backend::setBandwidth(const QString &value)
{
    if (m_bandwidth == value) return;
    m_bandwidth = value;
    emit bandwidthChanged();
}

QString Backend::infoRate() const
{
    return m_infoRate;
}

void Backend::setInfoRate(const QString &value)
{
    if (m_infoRate == value) return;
    m_infoRate = value;
    emit infoRateChanged();
}

QString Backend::hopPattern() const
{
    return m_hopPattern;
}

void Backend::setHopPattern(const QString &value)
{
    if (m_hopPattern == value) return;
    m_hopPattern = value;
    emit hopPatternChanged();
}

QString Backend::inputText() const
{
    return m_inputText;
}

void Backend::setInputText(const QString &value)
{
    if (m_inputText == value) return;
    m_inputText = value;
    emit inputTextChanged();
}

QString Backend::inputFilePath() const
{
    return m_inputFilePath;
}

void Backend::setInputFilePath(const QString &value)
{
    QString fixed = value;

    if (fixed.startsWith("file:///"))
        fixed = QDir::fromNativeSeparators(fixed.mid(8));
    else if (fixed.startsWith("file://"))
        fixed = QDir::fromNativeSeparators(fixed.mid(7));

    if (m_inputFilePath == fixed) return;
    m_inputFilePath = fixed;
    emit inputFilePathChanged();
}

QString Backend::transferSource() const
{
    return m_transferSource;
}

void Backend::setTransferSource(const QString &source)
{
    QString s = source.trimmed().toUpper();
    if (s != "RANDOM" && s != "TEXT" && s != "FILE")
        return;

    if (m_transferSource == s)
        return;

    m_transferSource = s;
    emit transferSourceChanged();

    QString showName = "闅忔満姣旂壒";
    if (m_transferSource == "TEXT") showName = "瀛楃";
    if (m_transferSource == "FILE") showName = "鏂囦欢";

    appendLog(QString("褰撳墠淇℃伅婧愬垏鎹负锛?1").arg(showName));
}

void Backend::appendLog(const QString &msg)
{
    m_logText += msg + "\n";
    emit logTextChanged();
}

bool Backend::payloadReady() const
{
    if (m_transferSource == "TEXT")
        return !m_inputText.trimmed().isEmpty();

    if (m_transferSource == "FILE")
        return !m_inputFilePath.trimmed().isEmpty();

    return true;
}

QString Backend::currentPayloadDescription() const
{
    if (m_transferSource == "TEXT") {
        QString text = m_inputText;
        if (text.size() > 40)
            text = text.left(40) + "...";
        return QString("鏂囨湰[%1]").arg(text);
    }

    if (m_transferSource == "FILE") {
        QFileInfo fi(m_inputFilePath);
        return QString("鏂囦欢[%1]").arg(fi.fileName());
    }

    return "闅忔満姣旂壒";
}

QString Backend::ensureTempTextFile()
{
    QDir baseDir(QCoreApplication::applicationDirPath());
    if (!baseDir.exists("temp")) {
        baseDir.mkpath("temp");
    }

    const QString filePath = baseDir.filePath("temp/input_text_payload.txt");

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        throw std::runtime_error(("Cannot create temp text file: " + filePath).toStdString());
    }

    QTextStream out(&file);
    out.setEncoding(QStringConverter::Utf8);
    out << m_inputText;
    file.close();

    m_tempTextFilePath = filePath;
    return m_tempTextFilePath;
}

QString Backend::buildRecoveredOutputPath(const QString &srcPath) const
{
    QFileInfo fi(srcPath);
    QString dir = fi.absolutePath();
    QString base = fi.completeBaseName();
    QString suffix = fi.suffix();

    QString recoveredName = base + "_rx";
    if (!suffix.isEmpty())
        recoveredName += "." + suffix;

    return QDir(dir).filePath(recoveredName);
}

void Backend::applyParameterSettings()
{
    appendLog(QString("鍙傛暟宸叉洿鏂帮細璋冨埗=%1, 涓績棰戠巼=%2 MHz, 甯﹀=%3 MHz, 淇℃伅閫熺巼=%4 Kbps, 璺抽鍥炬=%5")
                  .arg(m_selectedMode, m_centerFreq, m_bandwidth, m_infoRate, m_hopPattern));
}

void Backend::connectUsrp(bool on)
{
    if (m_usrpConnected == on)
        return;

    m_usrpConnected = on;
    emit usrpConnectedChanged();

    appendLog(on ? "USRP B210 宸茶繛鎺ワ紝鍚庣画浠诲姟灏嗚蛋 USRP 妯″紡"
                 : "USRP B210 宸叉柇寮€锛屽悗缁换鍔″皢璧?LOOPBACK 妯″紡");
}

void Backend::sendText()
{
    setTransferSource("TEXT");

    if (m_inputText.trimmed().isEmpty()) {
        appendLog("鏂囨湰瑁呰浇澶辫触锛氳緭鍏ヤ负绌恒€?);
        return;
    }

    appendLog(QString("鏂囨湰宸茶杞斤紝闀垮害=%1 瀛楃").arg(m_inputText.size()));
}

void Backend::browseInputFile()
{
    appendLog("QML FileDialog 灏嗚礋璐ｉ€夋枃浠讹紝褰撳墠鎺ュ彛淇濈暀銆?);
}

void Backend::sendFile()
{
    setTransferSource("FILE");

    if (m_inputFilePath.trimmed().isEmpty()) {
        appendLog("鏂囦欢瑁呰浇澶辫触锛氬皻鏈€夋嫨鏂囦欢銆?);
        return;
    }

    QFileInfo fi(m_inputFilePath);
    appendLog(QString("鏂囦欢宸茶杞斤細%1").arg(fi.fileName()));
    appendLog(QString("瀹屾暣璺緞锛?1").arg(m_inputFilePath));
}

void Backend::startSending(bool on)
{
    if (!on) {
        if (m_sending) {
            m_sending = false;
            emit sendingChanged();
            appendLog("鍙戦€佸凡鍋滄銆?);
        }
        return;
    }

    if (!payloadReady()) {
        appendLog("鍙戦€佸け璐ワ細褰撳墠娌℃湁鍙彂閫佺殑鏁版嵁銆?);
        if (m_transferSource == "TEXT")
            appendLog("璇峰厛杈撳叆鏂囨湰銆?);
        else if (m_transferSource == "FILE")
            appendLog("璇峰厛閫夋嫨鏂囦欢銆?);
        else
            appendLog("褰撳墠鏁版嵁婧愬紓甯搞€?);
        return;
    }

    if (m_isRunning) {
        appendLog("绯荤粺蹇欙細褰撳墠浠诲姟灏氭湭缁撴潫銆?);
        return;
    }

    m_sending = true;
    emit sendingChanged();

    runCurrentTask(true);
}

void Backend::runSimulation()
{
    if (m_isRunning) {
        appendLog("浠跨湡姝ｅ湪杩愯锛岃鍕块噸澶嶇偣鍑汇€?);
        return;
    }

    m_isRunning = true;

    const double centerFreqHz = m_centerFreq.toDouble() * 1e6;
    const double infoRateBps = m_infoRate.toDouble() * 1e3;
    const int hopPattern = m_hopPattern.toInt();

    const std::string modStr = m_selectedMode.toStdString();
    const ModulationType modulation = parse_modulation(modStr);
    const RunMode runMode = m_usrpConnected ? RunMode::USRP : RunMode::LOOPBACK;

    appendLog(QString("寮€濮嬭繍琛岄殢鏈烘瘮鐗?BER 娴嬭瘯锛氭ā寮?%1, 璋冨埗=%2, Fc=%3 MHz, 淇℃伅閫熺巼=%4 Kbps, 璺抽鍥炬=%5")
                  .arg(m_usrpConnected ? "USRP" : "LOOPBACK",
                       m_selectedMode,
                       m_centerFreq,
                       m_infoRate,
                       m_hopPattern));

    QtConcurrent::run([this, centerFreqHz, infoRateBps, hopPattern, modulation, runMode]() {
        try {
            TestResult tr = run_one_test(
                runMode,
                30.0,
                1,
                centerFreqHz,
                modulation,
                infoRateBps,
                hopPattern
                );

            QMetaObject::invokeMethod(this, [this, tr]() {
                applyTestResult(tr);
            }, Qt::QueuedConnection);
        }
        catch (const std::exception &e) {
            const QString err = QString("浠诲姟澶辫触: %1").arg(e.what());
            QMetaObject::invokeMethod(this, [this, err]() {
                m_isRunning = false;
                appendLog(err);
                if (m_sending) {
                    m_sending = false;
                    emit sendingChanged();
                }
            }, Qt::QueuedConnection);
        }
        catch (...) {
            QMetaObject::invokeMethod(this, [this]() {
                m_isRunning = false;
                appendLog("浠诲姟澶辫触: Unknown exception");
                if (m_sending) {
                    m_sending = false;
                    emit sendingChanged();
                }
            }, Qt::QueuedConnection);
        }
    });
}

void Backend::runCurrentTask(bool fromSendingAction)
{
    if (m_isRunning) {
        appendLog("浠诲姟姝ｅ湪杩愯锛岃鍕块噸澶嶈Е鍙戙€?);
        return;
    }

    if (!payloadReady()) {
        appendLog("浠诲姟鍚姩澶辫触锛氬綋鍓嶆暟鎹湭鍑嗗濂姐€?);
        return;
    }

    m_isRunning = true;

    const double centerFreqHz = m_centerFreq.toDouble() * 1e6;
    const double infoRateBps = m_infoRate.toDouble() * 1e3;
    const int hopPattern = m_hopPattern.toInt();

    const std::string modStr = m_selectedMode.toStdString();
    const ModulationType modulation = parse_modulation(modStr);
    const RunMode runMode = m_usrpConnected ? RunMode::USRP : RunMode::LOOPBACK;

    const QString actionName = fromSendingAction ? "寮€濮嬪彂閫? : "寮€濮嬭繍琛屼换鍔?;

    appendLog(QString("%1锛氭ā寮?%2, 璋冨埗=%3, Fc=%4 MHz, 淇℃伅閫熺巼=%5 Kbps, 璺抽鍥炬=%6, 鏁版嵁婧?%7")
                  .arg(actionName)
                  .arg(m_usrpConnected ? "USRP" : "LOOPBACK")
                  .arg(m_selectedMode)
                  .arg(m_centerFreq)
                  .arg(m_infoRate)
                  .arg(m_hopPattern)
                  .arg(currentPayloadDescription()));

    if (m_transferSource == "RANDOM")
    {
        QtConcurrent::run([this, centerFreqHz, infoRateBps, hopPattern, modulation, runMode]() {
            try {
                TestResult tr = run_one_test(
                    runMode,
                    30.0,
                    1,
                    centerFreqHz,
                    modulation,
                    infoRateBps,
                    hopPattern
                    );

                QMetaObject::invokeMethod(this, [this, tr]() {
                    applyTestResult(tr);
                }, Qt::QueuedConnection);
            }
            catch (const std::exception &e) {
                const QString err = QString("浠诲姟澶辫触: %1").arg(e.what());
                QMetaObject::invokeMethod(this, [this, err]() {
                    m_isRunning = false;
                    appendLog(err);
                    if (m_sending) {
                        m_sending = false;
                        emit sendingChanged();
                    }
                }, Qt::QueuedConnection);
            }
            catch (...) {
                QMetaObject::invokeMethod(this, [this]() {
                    m_isRunning = false;
                    appendLog("浠诲姟澶辫触: Unknown exception");
                    if (m_sending) {
                        m_sending = false;
                        emit sendingChanged();
                    }
                }, Qt::QueuedConnection);
            }
        });

        return;
    }

    QString inputPath;
    if (m_transferSource == "TEXT") {
        inputPath = ensureTempTextFile();
        appendLog(QString("鏂囨湰宸插啓鍏ヤ复鏃舵枃浠讹細%1").arg(inputPath));
    } else {
        inputPath = m_inputFilePath;
    }

    const QString outputPath = buildRecoveredOutputPath(inputPath);

    appendLog(QString("鎺ユ敹鎭㈠鏂囦欢灏嗕繚瀛樺埌锛?1").arg(outputPath));

    QtConcurrent::run([this, centerFreqHz, infoRateBps, hopPattern, modulation, runMode, inputPath, outputPath]() {
        try {
            TestResult tr = run_file_transfer_test(
                runMode,
                30.0,
                centerFreqHz,
                modulation,
                infoRateBps,
                hopPattern,
                inputPath.toStdString(),
                outputPath.toStdString()
                );

            QMetaObject::invokeMethod(this, [this, tr]() {
                applyTestResult(tr);
            }, Qt::QueuedConnection);
        }
        catch (const std::exception &e) {
            const QString err = QString("浠诲姟澶辫触: %1").arg(e.what());
            QMetaObject::invokeMethod(this, [this, err]() {
                m_isRunning = false;
                appendLog(err);
                if (m_sending) {
                    m_sending = false;
                    emit sendingChanged();
                }
            }, Qt::QueuedConnection);
        }
        catch (...) {
            QMetaObject::invokeMethod(this, [this]() {
                m_isRunning = false;
                appendLog("浠诲姟澶辫触: Unknown exception");
                if (m_sending) {
                    m_sending = false;
                    emit sendingChanged();
                }
            }, Qt::QueuedConnection);
        }
    });
}

void Backend::handleSimulationFinished()
{
    m_isRunning = false;

    try {
        const TestResult tr = m_pendingResult;

        m_berText = QString::number(tr.total_ber, 'g', 6);
        emit berTextChanged();

        m_waveform.clear();

        const int maxSamples = 500;
        const int n = static_cast<int>(tr.waveform.size());
        const int showN = std::min(maxSamples, n);

        double vmin = 1e100;
        double vmax = -1e100;

        for (int i = 0; i < showN; ++i) {
            const double v = tr.waveform[i];
            m_waveform.append(v);

            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }

        if (showN == 0) {
            m_waveformMin = -1.0;
            m_waveformMax = 1.0;
        } else {
            m_waveformMin = vmin;
            m_waveformMax = vmax;

            if (m_waveformMax - m_waveformMin < 1e-12) {
                m_waveformMin -= 1.0;
                m_waveformMax += 1.0;
            }
        }

        emit waveformChanged();

        m_spectrumFreq.clear();
        m_spectrumMag.clear();

        const int specN = std::min(
            static_cast<int>(tr.spectrum_freq.size()),
            static_cast<int>(tr.spectrum_mag.size())
            );

        for (int i = 0; i < specN; ++i) {
            m_spectrumFreq.append(tr.spectrum_freq[i]);
            m_spectrumMag.append(tr.spectrum_mag[i]);
        }

        m_rxWaveform.clear();

        const int rxMaxSamples = 500;
        const int rxN = static_cast<int>(tr.rx_waveform.size());
        const int rxShowN = std::min(rxMaxSamples, rxN);

        double rxVmin = 1e100;
        double rxVmax = -1e100;

        for (int i = 0; i < rxShowN; ++i) {
            const double v = tr.rx_waveform[i];
            m_rxWaveform.append(v);

            if (v < rxVmin) rxVmin = v;
            if (v > rxVmax) rxVmax = v;
        }

        if (rxShowN == 0) {
            m_rxWaveformMin = -1.0;
            m_rxWaveformMax = 1.0;
        } else {
            m_rxWaveformMin = rxVmin;
            m_rxWaveformMax = rxVmax;

            if (m_rxWaveformMax - m_rxWaveformMin < 1e-12) {
                m_rxWaveformMin -= 1.0;
                m_rxWaveformMax += 1.0;
            }
        }

        emit rxWaveformChanged();

        m_rxSpectrumFreq.clear();
        m_rxSpectrumMag.clear();

        const int rxSpecN = std::min(
            static_cast<int>(tr.rx_spectrum_freq.size()),
            static_cast<int>(tr.rx_spectrum_mag.size())
            );

        for (int i = 0; i < rxSpecN; ++i) {
            m_rxSpectrumFreq.append(tr.rx_spectrum_freq[i]);
            m_rxSpectrumMag.append(tr.rx_spectrum_mag[i]);
        }

        emit rxSpectrumChanged();
        emit spectrumChanged();

        m_spectrogramData.clear();
        m_spectrogramWidth = tr.spectrogram_width;
        m_spectrogramHeight = tr.spectrogram_height;
        m_spectrogramTimeSpan = tr.spectrogram_time_span;
        m_spectrogramFreqMin = tr.spectrogram_freq_min;
        m_spectrogramFreqMax = tr.spectrogram_freq_max;

        const int spec2dN = static_cast<int>(tr.spectrogram_data.size());
        for (int i = 0; i < spec2dN; ++i) {
            m_spectrogramData.append(tr.spectrogram_data[i]);
        }

        emit spectrogramChanged();

        m_constellationI.clear();
        m_constellationQ.clear();

        const int constN = std::min(
            static_cast<int>(tr.constellation_i.size()),
            static_cast<int>(tr.constellation_q.size())
            );

        for (int i = 0; i < constN; ++i) {
            m_constellationI.append(tr.constellation_i[i]);
            m_constellationQ.append(tr.constellation_q[i]);
        }

        emit constellationChanged();

        appendLog(QString::fromStdString(tr.log_text));

        if (tr.file_saved) {
            appendLog(QString("鏂囦欢鎭㈠鎴愬姛锛?1").arg(QString::fromStdString(tr.saved_file_path)));
        }

        appendLog(QString("浠诲姟瀹屾垚锛孊ER=%1").arg(m_berText));
    }
    catch (const std::exception &e) {
        appendLog(QString("浠诲姟澶辫触: %1").arg(e.what()));
    }
    catch (...) {
        appendLog("浠诲姟澶辫触: Unknown exception");
    }

    if (m_sending) {
        m_sending = false;
        emit sendingChanged();
        appendLog("鏈鍙戦€佹祦绋嬬粨鏉熴€?);
    }
}

QVariantList Backend::rxWaveform() const
{
    return m_rxWaveform;
}

double Backend::rxWaveformMin() const
{
    return m_rxWaveformMin;
}

double Backend::rxWaveformMax() const
{
    return m_rxWaveformMax;
}

QVariantList Backend::rxSpectrumFreq() const
{
    return m_rxSpectrumFreq;
}

QVariantList Backend::rxSpectrumMag() const
{
    return m_rxSpectrumMag;
}

QVariantList Backend::constellationI() const
{
    return m_constellationI;
}

QVariantList Backend::constellationQ() const
{
    return m_constellationQ;
}

void Backend::applyTestResult(const TestResult &tr)
{
    m_berText = QString::number(tr.total_ber, 'g', 6);
    emit berTextChanged();

    m_waveform.clear();

    const int maxSamples = 500;
    const int n = static_cast<int>(tr.waveform.size());
    const int showN = std::min(maxSamples, n);

    double vmin = 1e100;
    double vmax = -1e100;

    for (int i = 0; i < showN; ++i) {
        const double v = tr.waveform[i];
        m_waveform.append(v);

        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }

    if (showN == 0) {
        m_waveformMin = -1.0;
        m_waveformMax = 1.0;
    } else {
        m_waveformMin = vmin;
        m_waveformMax = vmax;

        if (m_waveformMax - m_waveformMin < 1e-12) {
            m_waveformMin -= 1.0;
            m_waveformMax += 1.0;
        }
    }

    emit waveformChanged();

    m_spectrumFreq.clear();
    m_spectrumMag.clear();

    const int specN = std::min(
        static_cast<int>(tr.spectrum_freq.size()),
        static_cast<int>(tr.spectrum_mag.size())
        );

    for (int i = 0; i < specN; ++i) {
        m_spectrumFreq.append(tr.spectrum_freq[i]);
        m_spectrumMag.append(tr.spectrum_mag[i]);
    }

    emit spectrumChanged();

    m_rxWaveform.clear();

    const int rxMaxSamples = 500;
    const int rxN = static_cast<int>(tr.rx_waveform.size());
    const int rxShowN = std::min(rxMaxSamples, rxN);

    double rxVmin = 1e100;
    double rxVmax = -1e100;

    for (int i = 0; i < rxShowN; ++i) {
        const double v = tr.rx_waveform[i];
        m_rxWaveform.append(v);

        if (v < rxVmin) rxVmin = v;
        if (v > rxVmax) rxVmax = v;
    }

    if (rxShowN == 0) {
        m_rxWaveformMin = -1.0;
        m_rxWaveformMax = 1.0;
    } else {
        m_rxWaveformMin = rxVmin;
        m_rxWaveformMax = rxVmax;

        if (m_rxWaveformMax - m_rxWaveformMin < 1e-12) {
            m_rxWaveformMin -= 1.0;
            m_rxWaveformMax += 1.0;
        }
    }

    emit rxWaveformChanged();

    m_rxSpectrumFreq.clear();
    m_rxSpectrumMag.clear();

    const int rxSpecN = std::min(
        static_cast<int>(tr.rx_spectrum_freq.size()),
        static_cast<int>(tr.rx_spectrum_mag.size())
        );

    for (int i = 0; i < rxSpecN; ++i) {
        m_rxSpectrumFreq.append(tr.rx_spectrum_freq[i]);
        m_rxSpectrumMag.append(tr.rx_spectrum_mag[i]);
    }

    emit rxSpectrumChanged();

    m_spectrogramData.clear();
    m_spectrogramWidth = tr.spectrogram_width;
    m_spectrogramHeight = tr.spectrogram_height;
    m_spectrogramTimeSpan = tr.spectrogram_time_span;
    m_spectrogramFreqMin = tr.spectrogram_freq_min;
    m_spectrogramFreqMax = tr.spectrogram_freq_max;

    const int spec2dN = static_cast<int>(tr.spectrogram_data.size());
    for (int i = 0; i < spec2dN; ++i) {
        m_spectrogramData.append(tr.spectrogram_data[i]);
    }

    emit spectrogramChanged();

    m_constellationI.clear();
    m_constellationQ.clear();

    const int constN = std::min(
        static_cast<int>(tr.constellation_i.size()),
        static_cast<int>(tr.constellation_q.size())
        );

    for (int i = 0; i < constN; ++i) {
        m_constellationI.append(tr.constellation_i[i]);
        m_constellationQ.append(tr.constellation_q[i]);
    }

    emit constellationChanged();

    appendLog(QString::fromStdString(tr.log_text));

    if (tr.file_saved) {
        appendLog(QString("鏂囦欢鎭㈠鎴愬姛锛?1").arg(QString::fromStdString(tr.saved_file_path)));
    }

    appendLog(QString("浠诲姟瀹屾垚锛孊ER=%1").arg(m_berText));

    m_isRunning = false;

    if (m_sending) {
        m_sending = false;
        emit sendingChanged();
        appendLog("鏈鍙戦€佹祦绋嬬粨鏉熴€?);
    }
}
