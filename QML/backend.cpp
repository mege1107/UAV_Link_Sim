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
    m_logText("系统就绪。\n"),
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

    QString showName = "随机比特";
    if (m_transferSource == "TEXT") showName = "字符";
    if (m_transferSource == "FILE") showName = "文件";

    appendLog(QString("当前信息源切换为：%1").arg(showName));
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
        return QString("文本[%1]").arg(text);
    }

    if (m_transferSource == "FILE") {
        QFileInfo fi(m_inputFilePath);
        return QString("文件[%1]").arg(fi.fileName());
    }

    return "随机比特";
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
    appendLog(QString("参数已更新：调制=%1, 中心频率=%2 MHz, 带宽=%3 MHz, 信息速率=%4 Kbps, 跳频图案=%5")
                  .arg(m_selectedMode, m_centerFreq, m_bandwidth, m_infoRate, m_hopPattern));
}

void Backend::connectUsrp(bool on)
{
    if (m_usrpConnected == on)
        return;

    m_usrpConnected = on;
    emit usrpConnectedChanged();

    appendLog(on ? "USRP B210 已连接，后续任务将走 USRP 模式"
                 : "USRP B210 已断开，后续任务将走 LOOPBACK 模式");
}

void Backend::sendText()
{
    setTransferSource("TEXT");

    if (m_inputText.trimmed().isEmpty()) {
        appendLog("文本装载失败：输入为空。");
        return;
    }

    appendLog(QString("文本已装载，长度=%1 字符").arg(m_inputText.size()));
}

void Backend::browseInputFile()
{
    appendLog("QML FileDialog 将负责选文件，当前接口保留。");
}

void Backend::sendFile()
{
    setTransferSource("FILE");

    if (m_inputFilePath.trimmed().isEmpty()) {
        appendLog("文件装载失败：尚未选择文件。");
        return;
    }

    QFileInfo fi(m_inputFilePath);
    appendLog(QString("文件已装载：%1").arg(fi.fileName()));
    appendLog(QString("完整路径：%1").arg(m_inputFilePath));
}

void Backend::startSending(bool on)
{
    if (!on) {
        if (m_sending) {
            m_sending = false;
            emit sendingChanged();
            appendLog("发送已停止。");
        }
        return;
    }

    if (!payloadReady()) {
        appendLog("发送失败：当前没有可发送的数据。");
        if (m_transferSource == "TEXT")
            appendLog("请先输入文本。");
        else if (m_transferSource == "FILE")
            appendLog("请先选择文件。");
        else
            appendLog("当前数据源异常。");
        return;
    }

    if (m_isRunning) {
        appendLog("系统忙：当前任务尚未结束。");
        return;
    }

    m_sending = true;
    emit sendingChanged();

    runCurrentTask(true);
}

void Backend::runSimulation()
{
    if (m_isRunning) {
        appendLog("仿真正在运行，请勿重复点击。");
        return;
    }

    m_isRunning = true;

    const double centerFreqHz = m_centerFreq.toDouble() * 1e6;
    const double infoRateBps = m_infoRate.toDouble() * 1e3;
    const int hopPattern = m_hopPattern.toInt();

    const std::string modStr = m_selectedMode.toStdString();
    const ModulationType modulation = parse_modulation(modStr);
    const RunMode runMode = m_usrpConnected ? RunMode::USRP : RunMode::LOOPBACK;

    appendLog(QString("开始运行随机比特 BER 测试：模式=%1, 调制=%2, Fc=%3 MHz, 信息速率=%4 Kbps, 跳频图案=%5")
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
            const QString err = QString("任务失败: %1").arg(e.what());
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
                appendLog("任务失败: Unknown exception");
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
        appendLog("任务正在运行，请勿重复触发。");
        return;
    }

    if (!payloadReady()) {
        appendLog("任务启动失败：当前数据未准备好。");
        return;
    }

    m_isRunning = true;

    const double centerFreqHz = m_centerFreq.toDouble() * 1e6;
    const double infoRateBps = m_infoRate.toDouble() * 1e3;
    const int hopPattern = m_hopPattern.toInt();

    const std::string modStr = m_selectedMode.toStdString();
    const ModulationType modulation = parse_modulation(modStr);
    const RunMode runMode = m_usrpConnected ? RunMode::USRP : RunMode::LOOPBACK;

    const QString actionName = fromSendingAction ? "开始发送" : "开始运行任务";

    appendLog(QString("%1：模式=%2, 调制=%3, Fc=%4 MHz, 信息速率=%5 Kbps, 跳频图案=%6, 数据源=%7")
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
                const QString err = QString("任务失败: %1").arg(e.what());
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
                    appendLog("任务失败: Unknown exception");
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
        appendLog(QString("文本已写入临时文件：%1").arg(inputPath));
    } else {
        inputPath = m_inputFilePath;
    }

    const QString outputPath = buildRecoveredOutputPath(inputPath);

    appendLog(QString("接收恢复文件将保存到：%1").arg(outputPath));

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
            const QString err = QString("任务失败: %1").arg(e.what());
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
                appendLog("任务失败: Unknown exception");
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
            appendLog(QString("文件恢复成功：%1").arg(QString::fromStdString(tr.saved_file_path)));
        }

        appendLog(QString("任务完成，BER=%1").arg(m_berText));
    }
    catch (const std::exception &e) {
        appendLog(QString("任务失败: %1").arg(e.what()));
    }
    catch (...) {
        appendLog("任务失败: Unknown exception");
    }

    if (m_sending) {
        m_sending = false;
        emit sendingChanged();
        appendLog("本次发送流程结束。");
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
        appendLog(QString("文件恢复成功：%1").arg(QString::fromStdString(tr.saved_file_path)));
    }

    appendLog(QString("任务完成，BER=%1").arg(m_berText));

    m_isRunning = false;

    if (m_sending) {
        m_sending = false;
        emit sendingChanged();
        appendLog("本次发送流程结束。");
    }
}