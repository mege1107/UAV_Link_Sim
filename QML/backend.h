#pragma once
#include <QObject>
#include <QString>
#include <QVariantList>
#include <QFutureWatcher>
#include "sim_runner.h"

class Backend : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QString logText READ logText NOTIFY logTextChanged)
    Q_PROPERTY(QString berText READ berText NOTIFY berTextChanged)

    Q_PROPERTY(QVariantList waveform READ waveform NOTIFY waveformChanged)
    Q_PROPERTY(double waveformMin READ waveformMin NOTIFY waveformChanged)
    Q_PROPERTY(double waveformMax READ waveformMax NOTIFY waveformChanged)

    Q_PROPERTY(QVariantList spectrumFreq READ spectrumFreq NOTIFY spectrumChanged)
    Q_PROPERTY(QVariantList spectrumMag READ spectrumMag NOTIFY spectrumChanged)

    Q_PROPERTY(QVariantList spectrogramData READ spectrogramData NOTIFY spectrogramChanged)
    Q_PROPERTY(int spectrogramWidth READ spectrogramWidth NOTIFY spectrogramChanged)
    Q_PROPERTY(int spectrogramHeight READ spectrogramHeight NOTIFY spectrogramChanged)
    Q_PROPERTY(double spectrogramTimeSpan READ spectrogramTimeSpan NOTIFY spectrogramChanged)
    Q_PROPERTY(double spectrogramFreqMin READ spectrogramFreqMin NOTIFY spectrogramChanged)
    Q_PROPERTY(double spectrogramFreqMax READ spectrogramFreqMax NOTIFY spectrogramChanged)

    Q_PROPERTY(bool usrpConnected READ usrpConnected NOTIFY usrpConnectedChanged)
    Q_PROPERTY(bool sending READ sending NOTIFY sendingChanged)

    Q_PROPERTY(QString selectedMode READ selectedMode WRITE setSelectedMode NOTIFY selectedModeChanged)
    Q_PROPERTY(QString centerFreq READ centerFreq WRITE setCenterFreq NOTIFY centerFreqChanged)
    Q_PROPERTY(QString bandwidth READ bandwidth WRITE setBandwidth NOTIFY bandwidthChanged)
    Q_PROPERTY(QString infoRate READ infoRate WRITE setInfoRate NOTIFY infoRateChanged)
    Q_PROPERTY(QString hopPattern READ hopPattern WRITE setHopPattern NOTIFY hopPatternChanged)

    Q_PROPERTY(QString inputText READ inputText WRITE setInputText NOTIFY inputTextChanged)
    Q_PROPERTY(QString inputFilePath READ inputFilePath WRITE setInputFilePath NOTIFY inputFilePathChanged)
    Q_PROPERTY(QString transferSource READ transferSource NOTIFY transferSourceChanged)

    Q_PROPERTY(QVariantList rxWaveform READ rxWaveform NOTIFY rxWaveformChanged)
    Q_PROPERTY(double rxWaveformMin READ rxWaveformMin NOTIFY rxWaveformChanged)
    Q_PROPERTY(double rxWaveformMax READ rxWaveformMax NOTIFY rxWaveformChanged)

    Q_PROPERTY(QVariantList rxSpectrumFreq READ rxSpectrumFreq NOTIFY rxSpectrumChanged)
    Q_PROPERTY(QVariantList rxSpectrumMag READ rxSpectrumMag NOTIFY rxSpectrumChanged)


public:
    explicit Backend(QObject *parent = nullptr);

    QString logText() const;
    QString berText() const;

    QVariantList waveform() const;
    double waveformMin() const;
    double waveformMax() const;

    QVariantList spectrumFreq() const;
    QVariantList spectrumMag() const;

    QVariantList spectrogramData() const;
    int spectrogramWidth() const;
    int spectrogramHeight() const;
    double spectrogramTimeSpan() const;
    double spectrogramFreqMin() const;
    double spectrogramFreqMax() const;

    bool usrpConnected() const;
    bool sending() const;

    QString selectedMode() const;
    void setSelectedMode(const QString &value);

    QString centerFreq() const;
    void setCenterFreq(const QString &value);

    QString bandwidth() const;
    void setBandwidth(const QString &value);

    QString infoRate() const;
    void setInfoRate(const QString &value);

    QString inputText() const;
    void setInputText(const QString &value);

    QString inputFilePath() const;
    void setInputFilePath(const QString &value);

    QString transferSource() const;

    QVariantList rxWaveform() const;
    double rxWaveformMin() const;
    double rxWaveformMax() const;

    QVariantList rxSpectrumFreq() const;
    QVariantList rxSpectrumMag() const;

    QString hopPattern() const;
    void setHopPattern(const QString &value);

    Q_INVOKABLE void setTransferSource(const QString &source);
    Q_INVOKABLE void applyParameterSettings();
    Q_INVOKABLE void connectUsrp(bool on);
    Q_INVOKABLE void sendText();
    Q_INVOKABLE void browseInputFile();
    Q_INVOKABLE void sendFile();
    Q_INVOKABLE void startSending(bool on);
    Q_INVOKABLE void runSimulation();

signals:
    void logTextChanged();
    void berTextChanged();
    void waveformChanged();
    void spectrumChanged();
    void spectrogramChanged();

    void usrpConnectedChanged();
    void sendingChanged();

    void selectedModeChanged();
    void centerFreqChanged();
    void bandwidthChanged();
    void infoRateChanged();

    void inputTextChanged();
    void inputFilePathChanged();
    void transferSourceChanged();

    void hopPatternChanged();

    void rxWaveformChanged();
    void rxSpectrumChanged();

private:
    void appendLog(const QString &msg);
    void handleSimulationFinished();

    bool payloadReady() const;
    QString currentPayloadDescription() const;
    QString ensureTempTextFile();
    QString buildRecoveredOutputPath(const QString &srcPath) const;
    void runCurrentTask(bool fromSendingAction);

    QString m_hopPattern;

private:
    QString m_logText;
    QString m_berText;

    QVariantList m_waveform;
    double m_waveformMin = -1.0;
    double m_waveformMax = 1.0;

    QVariantList m_spectrumFreq;
    QVariantList m_spectrumMag;

    QVariantList m_spectrogramData;
    int m_spectrogramWidth = 0;
    int m_spectrogramHeight = 0;
    double m_spectrogramTimeSpan = 1.0;
    double m_spectrogramFreqMin = -1.0;
    double m_spectrogramFreqMax = 1.0;

    bool m_usrpConnected = false;
    bool m_sending = false;

    QVariantList m_rxWaveform;
    double m_rxWaveformMin = -1.0;
    double m_rxWaveformMax = 1.0;

    QVariantList m_rxSpectrumFreq;
    QVariantList m_rxSpectrumMag;

    QString m_selectedMode;
    QString m_centerFreq;
    QString m_bandwidth;
    QString m_infoRate;

    QString m_inputText;
    QString m_inputFilePath;
    QString m_transferSource;   // "RANDOM" / "TEXT" / "FILE"
    QString m_tempTextFilePath;

    QFutureWatcher<void> m_simWatcher;
    TestResult m_pendingResult;
    void applyTestResult(const TestResult &tr);
    bool m_isRunning = false;
};