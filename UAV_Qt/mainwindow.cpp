#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QtConcurrent>
#include <QVBoxLayout>
#include <QString>
#include <QtCharts/QAbstractAxis>
#include <algorithm>

#include <QFileDialog>
#include <QFileInfo>
#include <QDesktopServices>
#include <QUrl>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->comboMod->clear();
    ui->comboMod->addItem("BPSK");
    ui->comboMod->addItem("QPSK");
    ui->comboMod->addItem("QAM");
    ui->comboMod->addItem("FSK");
    ui->comboMod->addItem("OOK");
    ui->comboMod->addItem("MSK");
    ui->comboMod->setCurrentText("BPSK");

    ui->comboMode->clear();
    ui->comboMode->addItem("AWGN");
    ui->comboMode->addItem("LOOPBACK");
    ui->comboMode->addItem("USRP");
    ui->comboMode->setCurrentText("AWGN");

    ui->spinSNR->setRange(-20, 40);
    ui->spinSNR->setValue(10);

    ui->spinFcGHz->setRange(0.1, 10.0);
    ui->spinFcGHz->setDecimals(3);
    ui->spinFcGHz->setSingleStep(0.01);
    ui->spinFcGHz->setValue(2.45);

    ui->spinSNRStart->setRange(-20, 40);
    ui->spinSNRStart->setValue(0);

    ui->spinSNREnd->setRange(-20, 40);
    ui->spinSNREnd->setValue(20);

    ui->spinSNRStep->setRange(1, 20);
    ui->spinSNRStep->setValue(2);

    ui->spinFrames->setRange(1, 10000);
    ui->spinFrames->setSingleStep(10);
    ui->spinFrames->setValue(200);

    connect(&watcher, &QFutureWatcher<TestResult>::finished,
            this, &MainWindow::onSimulationFinished);

    connect(&sweepWatcher, &QFutureWatcher<SweepResult>::finished,
            this, &MainWindow::onSweepFinished);

    connect(ui->comboMode,
            &QComboBox::currentTextChanged,
            this,
            &MainWindow::on_comboMode_currentTextChanged);

    connect(ui->checkFileTransfer, &QCheckBox::toggled,
            this, &MainWindow::on_checkFileTransfer_toggled);

    connect(ui->btnBrowseInput, &QPushButton::clicked,
            this, &MainWindow::on_btnBrowseInput_clicked);

    connect(ui->btnBrowseOutput, &QPushButton::clicked,
            this, &MainWindow::on_btnBrowseOutput_clicked);

    connect(ui->btnOpenOutput, &QPushButton::clicked,
            this, &MainWindow::on_btnOpenOutput_clicked);

    initBerChart();
    updateUiState();
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool MainWindow::isFileTransferMode() const
{
    return ui->checkFileTransfer && ui->checkFileTransfer->isChecked();
}

void MainWindow::updateUiState()
{
    const QString modeText = ui->comboMode->currentText();
    const bool fileMode = isFileTransferMode();

    if (modeText == "AWGN")
    {
        ui->spinSNR->setEnabled(true);
        ui->spinFcGHz->setEnabled(false);

        ui->spinSNRStart->setEnabled(!fileMode);
        ui->spinSNREnd->setEnabled(!fileMode);
        ui->spinSNRStep->setEnabled(!fileMode);
        ui->btnSweep->setEnabled(!fileMode);
    }
    else if (modeText == "LOOPBACK")
    {
        ui->spinSNR->setEnabled(false);
        ui->spinFcGHz->setEnabled(false);

        ui->spinSNRStart->setEnabled(false);
        ui->spinSNREnd->setEnabled(false);
        ui->spinSNRStep->setEnabled(false);
        ui->btnSweep->setEnabled(false);
    }
    else if (modeText == "USRP")
    {
        ui->spinSNR->setEnabled(false);
        ui->spinFcGHz->setEnabled(true);

        ui->spinSNRStart->setEnabled(false);
        ui->spinSNREnd->setEnabled(false);
        ui->spinSNRStep->setEnabled(false);
        ui->btnSweep->setEnabled(false);
    }

    ui->editInputFile->setEnabled(fileMode);
    ui->editOutputFile->setEnabled(fileMode);
    ui->btnBrowseInput->setEnabled(fileMode);
    ui->btnBrowseOutput->setEnabled(fileMode);
    ui->btnOpenOutput->setEnabled(fileMode && !lastSavedFilePath.isEmpty());
}

void MainWindow::initBerChart()
{
    berChartView = new QChartView(this);
    berChartView->setRenderHint(QPainter::Antialiasing);

    auto *chart = new QChart();
    chart->setTitle("BER Curve");
    chart->legend()->hide();
    berChartView->setChart(chart);

    auto *layout = new QVBoxLayout(ui->widgetBerChart);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(berChartView);

    auto *series = new QLineSeries();
    chart->addSeries(series);

    auto *axisX = new QValueAxis();
    axisX->setTitleText("SNR (dB)");
    axisX->setRange(0, 20);

    auto *axisY = new QLogValueAxis();
    axisY->setTitleText("BER");
    axisY->setBase(10.0);
    axisY->setRange(1e-8, 1.0);
    axisY->setLabelFormat("%.0e");

    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisX);
    series->attachAxis(axisY);
}

void MainWindow::plotSingleBerPoint(double snrDb, double ber)
{
    auto *chart = berChartView->chart();
    chart->removeAllSeries();

    const auto axes = chart->axes();
    for (QAbstractAxis *axis : axes) {
        chart->removeAxis(axis);
        delete axis;
    }

    auto *series = new QScatterSeries();
    series->setMarkerSize(10.0);

    const double safeBer = std::max(ber, 1e-8);
    series->append(snrDb, safeBer);

    chart->addSeries(series);
    chart->setTitle("BER Result");

    auto *axisX = new QValueAxis();
    axisX->setTitleText("SNR (dB)");
    axisX->setRange(snrDb - 1.0, snrDb + 1.0);

    auto *axisY = new QLogValueAxis();
    axisY->setTitleText("BER");
    axisY->setBase(10.0);
    axisY->setRange(1e-8, 1.0);
    axisY->setLabelFormat("%.0e");

    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisX);
    series->attachAxis(axisY);
}

void MainWindow::plotBerCurve(const SweepResult& sr)
{
    auto *chart = berChartView->chart();
    chart->removeAllSeries();

    const auto axes = chart->axes();
    for (QAbstractAxis *axis : axes) {
        chart->removeAxis(axis);
        delete axis;
    }

    auto *series = new QLineSeries();

    if (sr.points.empty()) {
        chart->setTitle("BER Curve (no data)");
        chart->addSeries(series);

        auto *axisX = new QValueAxis();
        axisX->setTitleText("SNR (dB)");
        axisX->setRange(0, 20);

        auto *axisY = new QLogValueAxis();
        axisY->setTitleText("BER");
        axisY->setBase(10.0);
        axisY->setRange(1e-8, 1.0);
        axisY->setLabelFormat("%.0e");

        chart->addAxis(axisX, Qt::AlignBottom);
        chart->addAxis(axisY, Qt::AlignLeft);
        series->attachAxis(axisX);
        series->attachAxis(axisY);
        return;
    }

    double minSnr = sr.points.front().snr_db;
    double maxSnr = sr.points.front().snr_db;

    for (const auto& pt : sr.points) {
        const double safeBer = std::max(pt.ber, 1e-8);
        series->append(pt.snr_db, safeBer);
        minSnr = std::min(minSnr, pt.snr_db);
        maxSnr = std::max(maxSnr, pt.snr_db);
    }

    chart->addSeries(series);
    chart->setTitle("BER Curve");

    auto *axisX = new QValueAxis();
    axisX->setTitleText("SNR (dB)");
    if (minSnr == maxSnr) {
        axisX->setRange(minSnr - 1.0, maxSnr + 1.0);
    } else {
        axisX->setRange(minSnr, maxSnr);
    }

    auto *axisY = new QLogValueAxis();
    axisY->setTitleText("BER");
    axisY->setBase(10.0);
    axisY->setRange(1e-8, 1.0);
    axisY->setLabelFormat("%.0e");

    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisX);
    series->attachAxis(axisY);
}

void MainWindow::on_comboMode_currentTextChanged(const QString &)
{
    updateUiState();
}

void MainWindow::on_checkFileTransfer_toggled(bool)
{
    updateUiState();
}

void MainWindow::on_btnBrowseInput_clicked()
{
    const QString path = QFileDialog::getOpenFileName(
        this,
        "Select Input File"
        );

    if (path.isEmpty()) return;

    ui->editInputFile->setText(path);

    if (ui->editOutputFile->text().trimmed().isEmpty()) {
        QFileInfo fi(path);
        QString outPath = fi.absolutePath() + "/recovered_" + fi.fileName();
        ui->editOutputFile->setText(outPath);
    }
}

void MainWindow::on_btnBrowseOutput_clicked()
{
    const QString path = QFileDialog::getSaveFileName(
        this,
        "Select Output File Path"
        );

    if (path.isEmpty()) return;

    ui->editOutputFile->setText(path);
}

void MainWindow::on_btnOpenOutput_clicked()
{
    if (lastSavedFilePath.isEmpty()) {
        QMessageBox::information(this, "Info", "No recovered output file yet.");
        return;
    }

    QDesktopServices::openUrl(QUrl::fromLocalFile(lastSavedFilePath));
}

void MainWindow::on_btnRun_clicked()
{
    ui->btnRun->setEnabled(false);
    ui->btnSweep->setEnabled(false);
    ui->btnOpenOutput->setEnabled(false);
    lastSavedFilePath.clear();

    QString modText = ui->comboMod->currentText();
    QString modeText = ui->comboMode->currentText();

    int snr = ui->spinSNR->value();
    int frames = ui->spinFrames->value();
    double fcGHz = ui->spinFcGHz->value();
    double fcHz = fcGHz * 1e9;

    ModulationType mod = parse_modulation(modText.toStdString());
    RunMode mode = parse_mode(modeText.toStdString());

    if (isFileTransferMode())
    {
        const QString inputPath = ui->editInputFile->text().trimmed();
        const QString outputPath = ui->editOutputFile->text().trimmed();

        if (inputPath.isEmpty()) {
            QMessageBox::warning(this, "Warning", "Please select an input file.");
            updateUiState();
            ui->btnRun->setEnabled(true);
            return;
        }

        if (outputPath.isEmpty()) {
            QMessageBox::warning(this, "Warning", "Please select an output file path.");
            updateUiState();
            ui->btnRun->setEnabled(true);
            return;
        }

        ui->labelBER->setText("Running file transfer...");

        QFuture<TestResult> future = QtConcurrent::run([=]() {
            return run_file_transfer_test(
                mode,
                static_cast<double>(snr),
                fcHz,
                mod,
                inputPath.toStdString(),
                outputPath.toStdString()
                );
        });

        watcher.setFuture(future);
    }
    else
    {
        ui->labelBER->setText(QString("Running... (%1 frames)").arg(frames));

        QFuture<TestResult> future = QtConcurrent::run([=]() {
            return run_one_test(
                mode,
                static_cast<double>(snr),
                frames,
                fcHz,
                mod
                );
        });

        watcher.setFuture(future);
    }
}

void MainWindow::onSimulationFinished()
{
    try
    {
        TestResult tr = watcher.result();

        ui->labelBER->setText(
            QString("BER = %1").arg(tr.total_ber, 0, 'e', 4)
            );

        if (ui->textLog) {
            ui->textLog->setPlainText(QString::fromStdString(tr.log_text));
        }

        if (isFileTransferMode())
        {
            if (tr.file_saved) {
                lastSavedFilePath = QString::fromStdString(tr.saved_file_path);
                ui->btnOpenOutput->setEnabled(true);
                ui->labelBER->setText(
                    QString("BER = %1, file recovered")
                        .arg(tr.total_ber, 0, 'e', 4)
                    );
            } else {
                lastSavedFilePath.clear();
                ui->btnOpenOutput->setEnabled(false);
                ui->labelBER->setText(
                    QString("BER = %1, file recovery failed")
                        .arg(tr.total_ber, 0, 'e', 4)
                    );
            }
        }
        else if (ui->comboMode->currentText() == "AWGN")
        {
            plotSingleBerPoint(ui->spinSNR->value(), tr.total_ber);
        }
    }
    catch (const std::exception& e)
    {
        ui->labelBER->setText("Run failed");
        if (ui->textLog) {
            ui->textLog->setPlainText(QString("Error: %1").arg(e.what()));
        }
        QMessageBox::critical(this, "Error", e.what());
    }

    ui->btnRun->setEnabled(true);
    updateUiState();
}

void MainWindow::on_btnSweep_clicked()
{
    QString modeText = ui->comboMode->currentText();
    if (modeText != "AWGN") {
        ui->labelBER->setText("Sweep only supports AWGN");
        return;
    }

    if (isFileTransferMode()) {
        ui->labelBER->setText("Sweep is disabled in file transfer mode");
        return;
    }

    ui->btnRun->setEnabled(false);
    ui->btnSweep->setEnabled(false);

    QString modText = ui->comboMod->currentText();
    ModulationType mod = parse_modulation(modText.toStdString());

    double snrStart = ui->spinSNRStart->value();
    double snrEnd   = ui->spinSNREnd->value();
    double snrStep  = ui->spinSNRStep->value();
    int frames      = ui->spinFrames->value();
    double fcHz     = ui->spinFcGHz->value() * 1e9;

    ui->labelBER->setText(QString("Sweeping... (%1 frames/point)").arg(frames));

    QFuture<SweepResult> future = QtConcurrent::run([=]() {
        return run_awgn_sweep(
            snrStart,
            snrEnd,
            snrStep,
            frames,
            fcHz,
            mod
            );
    });

    sweepWatcher.setFuture(future);
}

void MainWindow::onSweepFinished()
{
    try
    {
        SweepResult sr = sweepWatcher.result();

        QString text;
        for (const auto& pt : sr.points) {
            text += QString("SNR = %1 dB, BER = %2, decoded = %3\n")
            .arg(pt.snr_db, 0, 'f', 2)
                .arg(pt.ber, 0, 'e', 4)
                .arg((qulonglong)pt.decoded_frames);
        }

        if (ui->textLog) {
            ui->textLog->setPlainText(text);
        }

        plotBerCurve(sr);

        if (!sr.points.empty()) {
            ui->labelBER->setText(
                QString("Last BER = %1")
                    .arg(sr.points.back().ber, 0, 'e', 4)
                );
        } else {
            ui->labelBER->setText("Sweep finished");
        }
    }
    catch (const std::exception& e)
    {
        ui->labelBER->setText("Sweep failed");
        if (ui->textLog) {
            ui->textLog->setPlainText(QString("Error: %1").arg(e.what()));
        }
        QMessageBox::critical(this, "Error", e.what());
    }

    ui->btnRun->setEnabled(true);
    updateUiState();
}
