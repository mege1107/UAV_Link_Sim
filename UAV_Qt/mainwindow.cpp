#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QtConcurrent>
#include <QVBoxLayout>
#include <QString>
#include <QtCharts/QAbstractAxis>
#include <algorithm>

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

    initBerChart();
    on_comboMode_currentTextChanged(ui->comboMode->currentText());
}

MainWindow::~MainWindow()
{
    delete ui;
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

void MainWindow::on_comboMode_currentTextChanged(const QString &text)
{
    if (text == "AWGN")
    {
        ui->spinSNR->setEnabled(true);
        ui->spinFcGHz->setEnabled(false);

        ui->spinSNRStart->setEnabled(true);
        ui->spinSNREnd->setEnabled(true);
        ui->spinSNRStep->setEnabled(true);
        ui->btnSweep->setEnabled(true);
    }
    else if (text == "LOOPBACK")
    {
        ui->spinSNR->setEnabled(false);
        ui->spinFcGHz->setEnabled(false);

        ui->spinSNRStart->setEnabled(false);
        ui->spinSNREnd->setEnabled(false);
        ui->spinSNRStep->setEnabled(false);
        ui->btnSweep->setEnabled(false);
    }
    else if (text == "USRP")
    {
        ui->spinSNR->setEnabled(false);
        ui->spinFcGHz->setEnabled(true);

        ui->spinSNRStart->setEnabled(false);
        ui->spinSNREnd->setEnabled(false);
        ui->spinSNRStep->setEnabled(false);
        ui->btnSweep->setEnabled(false);
    }
}

void MainWindow::on_btnRun_clicked()
{
    ui->btnRun->setEnabled(false);
    ui->btnSweep->setEnabled(false);

    QString modText = ui->comboMod->currentText();
    QString modeText = ui->comboMode->currentText();

    int snr = ui->spinSNR->value();
    int frames = ui->spinFrames->value();
    double fcGHz = ui->spinFcGHz->value();
    double fcHz = fcGHz * 1e9;

    ui->labelBER->setText(QString("Running... (%1 frames)").arg(frames));

    ModulationType mod = parse_modulation(modText.toStdString());
    RunMode mode = parse_mode(modeText.toStdString());

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

void MainWindow::onSimulationFinished()
{
    TestResult tr = watcher.result();

    ui->labelBER->setText(
        QString("BER = %1").arg(tr.total_ber, 0, 'e', 4)
        );

    if (ui->textLog) {
        ui->textLog->setPlainText(QString::fromStdString(tr.log_text));
    }

    if (ui->comboMode->currentText() == "AWGN") {
        plotSingleBerPoint(ui->spinSNR->value(), tr.total_ber);
    }

    ui->btnRun->setEnabled(true);
    if (ui->comboMode->currentText() == "AWGN") {
        ui->btnSweep->setEnabled(true);
    }
}

void MainWindow::on_btnSweep_clicked()
{
    QString modeText = ui->comboMode->currentText();
    if (modeText != "AWGN") {
        ui->labelBER->setText("Sweep only supports AWGN");
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

    ui->btnRun->setEnabled(true);
    if (ui->comboMode->currentText() == "AWGN") {
        ui->btnSweep->setEnabled(true);
    }
}