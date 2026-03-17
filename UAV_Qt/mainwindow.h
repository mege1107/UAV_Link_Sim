#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFutureWatcher>
#include <QString>

// Qt Charts
#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLogValueAxis>

#include "sim_runner.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_btnRun_clicked();
    void on_btnSweep_clicked();

    void onSimulationFinished();
    void onSweepFinished();

    void on_comboMode_currentTextChanged(const QString &text);

    void on_checkFileTransfer_toggled(bool checked);
    void on_btnBrowseInput_clicked();
    void on_btnBrowseOutput_clicked();
    void on_btnOpenOutput_clicked();

private:
    void initBerChart();
    void plotSingleBerPoint(double snrDb, double ber);
    void plotBerCurve(const SweepResult& sr);

    void updateUiState();
    bool isFileTransferMode() const;

private:
    Ui::MainWindow *ui;

    QFutureWatcher<TestResult> watcher;
    QFutureWatcher<SweepResult> sweepWatcher;

    QChartView *berChartView = nullptr;

    QString lastSavedFilePath;
};

#endif