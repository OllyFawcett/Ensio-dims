#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include "cameraThread.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onNewFrame(cv::Mat frame);

    void on_connectBtn_clicked();

    void on_calibrateBtn_clicked();

private:
    Ui::MainWindow *ui;
    QLabel *streamLabel;
    QPushButton *connectBtn;
    CameraThread *cameraThread;
    bool connected;
};

#endif // MAINWINDOW_H

