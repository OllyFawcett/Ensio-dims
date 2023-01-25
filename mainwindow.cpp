#include "mainwindow.h"
#include "./ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    cameraThread = new CameraThread();

    connect(cameraThread, SIGNAL(newFrame(cv::Mat)), this, SLOT(onNewFrame(cv::Mat)));

    connected = false;
}

MainWindow::~MainWindow()
{
    cameraThread->stop();
    cameraThread->wait();
}


void MainWindow::onNewFrame(cv::Mat frame)
{
    QImage image(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
    ui->stream->setPixmap(QPixmap::fromImage(image));
}



void MainWindow::on_connectBtn_clicked()
{
    if (!connected) {
        cameraThread->start();
        ui->connectBtn->setText("Disconnect");
    } else {
        cameraThread->stop();
        ui->connectBtn->setText("Connect");

    }
    connected = !connected;

}


void MainWindow::on_calibrateBtn_clicked()
{
    if(connected)
    {
        cameraThread->lg_height = ui->loadguardheightInput->text().toFloat();
        cameraThread->height = ui->boxheightInput->text().toFloat();
        cameraThread->cols = ui->columnsInput->text().toInt();
        cameraThread->rows = ui->rowsInput->text().toInt();
        cameraThread->dist = ui->distInput->text().toFloat();
        cameraThread->calibrateCam();
        cameraThread->stop();
    }
}

