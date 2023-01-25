#ifndef CAMERATHREAD_H
#define CAMERATHREAD_H

#include <QThread>
#include <opencv2/opencv.hpp>

class CameraThread : public QThread
{
    Q_OBJECT

public:
    CameraThread();
    void run();
    void stop();
    void start();
    void calibrateCam();
    float rows;
    float cols;
    float height;
    float lg_height;
    float dist;

signals:
    void newFrame(cv::Mat frame);

private:
    bool stopped;
    bool calibrate;
    bool calibrated;

};

#endif // CAMERATHREAD_H
