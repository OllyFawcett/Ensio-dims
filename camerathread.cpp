#include "cameraThread.h"
#include "dims_tools.hpp"
#include <QDebug>

CameraThread::CameraThread()
{
    stopped = false;
    calibrate = false;
    calibrated = false;
}

void CameraThread::run()
{
    rs2::context                          ctx;

    std::map<std::string, rs2::colorizer> colorizers;

    std::vector<rs2::pipeline>            pipelines;

    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices())
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH);
        cfg.enable_stream(RS2_STREAM_COLOR);
        cfg.enable_device(serial);
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        colorizers[serial] = rs2::colorizer();
    }
    colorizer c;
    rs2::align align_to_color(RS2_STREAM_COLOR);
    calibration_state cal_state;
    while (!stopped) {
        auto frames = pipelines[0].wait_for_frames();
        auto color = frames.get_color_frame();
        if (!color)
            color = frames.get_infrared_frame();

        auto depth = frames.get_depth_frame();

        const int w = color.as<rs2::video_frame>().get_width();
        const int h = color.as<rs2::video_frame>().get_height();
        Mat frame(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        if(!calibrated)
        {
            if (!frame.empty()) {
                emit newFrame(frame);
            }
        }

        if(calibrate)
        {
            cout << "calibrating" << endl;
            cal_state = calibrateCamera(pipelines[0], 0, true, "right", this->dist, this->height, this->rows, this->cols);
            cout << "calibrated" << endl;
            calibrated = true;
            calibrate = false;
            emit newFrame(cal_state.depth_image);
        }
    }
}
void CameraThread::start()
{
    cout << "starting thread" << endl;
    QThread::start();
}

void CameraThread::calibrateCam()
{
    calibrate=true;
}

void CameraThread::stop()
{
    stopped = true;
    quit();
    cout << "stopped thread" << endl;
}





