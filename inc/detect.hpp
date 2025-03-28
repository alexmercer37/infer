/**
 * @file detect.hpp
 * @author alexmercer37 (3450141407@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __INFER_H__
#define __INFER_H__

#include "main.hpp"
#include <k4a/k4a.hpp>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>

#define use_random_color

static const char *cocolabels[] = {"red_ball", "blue_ball"};

class Detect
{
private:
    cv::Mat *image;
    std::shared_ptr<yolo::Infer> yolo;

public:
    void Time();
    void perf();
    void batch_inference();
    yolo::BoxArray single_inference(std::shared_ptr<cv::Mat> image, std::shared_ptr<yolo::Infer> yolo);
    yolo::BoxArray seg_inference(std::shared_ptr<cv::Mat> image, std::shared_ptr<yolo::Infer> yolo);
    yolo::BoxArray single_inference(std::shared_ptr<cv::Mat> image, std::shared_ptr<yolo::Infer> yolo, cv::KalmanFilter &kf, std::deque<cv::Point2f> &measurements, std::vector<cv::Point2f> &trajectory, uint16_t &flag);
    void detect_boxes(yolo::BoxArray &bboxes, cv::Mat rgb, cv::Mat &cv_depth, k4a::transformation &k4aTransformation, k4a::calibration &k4aCalibration);

    void setYolo(std::shared_ptr<yolo::Infer> new_yolo)
    {
        new_yolo = yolo;
    };
};
#endif