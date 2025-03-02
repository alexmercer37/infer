#ifndef FILTER_H
#define FILTER_H
#include "main.hpp"

class Filter
{
public:
    void init_filter(cv::KalmanFilter &kf);
    void filter_predict(std::vector<cv::Point2f> &measurements, cv::KalmanFilter &kf, cv::Mat frame);

private:
    cv::KalmanFilter kf;
    std::vector<cv::Point2f> measurements;
    cv::Mat frame;
};
#endif