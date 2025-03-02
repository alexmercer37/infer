/**
 * @file filter.cpp
 * @author alexmercer37 (3416181310@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-09-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "../inc/filter.hpp"
using namespace cv;
using namespace std;
Filter *filter = new Filter;

void Filter::init_filter(cv::KalmanFilter &kf)
{

    kf.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);

    kf.measurementMatrix = (Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
    setIdentity(kf.processNoiseCov, Scalar(1e-5));
    setIdentity(kf.measurementNoiseCov, Scalar(1e-1));
    setIdentity(kf.errorCovPost, Scalar(1));

    Mat state(4, 1, CV_32F);
    state.at<float>(0) = 0;
    state.at<float>(1) = 0;
    state.at<float>(2) = 0;
    state.at<float>(3) = 0;
    kf.statePost = state;
}

void Filter::filter_predict(std::vector<cv::Point2f> &measurements, cv::KalmanFilter &kf, cv::Mat frame)
{

    vector<Point2f> trajectory;
    Mat prediction = kf.predict();

    Point2f predictPt(prediction.at<float>(0) * 1, prediction.at<float>(1) * 1);
    circle(frame, predictPt, 5, Scalar(0, 255, 0), -1);

    Mat measurement = (Mat_<float>(2, 1) << measurements[0].x, measurements[0].y);

    kf.correct(measurement);

    Mat estimated = kf.statePost;
    Point2f estimatedPt(estimated.at<float>(0) * 5, estimated.at<float>(1) * 5);
    // trajectory.push_back(estimatedPt);

    // for (size_t j = 1; j < trajectory.size(); ++j)
    // {
    //     line(frame, trajectory[j - 1], trajectory[j], Scalar(255, 0, 0), 2);
    // }

    imshow("Tracking", frame);
    waitKey(1);
}