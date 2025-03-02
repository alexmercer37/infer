/**
 * @file camera.hpp
 * @author alexmercer37 (3450141407@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef CAMERA_H
#define CAMERA_H
#include "main.hpp"
#include "PclHead.hpp"
#include <k4a/k4a.hpp>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>

class Camera
{
public:
    void init_kinect(k4a::device &device, k4a::capture &capture, k4a::transformation &k4aTransformation, k4a::calibration &k4aCalibration);
    void picture_update(k4a::device &device, k4a::capture &capture);
    void getAngel(k4a::device &device);
    cv::Mat *getpicture(k4a::device &device, k4a::capture &capture, cv::Mat &cv_color, k4a::transformation &k4aTransformation);
    cv::Mat *getdepth(k4a::device &device, k4a::capture &capture, cv::Mat &cv_depth, k4a::transformation &k4aTransformation);
    cv::Mat *getir(k4a::device &device, k4a::capture &capture, cv::Mat &cv_infrared, k4a::transformation &k4aTransformation);
    void stopCamera();
    void camera_detect(cv::Mat currFrame);
    void Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud, yolo::BoxArray &objs);

private:
    u_int32_t device_count;
    k4a::device device;
    k4a_device_configuration_t init;
    k4a::image k4a_color;
    k4a::image k4a_depth;
    k4a::image k4a_infrared;
    k4a::image k4a_tf_depth;
    k4a::capture capture;
    cv::Mat cv_color;
    cv::Mat cv_depth;
    cv::Mat cv_infrared;
    k4a::transformation k4aTransformation;
    k4a::calibration k4aCalibration;
    k4a_calibration_camera_t color_intrinsics;
    cv::Mat pre_prame;
    int count = 0;
};
#endif
