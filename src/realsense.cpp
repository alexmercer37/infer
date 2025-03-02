/**
 * @file realsense.cpp
 * @author alexmercer37 (3416181310@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-29
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "../inc/realsense.hpp"

void RealSense::realsense_init(rs2::pipeline pipe)
{

    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 90);
    std::cout << "Done:init depth camera." << std::endl;

    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    std::cout << "Done:init color camera." << std::endl;

    rs2::pipeline_profile profile = pipe.start(cfg);
    std::cout << "Done:start camera." << std::endl;

    // rs2::align align_to_color(RS2_STREAM_COLOR);

    int iAuto = 0;
    while (1)
    {
        if (frames = pipe.wait_for_frames())
            std::cout << iAuto << ". Capture several frames to give auto-exposure" << std::endl;
        if (iAuto < 30)
        {
            iAuto++;
            continue;
        }
        else
        {
            std::cout << "Done: auto-exposure" << std::endl;
            break;
        }
    }
}

void RealSense::realsense_update(rs2::pipeline pipe, rs2::frameset &frames, rs2::align align_to_color)
{

    frames = pipe.wait_for_frames();
    frames = align_to_color.process(frames);
}

cv::Mat *RealSense::get_realsense_depth(rs2::frameset frames)
{
    rs2::depth_frame depth_stream = frames.get_depth_frame();

    const int width = depth_stream.as<rs2::video_frame>().get_width();
    const int height = depth_stream.as<rs2::video_frame>().get_height();

    cv::Mat *depth_image = new cv::Mat(cv::Size(width, height), CV_16U, (void *)depth_stream.get_data(), cv::Mat::AUTO_STEP);

    return depth_image;
}

cv::Mat *RealSense::get_realsense_rgb(rs2::frameset frames)
{
    rs2::video_frame color_stream = frames.get_color_frame();

    const int width = color_stream.as<rs2::video_frame>().get_width();
    const int height = color_stream.as<rs2::video_frame>().get_height();

    cv::Mat *color_image = new cv::Mat(cv::Size(width, height), CV_8UC3, (void *)color_stream.get_data(), cv::Mat::AUTO_STEP);

    return color_image;
}

void RealSense::detect_realsense()
{
    rs2::context context;
    rs2::device_list devs = context.query_devices();
    int device_num = devs.size();
    std::cout << "获取相机设备号：" << device_num << std::endl;
    if (!device_num)
    {
        throw std::runtime_error("Error: camera do not update!");
    }
}