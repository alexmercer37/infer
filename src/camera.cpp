/**
 * @file camera.cpp
 * @author alexmercer37 (3450141407@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "../inc/camera.hpp"

#define PI 3.1415926535

using namespace std;
using namespace k4a;
using namespace cv;
int fd;
UART uart;

void Camera::init_kinect(k4a::device &device, k4a::capture &capture, k4a::transformation &k4aTransformation, k4a::calibration &k4aCalibration)
{

  device_count = device::get_installed_count();
  if (device_count == 0)
  {
    cout << "Error:no K4A devices found." << endl;
    return;
  }
  else
  {
    cout << "Found" << device_count << "connected devices." << endl;
  }

  device = device::open(K4A_DEVICE_DEFAULT);
  // device = device::open(0);
  cout << "Done:open device." << endl;
  init = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  init.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  init.color_resolution = K4A_COLOR_RESOLUTION_720P;
  init.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
  init.camera_fps = K4A_FRAMES_PER_SECOND_30;
  init.synchronized_images_only = true;

  device.start_cameras(&init);
  cout << "Done:start camera." << endl;
  device.start_imu();
  cout << "Done:start imu." << endl;

  k4aCalibration = device.get_calibration(init.depth_mode, init.color_resolution);
  k4aTransformation = k4a::transformation(k4aCalibration);
  color_intrinsics = k4aCalibration.color_camera_calibration;

  int iAuto = 0;
  while (1)
  {
    if (device.get_capture(&capture))
      cout << iAuto << ". Capture several frames to give auto-exposure" << endl;
    if (iAuto < 30)
    {
      iAuto++;
      continue;
    }
    else
    {
      cout << "Done: auto-exposure" << endl;
      break;
    }
  }
}

void Camera::getAngel(k4a::device &device)
{
  __u8 buff = 0x01;
  double angle[3] = {0};

  k4a_imu_sample_t *imu_sample = new k4a_imu_sample_t;
  device.get_imu_sample(imu_sample, std::chrono::milliseconds(1000));

  if (device.get_imu_sample(imu_sample, std::chrono::milliseconds(100)))
    cout << "imu success" << endl;
  else
    cout << "imu error" << endl;

  if (imu_sample != NULL)
  {
    // cout << imu_sample->temperature << endl;
    angle[0] = {-atan(imu_sample->acc_sample.xyz.x / pow(pow(imu_sample->acc_sample.xyz.z, 2) + pow(imu_sample->acc_sample.xyz.y, 2), 0.5)) * 180 / PI};
    cout << angle[0] << endl;
    sleep(0.01);
    uart.Libtty_Write(fd, angle, buff);
  }
}

void Camera::picture_update(k4a::device &device, k4a::capture &capture)
{
  device.get_capture(&capture, std::chrono::milliseconds(100));
}

cv::Mat *Camera::getpicture(k4a::device &device, k4a::capture &capture, cv::Mat &cv_color, k4a::transformation &k4aTransformation)
{

  k4a_color = capture.get_color_image();

  cv::Mat *output;
  output = new Mat(k4a_color.get_height_pixels(), k4a_color.get_width_pixels(), CV_8UC4, k4a_color.get_buffer());

  return output;
}

cv::Mat *Camera::getdepth(k4a::device &device, k4a::capture &capture, cv::Mat &cv_depth, k4a::transformation &k4aTransformation)
{

  k4a_depth = capture.get_depth_image();

  cv::Mat *output;

  k4a_tf_depth = k4aTransformation.depth_image_to_color_camera(k4a_depth);

  output = new Mat(k4a_tf_depth.get_height_pixels(), k4a_tf_depth.get_width_pixels(), CV_16U, k4a_tf_depth.get_buffer());
  // (*cv_depth1).convertTo(cv_depth, CV_8U, 1);

  return output;
}
cv::Mat *Camera::getir(k4a::device &device, k4a::capture &capture, cv::Mat &cv_infrared, k4a::transformation &k4aTransformation)
{

  k4a_infrared = capture.get_ir_image();

  cv::Mat *output;

  output = new Mat(k4a_infrared.get_height_pixels(), k4a_infrared.get_width_pixels(), CV_16U, k4a_infrared.get_buffer());
  (*output).convertTo(cv_infrared, CV_8U, 1);
  return output;
}

void Camera::stopCamera()
{
  device.close();
}

void Camera::camera_detect(cv::Mat cur_frame)
{

  if (pre_prame.empty())
  {
    pre_prame = cur_frame.clone();
  }

  cv::Mat diff;
  cv::absdiff(pre_prame, cur_frame, diff);

  int frame_diff = cv::countNonZero(diff);
  // cout << frame_diff << endl;

  if (frame_diff == 0)
  {
    count++;
  }
  else
  {
    count = 0;
  }

  if (count >= 10)
  {
    throw std::runtime_error("Error: camera do not update!");
  }

  pre_prame = cur_frame.clone();
}

void Camera::Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud, yolo::BoxArray &objs)
{
  cloud.clear();

  if (objs.empty())
  {
    // std::cout << "Error: objs is empty!" << std::endl;
    return;
  }

  uint16_t *depth_data = (uint16_t *)k4a_tf_depth.get_buffer();

  yolo::Box boxBest = objs[0];
  for (auto &box : objs)
  {
    if (box.confidence > boxBest.confidence)
      boxBest = box;
  }

  for (int v = boxBest.top; v < boxBest.bottom; v++)
  {
    for (int u = boxBest.left; u < boxBest.right; u++)
    {
      if (boxBest.seg->data[u, v] == 0)
      {
        float depth_value = static_cast<float>(depth_data[v * k4a_tf_depth.get_width_pixels() + u] / 1000.0);
        if (depth_value != 0)
        {
          float x = (u - color_intrinsics.intrinsics.parameters.param.cx) * depth_value / color_intrinsics.intrinsics.parameters.param.fx;
          float y = (v - color_intrinsics.intrinsics.parameters.param.cy) * depth_value / color_intrinsics.intrinsics.parameters.param.fy;
          float z = depth_value;
          cloud.push_back(pcl::PointXYZ(x, y, z));
        }
      }
    }
  }
}
