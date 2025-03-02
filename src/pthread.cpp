/**
 * @file pthread.cpp
 * @author alexmercer37 (3450141407@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "../inc/main.hpp"
#include "../inc/pthread.hpp"
#include "../inc/PclHead.hpp"
// #include <python3.12/Python.h>
// #include <numpy/arrayobject.h>
RealSense *realsense = new RealSense;
Camera *camera = new Camera;
Detect *detect = new Detect;
Lcloud *lcloud = new Lcloud;

cv::Mat rgb_frame, depth_frame;
cv::Mat *rgb_ptr = new cv::Mat;
cv::Mat *depth_ptr = new cv::Mat;

k4a::capture capture;
k4a::device Device;
k4a::transformation k4aTransformation;
k4a::calibration k4aCalibration;

rs2::pipeline pipelines;
rs2::frameset frame;
rs2::align align_to_color(RS2_STREAM_COLOR);

uint16_t *depth_data;
uint16_t depth_width;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
std::shared_ptr<yolo::BoxArray> objs_ptr = std::make_shared<yolo::BoxArray>();

void *pthread::k4aUpdate(void *argc)
{
    camera->init_kinect(Device, capture, k4aTransformation, k4aCalibration);
    // int frame_count = 0;

    while (1)
    {
        // std::ostringstream filename;

        // filename << "/home/ddxy/Downloads/picture/frame_ " << std::setw(5) << std::setfill('0') << frame_count++ << ".jpg ";

        camera->picture_update(Device, capture);

        pthread_mutex_lock(&buff_mutex);

        rgb_ptr = camera->getpicture(Device, capture, rgb_frame, k4aTransformation);
        depth_ptr = camera->getdepth(Device, capture, depth_frame, k4aTransformation);

        // cv::imwrite(filename.str(), *rgb_ptr);

        // camera->getAngel(Device);

        pthread_mutex_unlock(&buff_mutex);

        cv::cvtColor(*rgb_ptr, *rgb_ptr, cv::COLOR_BGRA2BGR);

        pthread_mutex_lock(&buff_mutex);

        matBuff = std::make_shared<cv::Mat>(rgb_ptr->clone());
        depthBuff = std::make_shared<cv::Mat>(depth_ptr->clone());

        cvtColor(*matBuff, *greyBuff, cv::COLOR_BGR2GRAY);

        pthread_mutex_unlock(&buff_mutex);

        camera->camera_detect(*greyBuff);

        if (matBuff->empty())
        {
            throw std::runtime_error("Error: matBuff is empty!");
        }

        if (depthBuff->empty())
        {
            throw std::runtime_error("Error: depthBuff is empty!");
        }

        rgb_ptr->release();
        depth_ptr->release();

        usleep(100);
    }
    pthread_exit(NULL);
}

void *pthread::realsenseUpdate(void *argc)
{
    realsense->realsense_init(pipelines);

    while (1)
    {

        realsense->realsense_update(pipelines, frame, align_to_color);

        pthread_mutex_lock(&buff_mutex);

        rgb_ptr = realsense->get_realsense_rgb(frame);
        depth_ptr = realsense->get_realsense_depth(frame);

        pthread_mutex_unlock(&buff_mutex);

        pthread_mutex_lock(&buff_mutex);

        matBuff = std::make_shared<cv::Mat>(rgb_ptr->clone());
        depthBuff = std::make_shared<cv::Mat>(depth_ptr->clone());

        pthread_mutex_unlock(&buff_mutex);

        if (matBuff->empty())
        {
            throw std::runtime_error("Error: matBuff is empty!");
        }

        if (depthBuff->empty())
        {
            throw std::runtime_error("Error: depthBuff is empty!");
        }

        rgb_ptr->release();
        depth_ptr->release();

        cv::imshow("dzx", *matBuff);
        cv::waitKey(1);

        usleep(100);
    }
    pthread_exit(NULL);
}

void *pthread::create_infer(void *argc)
{
    auto yolo = yolo::load("/home/ddxy/dxy_infer-master/workspace/engine/1.engine", yolo::Type::V8);
    if (yolo == nullptr)
    {
        throw std::runtime_error("Loading yolo failed");
        return nullptr;
    }

    std::shared_ptr<cv::Mat> output;
    detect->setYolo(yolo);

    while (1)
    {
        while (!depthBuff->empty())
        {
            pthread_mutex_lock(&buff_mutex);
            *objs_ptr = detect->single_inference(matBuff, yolo);

            if (objs_ptr != nullptr)
            {
                detect->detect_boxes(*objs_ptr, *matBuff, *depthBuff, k4aTransformation, k4aCalibration);
            }

            pthread_mutex_unlock(&buff_mutex);
            usleep(100);

            cv::imshow("k4a", *matBuff);
            cv::waitKey(1);
        }
    }
    pthread_exit(NULL);
}

void *pthread::create_infer_seg(void *argc)
{
    auto yolo = yolo::load("/home/ddxy/dxy_infer-master/workspace/engine/best.engine", yolo::Type::V8Seg);
    if (yolo == nullptr)
    {
        throw std::runtime_error("Loading yolo_seg failed");
        return nullptr;
    }

    detect->setYolo(yolo);
    cv::Mat *output;
    __u8 data;

    while (1)
    {
        clock_t start, end;
        auto yoloStart = std::chrono::system_clock::now();

        while (!matBuff->empty())
        {
            pthread_mutex_lock(&buff_mutex);
            *objs_ptr = detect->seg_inference(matBuff, yolo);
            pthread_mutex_unlock(&buff_mutex);
            usleep(100);

            if (objs_ptr != nullptr)
            {
                camera->Value_Mask_to_Pcl(*cloud_seg_ptr, *objs_ptr);
                // lcloud->getPLY(data, cloud_seg_ptr);
            }
            else
                std::cout << "Error: objs is empty!" << std::endl;
        }

        usleep(100);

        auto yoloEnd = std::chrono::system_clock::now();
        auto yoloDuration = std::chrono::duration_cast<std::chrono::microseconds>(yoloEnd - yoloStart);
        std::cout << "yolo:" << double(yoloDuration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << "s" << std::endl;
    }
    pthread_exit(NULL);
}

void *pthread::usb_camera_infer(void *argc)
{
    auto yolo = yolo::load("/home/ddxy/Downloads/dxy_infer-master/workspace/engine/test.engine", yolo::Type::V8);
    if (yolo == nullptr)
    {
        throw std::runtime_error("Loading usb_yolo failed");
        return nullptr;
    }

    cv::VideoCapture capture(0);
    detect->setYolo(yolo);

    std::shared_ptr<cv::Mat> output;
    std::shared_ptr<cv::Mat> frame;
    frame.reset(new cv::Mat);

    while (1)
    {

        capture.read(*frame);

        pthread_mutex_lock(&buff_mutex);
        // output = detect->single_inference(frame, yolo);
        pthread_mutex_unlock(&buff_mutex);
        usleep(100);

        if (output != nullptr)
        {
            cv::imshow("USB_CAMERA", *output);
            cv::waitKey(1);
        }
        else
            std::cout << "Error: usb_camera_output is empty!" << std::endl;
    }
    pthread_exit(NULL);
}

#ifndef python_test_succeed

void *pthread::detect_python(void *argc)
{
    Py_Initialize();
    _import_array();

    PyRun_SimpleString("import sys");
    PyRun_SimpleString("import os");
    PyRun_SimpleString("sys.path.append(\'/home/dxy/Downloads/yolov5-6.0_two_labels\')");
    PyRun_SimpleString("sys.path.append(\'.\')");
    PyRun_SimpleString("import cv2");
    PyRun_SimpleString("import numpy");
    PyRun_SimpleString("import torch");
    PyRun_SimpleString("import time");
    PyRun_SimpleString("import serial");

    PyObject *m_PyModule = PyImport_ImportModule("detect_ball");
    PyObject *m_PyDict = PyModule_GetDict(m_PyModule);
    PyObject *load_model = PyDict_GetItemString(m_PyDict, "load_model");
    PyObject *model = PyObject_CallObject(load_model, NULL);

    while (1)
    {
        if (load_model != NULL)
        {
            pthread_mutex_lock(&buff_mutex);
            int r = (*matBuff).rows;
            int c = (*matBuff).cols;
            int chnl = (*matBuff).channels();
            int nElem = r * c * chnl;

            uchar *m = new uchar[nElem];

            std::memcpy(m, (*matBuff).data, nElem * sizeof(uchar));

            npy_intp mdim[] = {r, c, chnl};

            PyObject *mat = (PyObject *)PyArray_SimpleNewFromData(chnl, mdim, NPY_UINT8, (void *)m);
            pthread_mutex_unlock(&buff_mutex);

            PyObject *detect_fun = PyDict_GetItemString(m_PyDict, "detect");
            PyObject *detect_args = PyTuple_Pack(2, model, mat);
            PyObject *result = PyObject_CallObject(detect_fun, detect_args);

            // if (PyList_Check(result))
            // {
            //     float SizeOfList = PyList_Size(result);
            //     for (int i = 0; i < SizeOfList; i++)
            //     {
            //         PyObject *ListItem = PyList_GetItem(result, i);
            //         cout << ListItem << endl;
            //         Py_DECREF(ListItem);
            //     }
            // }

            Py_XDECREF(mat);
            Py_XDECREF(result);
            Py_XDECREF(detect_args);

            delete[] m;
        }
    }
}
#endif