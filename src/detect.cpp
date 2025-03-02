/**
 * @file detect.cpp
 * @author alexmercer37 (3450141407@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "../inc/detect.hpp"

extern Filter *filter;

yolo::Image cvimg(const cv::Mat &image)
{
    return yolo::Image(image.data, image.cols, image.rows);
}

void Detect::Time()
{
    clock_t start, end;
    auto yoloStart = std::chrono::system_clock::now();
    auto yoloEnd = std::chrono::system_clock::now();
    auto yoloDuration = std::chrono::duration_cast<std::chrono::microseconds>(yoloEnd - yoloStart);
    std::cout << "yolo:" << double(yoloDuration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << "s" << std::endl;
}

void Detect::perf()
{
    int max_infer_batch = 16;
    int batch = 16;
    std::vector<cv::Mat> images{cv::imread("inference/car.jpg"), cv::imread("inference/gril.jpg"),
                                cv::imread("inference/group.jpg")};

    for (int i = images.size(); i < batch; ++i)
        images.push_back(images[i % 3]);

    cpm::Instance<yolo::BoxArray, yolo::Image, yolo::Infer> cpmi;
    bool ok = cpmi.start([]
                         { return yolo::load("yolov8n.transd.engine", yolo::Type::V8); },
                         max_infer_batch);

    if (!ok)
        return;

    std::vector<yolo::Image> yoloimages(images.size());
    std::transform(images.begin(), images.end(), yoloimages.begin(), cvimg);

    trt::Timer timer;
    for (int i = 0; i < 5; ++i)
    {
        timer.start();
        cpmi.commits(yoloimages).back().get();
        timer.stop("BATCH16");
    }

    for (int i = 0; i < 5; ++i)
    {
        timer.start();
        cpmi.commit(yoloimages[0]).get();
        timer.stop("BATCH1");
    }
}

void Detect::batch_inference()
{
    std::vector<cv::Mat> images{cv::imread("inference/car.jpg")};
    auto yolo = yolo::load("/home/ddxy/下载/dxy-infer/yolov8n.transd.engine", yolo::Type::V5);
    if (yolo == nullptr)
        return;

    std::vector<yolo::Image> yoloimages(images.size());
    std::transform(images.begin(), images.end(), yoloimages.begin(), cvimg);
    auto batched_result = yolo->forwards(yoloimages);
    for (int ib = 0; ib < (int)batched_result.size(); ++ib)
    {
        auto &objs = batched_result[ib];
        auto &image = images[ib];
        for (auto &obj : objs)
        {
            uint8_t b, g, r;
            std::tie(b, g, r) = yolo::random_color(obj.class_label);
            cv::rectangle(image, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom),
                          cv::Scalar(b, g, r), 5);

            auto name = cocolabels[obj.class_label];
            auto caption = cv::format("%s %.2f", name, obj.confidence);
            int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
            cv::rectangle(image, cv::Point(obj.left - 3, obj.top - 33),
                          cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
            cv::putText(image, caption, cv::Point(obj.left, obj.top - 5), 0, 1, cv::Scalar::all(0), 2,
                        16);
        }
        printf("Save result to Result.jpg, %d objects\n", (int)objs.size());
        cv::imwrite(cv::format("Result%d.jpg", ib), image);
    }
}

<<<<<<< HEAD
yolo::BoxArray Detect::single_inference(std::shared_ptr<cv::Mat> image, std::shared_ptr<yolo::Infer> yolo)
=======
std::shared_ptr<cv::Mat> Detect::single_inference(std::shared_ptr<cv::Mat> image, std::shared_ptr<yolo::Infer> yolo, cv::KalmanFilter &kf)
>>>>>>> 32b05f84d0eac917060bc12f21c72abf15bb5887
{

    auto objs = yolo->forward(yolo::Image((*image).data, (*image).cols, (*image).rows));
    std::ostringstream oss;

    for (auto &obj : objs)
    {

#ifndef use_random_color
        uint8_t b, g, r;
        std::tie(b, g, r) = yolo::random_color(obj.class_label);
        cv::rectangle(*image, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom), cv::Scalar(b, g, r), 5);
#endif

#ifndef use_changed_color
        std::vector<cv::Scalar> colors;
        srand(time(0));
        for (int i = 0; i < 80; i++)
        {
            int b = rand() % 256;
            int g = rand() % 256;
            int r = rand() % 256;
            colors.push_back(cv::Scalar(b, g, r));
        }
        cv::Scalar color = colors[obj.class_label];
        cv::rectangle(*image, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom), color, 5);
        // cv::circle(*image, cv::Point(obj.left + (obj.right - obj.left) / 2, obj.bottom + (obj.top - obj.bottom) / 2), 5, cv::Scalar(255, 0, 0), -1);

        std::vector<cv::Point2f> measurements;
        measurements.push_back(cv::Point2f(obj.left + (obj.right - obj.left) / 2, obj.bottom + (obj.top - obj.bottom) / 2));
        filter->filter_predict(measurements, kf, *image);

        std::cout << "success!" << std::endl;
#endif

        // auto name = cocolabels[obj.class_label];
        // oss << name << " " << std::fixed << std::setprecision(2) << obj.confidence;
        // auto caption = oss.str();
        // int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;

        // cv::rectangle(*image, cv::Point(obj.left - 3, obj.top - 33), cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
        // cv::putText(*image, caption, cv::Point(obj.left, obj.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);
    }

    cv::imshow("k4a", *image);
    cv::waitKey(1);

    return objs;
}

yolo::BoxArray Detect::seg_inference(std::shared_ptr<cv::Mat> image, std::shared_ptr<yolo::Infer> yolo)

{

    auto objs = yolo->forward(yolo::Image((*image).data, (*image).cols, (*image).rows));
    cv::Mat *output;

    for (auto &obj : objs)
    {
        if (obj.left >= 0 && obj.right < (*image).cols && obj.top >= 0 && obj.bottom <= (*image).rows)
        {
            uint8_t b, g, r;
            std::tie(b, g, r) = yolo::random_color(obj.class_label);

            // cv::rectangle(*image, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom), cv::Scalar(b, g, r), 5);

            output = new cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);

            cv::Mat mask = cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
            mask.convertTo(mask, CV_8UC1);
            cv::resize(mask, mask, cv::Size(obj.right - obj.left, obj.bottom - obj.top), 0, 0, cv::INTER_LINEAR);
            cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);

            mask.copyTo((*image)(cv::Rect(obj.left, obj.top, obj.right - obj.left, obj.bottom - obj.top)));
        }
    }

    cv::imshow("test", *image);
    cv::waitKey(1);

    return objs;
}

void Detect::detect_boxes(yolo::BoxArray &bboxes, cv::Mat rgb, cv::Mat &cv_depth, k4a::transformation &k4aTransformation, k4a::calibration &k4aCalibration)
{
    int imgHeight = rgb.rows;
    int imgWidth = rgb.cols;

    int left, top, right, bottom;

    cv::Rect select;

    Lcloud lcloud;

    uint8_t r, g, b;

    if (bboxes.empty() || cv_depth.empty())
        return;
    if ((cv_depth.cols != 1280 && cv_depth.rows != 720))
        return;

    for (auto &box : bboxes)
    {

        left = (int)box.left;
        if (left <= 0)
            left = 0;

        top = (int)box.top;
        if (top <= 0)
            top = 0;

        right = (int)box.right;
        if (right > imgWidth)
            right = imgWidth;

        bottom = (int)box.bottom;
        if (bottom > imgHeight)
            bottom = imgHeight;
    }

    select = cv::Rect(left, top, right - left, bottom - top);

    cv::rectangle(rgb, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(b, g, r), 3);

    cv::Mat mask = cv::Mat::zeros(rgb.size(), CV_8UC1);
    cv::Mat output;
    cv::cvtColor(rgb, output, cv::COLOR_BGR2HSV);
    cv::inRange(output, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), mask);

    cv::Mat depthCut = cv::Mat::zeros(cv::Size(imgWidth, imgHeight), CV_16U);

    cv::Mat maskROI = mask(select);

    cv_depth(select).copyTo(depthCut(select), maskROI);

    cv::imshow("mask", maskROI);
    cv::waitKey(1);

    lcloud.getXYZPointCloud(k4aTransformation, k4aCalibration, depthCut);

    lcloud.getPLY();

    lcloud.clearCloud();

    mask.release();
    maskROI.release();
}
