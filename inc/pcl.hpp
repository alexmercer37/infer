#ifndef PCL_HPP
#define PCL_HPP
#include "PclHead.hpp"
#include "camera.hpp"
#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>

struct Circle3D
{
    Eigen::Vector3d center;
    double radius;
};

struct Functor
{
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud;
    double radius;
    Functor(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double radius) : cloud(cloud), radius(radius) {}

    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
    {
        Eigen::Vector3d center(x(0), x(1), x(2));

        for (size_t i = 0; i < cloud->size(); ++i)
        {
            const auto &point = (*cloud)[i];
            fvec(i) = (Eigen::Vector3d(point.x, point.y, point.z) - center).norm() - radius;
        }

        return 0;
    }

    int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const
    {
        Eigen::Vector3d center(x(0), x(1), x(2));

        for (size_t i = 0; i < cloud->size(); ++i)
        {
            const auto &point = (*cloud)[i];
            Eigen::Vector3d diff(point.x - center.x(), point.y - center.y(), point.z - center.z());
            double norm = diff.norm();
            fjac(i, 0) = -diff.x() / norm;
            fjac(i, 1) = -diff.y() / norm;
            fjac(i, 2) = -diff.z() / norm;
        }

        return 0;
    }

    int inputs() const { return 3; } // Only optimize center (x, y, z)
    int values() const { return cloud->size(); }
};

class Lcloud
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_downsampled;
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // pcl::ConditionalRemoval<pcl::PointXYZ> filt;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers_circle;
    pcl::ModelCoefficients::Ptr coefficients_circle;

public:
    Circle3D circle_center;
    Lcloud();

    void getXYZPointCloud(const k4a::transformation &k4aTransformation, const k4a::calibration &k4aCalibration, const cv::Mat &cv_depthimg);
    void getPLY();
    void clearCloud();
    Circle3D fitCircleLM(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, double radius, Eigen::VectorXf &coeff);
};

#endif