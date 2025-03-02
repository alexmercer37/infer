#include "../inc/pcl.hpp"

Lcloud::Lcloud()
{
    source = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    source_downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    circle = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    inliers_circle = std::make_shared<pcl::PointIndices>();
    coefficients_circle = std::make_shared<pcl::ModelCoefficients>();
}

void Lcloud::getXYZPointCloud(const k4a::transformation &k4aTransformation, const k4a::calibration &k4aCalibration, const cv::Mat &cv_depthimg)
{
    clock_t start, end;
    // start = clock();

    k4a::image depthImage{nullptr};
    k4a::image xyzImage{nullptr};
    k4a::image pointCloud{nullptr};

    int width = k4aCalibration.color_camera_calibration.resolution_width;
    int height = k4aCalibration.color_camera_calibration.resolution_height;

    // int width = cv_depthimg.cols;
    // int height = cv_depthimg.rows;

    depthImage = k4a::image::create_from_buffer(K4A_IMAGE_FORMAT_DEPTH16, width, height, width * (int)sizeof(uint16_t), cv_depthimg.data, width * height * 2, nullptr, nullptr);
    xyzImage = k4aTransformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_COLOR);

    auto *xyzImageData = (int16_t *)(void *)xyzImage.get_buffer();

    for (int i = 0; i < width * height; i++)
    {
        if (xyzImageData[3 * i + 2] == 0 || xyzImageData[3 * i + 2] >= 5000) // 要4m内

            continue;
        if (i % 3 != 0)

            continue;

        pcl::PointXYZ point;

        point.x = xyzImageData[3 * i + 0];
        point.y = xyzImageData[3 * i + 1];
        point.z = xyzImageData[3 * i + 2];

        source->points.push_back(point);
    }
    std::cout << "pcl:点的个数" << source->size() << std::endl;
    pointCloud.reset();
    xyzImage.reset();
    depthImage.reset();
}

// void Lcloud::getPLY()
// {

//     Eigen::VectorXf coeff;

//     Eigen::Vector4f leaf_size{0.01, 0.01, 0.01, 0};

//     clock_t start, end;

//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

//     // pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>()); // 所有条件都满足（or为满足一个即可）

//     // // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 300.0)));
//     // // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -600.0)));

//     // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 5000.0)));
//     // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 300.0)));
//     // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 400.0)));
//     // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -400.0)));

//     // start = clock();

//     // filt.setCondition(range_cond);
//     // filt.setKeepOrganized(false);
//     // filt.setInputCloud(source);
//     // std::cout << source->size() << std::endl;
//     // filt.filter(*source);
//     // std::cout << source->size() << std::endl;

//     // end = clock();

//     // cout << "过滤：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

//     // if (source_downsampled->size() < 100)
//     // {
//     //   cout << "点云数目太少" << endl;
//     //   return;
//     // }
//     //
//     // start = clock();

//     vg.setInputCloud(source);
//     vg.setLeafSize(leaf_size);
//     vg.setMinimumPointsNumberPerVoxel(1);
//     vg.filter(*source_downsampled);
//     std::cout << "PCL:点的个数" << source_downsampled->size() << std::endl;
//     // end = clock();
//     // cout << "下采样：" << (double)(end - start) / CLOCKS_PER_SEC << endl;

//     // getMinMax3D(*source_downsampled, min_pt, max_pt);

//     // if (source_downsampled->size() < 70 || max_pt(1) < 40)
//     // {
//     // cout << "下采样点云数目太少" << endl;
//     //   return;
//     // }

//     // start = clock();

//     sor.setInputCloud(source_downsampled);
//     sor.setMeanK(50);
//     sor.setStddevMulThresh(1);
//     sor.filter(*source_downsampled);

//     // ror.setInputCloud(source);
//     // ror.setRadiusSearch(radius);
//     // ror.setMinNeighborsInRadius(amount);
//     // ror.filter(*cloud_ptr);

//     if (source_downsampled->size() < 40)
//     {
//         std::cout << "Not enough points in cloud to fit a circle!" << std::endl;
//         return;
//     }
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr circle3d(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(source_downsampled));
//     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//     pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(circle3d);
//     std::vector<int> ransac_inliers;
//     ransac.setDistanceThreshold(0.2);
//     ransac.setMaxIterations(10000);
//     ransac.computeModel();
//     ransac.getModelCoefficients(coeff);
//     // 为提取圆点
//     ransac.getInliers(ransac_inliers);
//     inliers->indices = ransac_inliers;
//     extract.setInputCloud(source_downsampled);
//     extract.setIndices(inliers);
//     extract.setNegative(false);
//     extract.filter(*source_downsampled);

//     // std::cout << "circle cloud: " << cloud_ptr->size() << std::endl;
//     std::cout << "RS : x = " << coeff[0] << ", RS  : y = " << coeff[1] << ", RS : z = " << coeff[2] << ", RS : r = " << coeff[3] << std::endl;

//     // circle_center = fitCircleLM(source_downsampled, 0.225, coeff);

//     // double degree = 35.0;
//     // double radians = degree * M_PI / 180.0;

//     // float x = circle_center.center[0] * 1000 - 287.01;
//     // float y = circle_center.center[1] * sin(radians) * 1000 + circle_center.center[2] * cos(radians) * 1000 - 324;

//     // std::cout << "x = " << x << " , y = " << y << std::endl;

//     // if (coeff[3] < 0.24 && coeff[3] > 0.19)
//     // {
//     //     uint8_t data[13] = {0};
//     //     data[0] = 0xff;
//     //     data[1] = 0xfe;
//     //     memcpy(&data[2], &x, sizeof(float));
//     //     memcpy(&data[6], &y, sizeof(float));
//     //     for (int i = 2; i < 10; i++)
//     //     {
//     //         data[10] += data[i];
//     //     }
//     //     data[11] = 0xaa;
//     //     data[12] = 0xdd;
//     //     uart.UART_SEND(data, 13);
//     // }
// }

Circle3D Lcloud::fitCircleLM(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, double radius, Eigen::VectorXf &coeff)
{
    Eigen::VectorXd x(3);
    x << coeff[0], coeff[1], coeff[2]; // Initial guess for center

    Functor functor(source, radius);
    Eigen::LevenbergMarquardt<Functor> lm(functor);
    lm.minimize(x);

    Circle3D circle;
    circle.center = Eigen::Vector3d(x(0), x(1), x(2));

    std::cout << "LM : x = " << circle.center[0] << " , LM : y = " << circle.center[1] << " , LM : z = " << circle.center[2] << std::endl;

    return circle;
}

void Lcloud::getPLY()
{

    Eigen::Vector4f leaf_size{20, 20, 20, 0};

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    // pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>()); // 所有条件都满足（or为满足一个即可）

    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 300.0)));
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -600.0)));

    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 5000.0)));
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 300.0)));
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 400.0)));
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::Ptr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -400.0)));

    // filt.setCondition(range_cond);
    // filt.setKeepOrganized(false);
    // filt.setInputCloud(source);
    // std::cout << source->size() << std::endl;
    // filt.filter(*source_downsampled);

    if (source->size() < 100)
    {
        std::cout << "点云数目太少" << std::endl;
        return;
    }

    vg.setInputCloud(source);
    vg.setLeafSize(leaf_size);
    vg.setMinimumPointsNumberPerVoxel(1);
    vg.filter(*source_downsampled);
    std::cout << "PCL:点的个数" << source_downsampled->size() << std::endl;

    sor.setInputCloud(source_downsampled);
    sor.setMeanK(100);
    sor.setStddevMulThresh(1);
    sor.filter(*source_downsampled);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(50);
    seg.setMaxIterations(20000);
    seg.setInputCloud(source_downsampled);
    seg.segment(*inliers_circle, *coefficients_circle);

    extract.setInputCloud(source_downsampled);
    extract.setIndices(inliers_circle);
    extract.setNegative(false);
    extract.filter(*source_downsampled);

    std::cout << source_downsampled->size() << std::endl;
    // if (circle->size() > 20)
    // {

    std::cout << coefficients_circle->values[0] << std::endl;
    std::cout << coefficients_circle->values[1] << std::endl;
    std::cout << coefficients_circle->values[2] << std::endl;
    // }

    // if (data[0] < 600 && data[0] > (-600) && data[1] < 500 && data[1] > (-200) && data[2] > 200 && data[2] < 2000)
    // {
    //     libtty_write(fd, data, buff);
    // }
}

void Lcloud::clearCloud()
{
    source->clear();
    source_downsampled->clear();

    circle->clear();

    inliers_circle->header.frame_id.clear();
    inliers_circle->indices.clear();

    coefficients_circle->header.frame_id.clear();
    coefficients_circle->values.clear();
}