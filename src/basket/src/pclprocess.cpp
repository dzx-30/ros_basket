#include "../inc/pclprocess.hpp"
com::UART uart;

void PclProcess::Vg_Filter(float leafsize, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    vg.setInputCloud(cloud_ptr);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*cloud_ptr);
}

void PclProcess::Sor_Filter(int amount, float std, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{

    sor.setInputCloud(cloud_ptr);
    sor.setMeanK(amount);
    sor.setStddevMulThresh(std);
    sor.filter(*cloud_ptr);
}

// TODO:delete
void PclProcess::Ror_Filter(int amount, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{

    ror.setInputCloud(cloud_ptr);
    ror.setRadiusSearch(radius);
    ror.setMinNeighborsInRadius(amount);
    ror.filter(*cloud_ptr);
}

void PclProcess::Circle_Extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, Eigen::VectorXf &coeff)
{
    if (cloud_ptr->size() < 20)
    {
        std::cout << "Not enough points in cloud to fit a circle!" << std::endl;
        return;
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr circle3d(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cloud_ptr));
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(circle3d);
    std::vector<int> ransac_inliers;
    ransac.setDistanceThreshold(0.2);
    ransac.setMaxIterations(10000);
    ransac.computeModel();
    ransac.getModelCoefficients(coeff);
    // 为提取圆点
    ransac.getInliers(ransac_inliers);
    inliers->indices = ransac_inliers;
    if (inliers->indices.empty())
    {
        return;
    }
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_ptr);

    // std::cout << "circle cloud: " << cloud_ptr->size() << std::endl;
    std::cout << "RS : x = " << coeff[0] << ", RS  : y = " << coeff[1] << ", RS : z = " << coeff[2] << ", RS : r = " << coeff[3] << std::endl;

    circle_center = fitCircleLM(cloud_ptr, 0.225, coeff);

    double degree = 35.0;
    double radians = degree * M_PI / 180.0;

    float x = circle_center.center[0] * 1000 + 487.01;
    float y = circle_center.center[1] * sin(radians) * 1000 + circle_center.center[2] * cos(radians) * 1000 + 324;

    std::cout << "x = " << x << " , y = " << y << std::endl;

    // if (coeff[3] < 0.24 && coeff[3] > 0.19)
    {
        uint8_t data[13] = {0};
        data[0] = 0xff;
        data[1] = 0xfe;
        memcpy(&data[2], &x, sizeof(float));
        memcpy(&data[6], &y, sizeof(float));
        for (int i = 2; i < 10; i++)
        {
            data[10] += data[i];
        }
        data[11] = 0xaa;
        data[12] = 0xdd;
        uart.UART_SEND(data, 13);
    }
}

Circle3D PclProcess::fitCircleLM(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, double radius, Eigen::VectorXf &coeff)
{
    Eigen::VectorXd x(3);
    x << coeff[0], coeff[1], coeff[2]; // Initial guess for center

    Functor functor(cloud_ptr, radius);
    Eigen::LevenbergMarquardt<Functor> lm(functor);
    lm.minimize(x);

    Circle3D circle;
    circle.center = Eigen::Vector3d(x(0), x(1), x(2));

    std::cout << "LM : x = " << circle.center[0] << " , LM : y = " << circle.center[1] << " , LM : z = " << circle.center[2] << std::endl;

    return circle;
}

PclProcess::PclProcess()
{
}

PclProcess::~PclProcess()
{
}
