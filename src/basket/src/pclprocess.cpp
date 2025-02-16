#include "../inc/pclprocess.hpp"

void PclProcess::Input_PointCloud(std::string &pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(pcd_path, *cloud_ptr) == -1)
    {
        PCL_ERROR("Couldn't Read File\n");
        return;
    }
    else
    {
        std::cout << "Input Cloud Size:" << cloud_ptr->size() << std::endl;
    }
}

void PclProcess::Vg_Filter(float leafsize, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    vg.setInputCloud(cloud_ptr);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*cloud_ptr);
    // std::cout << "Vg PointCloud Size:" << cloud_ptr->size() << std::endl;
}

void PclProcess::Sor_Filter(int amount, float std, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{

    sor.setInputCloud(cloud_ptr);
    sor.setMeanK(amount);
    sor.setStddevMulThresh(std);
    sor.filter(*cloud_ptr);
    // std::cout << "Sor PointCloud Size:" << cloud_ptr->size() << std::endl;
}

void PclProcess::Ror_Filter(int amount, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{

    ror.setInputCloud(cloud_ptr);
    ror.setRadiusSearch(radius);
    ror.setMinNeighborsInRadius(amount);
    ror.filter(*cloud_ptr);
    // std::cout << "Ror PointCloud Size:" << cloud_ptr->size() << std::endl;
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
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_ptr);

    std::cout << "circle cloud: " << cloud_ptr->size() << std::endl;
    std::cout << "RS : x = " << coeff[0] << ", RS  : y = " << coeff[1] << ", RS : z = " << coeff[2] << ", RS : r = " << coeff[3] << std::endl;

    circle_center = fitCircleLM(cloud_ptr, 0.225, coeff);
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