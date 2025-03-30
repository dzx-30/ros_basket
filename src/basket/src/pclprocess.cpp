#include "../inc/pclprocess.hpp"
com::UART uart;

void PclProcess::Vg_Filter(float leafsize, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    vg.setInputCloud(cloud_ptr);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*cloud_ptr);
    std::cout << cloud_ptr->size() << std::endl;
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

// TODO:效果并不好，不确定是否因为理论高度限制与实际高度不符合
void PclProcess::height(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
    double radians = degree * M_PI / 180.0;

    for (int i = 0; i < cloud_ptr->size(); i++)
    {
        float x = cloud_ptr->points[i].x * 1000 + OX;
        float y = cloud_ptr->points[i].y * sin(radians) * 1000 + cloud_ptr->points[i].z * cos(radians) * 1000 + OY;
        float z = cloud_ptr->points[i].z * sin(radians) * 1000 + cloud_ptr->points[i].y * cos(radians) * 1000 + OZ;
        cloud_ptr->points[i].x = x / 1000;
        cloud_ptr->points[i].y = y / 1000;
        cloud_ptr->points[i].z = z / 1000;
    }
    std::cout << cloud_ptr->size() << std::endl;

    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 2.23)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 2.53)));
    pcl::ConditionalRemoval<pcl::PointXYZ> cr;
    cr.setCondition(range_cond);
    cr.setInputCloud(cloud_ptr);
    cr.filter(*cloud_ptr);
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
    ransac.setDistanceThreshold(rsdistance);
    ransac.setMaxIterations(rsmax);
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

    circle_center = fitCircleLM(cloud_ptr, 0.225, coeff);

    double radians = degree * M_PI / 180.0;

    float x = circle_center.center[0] * 1000 + OX;
    float y = circle_center.center[1] * sin(radians) * 1000 + circle_center.center[2] * cos(radians) * 1000 + OY;

    // float x = circle_center.center[0]*1000.0;
    // float y = circle_center.center[1]*1000.0;
    // TODO:方法3：拟合圆得到圆心
    std::cout << "x = " << x << " , y = " << y << std::endl;

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

Circle3D PclProcess::fitCircleLM(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, double radius, Eigen::VectorXf &coeff)
{
    Eigen::VectorXd x(3);
    x << coeff[0], coeff[1], coeff[2]; // Initial guess for center

    Functor functor(cloud_ptr, radius);
    Eigen::LevenbergMarquardt<Functor> lm(functor);
    lm.minimize(x);

    Circle3D circle;
    circle.center = Eigen::Vector3d(x(0), x(1), x(2));

    return circle;
}

PclProcess::PclProcess()
{
}

PclProcess::~PclProcess()
{
}
