#include "../inc/ros_basket.hpp"

RosBasket::RosBasket(ros::NodeHandle &nh)
{
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 10);
    sub_cloud = nh.subscribe("pointcloud", 10, &RosBasket::clb, this);
    pub_basket = nh.advertise<sensor_msgs::PointCloud2>("basket", 10);
    pub_center = nh.advertise<sensor_msgs::PointCloud2>("center", 10);
}

RosBasket::~RosBasket()
{
}

void RosBasket::K4a_Basket_Get()
{
    k4a.Image_to_Cv(*color_k4a_ptr, *depth_k4a_ptr);
    yolo.Single_Inference(*color_k4a_ptr, *objs_ptr, yol);
    k4a.Value_Mask_to_Pcl(*cloud_seg_ptr, *objs_ptr);
    k4a.Color_With_Mask(*color_k4a_ptr, *objs_ptr);

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*cloud_seg_ptr, *cloud_seg_ptr, valid_indices);
    std::cout << "Global PointCloud:" << cloud_seg_ptr->size() << std::endl;
    pcl::toROSMsg(*cloud_seg_ptr, cloud_msg);
    cloud_msg.header.frame_id = "map";
    pub_cloud.publish(cloud_msg);
}

void RosBasket::clb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    cloud_in = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::VectorXf coeff;
    pcl::fromROSMsg(*msg, *cloud_in);
    pclprocess.Vg_Filter(leafsize, cloud_in);
    pclprocess.Sor_Filter(amount, sordistance, cloud_in);
    // pclprocess.height(cloud_in);

    // TODO:最近点
    if (cloud_in->size() >= 10)
    {
        std::sort(cloud_in->begin(), cloud_in->end(), [](const auto &a, const auto &b)
                  { return (a.x * a.x + a.y * a.y + a.z * a.z) < (b.x * b.x + b.y * b.y + b.z * b.z); });
        float x = 0, y = 0, z = 0;
        for (int i = 0; i < 10; i++)
        {
            x += cloud_in->points[i].x;
            y += cloud_in->points[i].y;
            z += cloud_in->points[i].z;
        }
        x = x / 10;
        y = y / 10;
        z = z / 10;
        double radians2 = degree * M_PI / 180.0;
        x = x * 1000 + OX;
        y = y * sin(radians2) * 1000 + z * cos(radians2) * 1000 + OY;
        float l = sqrt(x * x + y * y);
        int i = 1;
        // if (x > 0)
        //     i = 1;
        // else
        //     i = -1;
        x = x + i * 225 * x / l;
        y = y + 225 * y / l;
        // TODO:方法1：最近点得到圆心
        std::cout << "closest.x = " << x << " , closest.y = " << y << " , closest.z = " << z << std::endl;
    }

    // TODO:圆质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_in, centroid);
    double radians = degree * M_PI / 180.0;
    float x = centroid[0] * 1000 + OX;
    float y = centroid[1] * sin(radians) * 1000 + centroid[2] * cos(radians) * 1000 + OY;
    // TODO:方法2：计算质心得到圆心
    std::cout << "centroidX : " << x << " , centroidY : " << y << " , centroidZ : " << centroid[2] << std::endl;

    // 拟合圆
    pclprocess.Circle_Extract(cloud_in, coeff);
    Draw_Circle(pclprocess.circle_center);

    center.clear();
    if (pclprocess.circle_center.center.size() != 0)
    {
        center.push_back(pcl::PointXYZ(pclprocess.circle_center.center[0], pclprocess.circle_center.center[1], pclprocess.circle_center.center[2]));
        pcl::toROSMsg(center, center_msg);
        center_msg.header.frame_id = "map";
        pub_center.publish(center_msg);
    }

    cv::imshow("Color Seg", *(color_k4a_ptr));
    cv::waitKey(1);

    pcl::toROSMsg(*cloud_in, basket_msg);
    basket_msg.header.frame_id = "map";
    pub_basket.publish(basket_msg);
}

void RosBasket::Draw_Circle(Circle3D circle_center)
{
    float fx = k4a.color_intrinsics.intrinsics.parameters.param.fx;
    float fy = k4a.color_intrinsics.intrinsics.parameters.param.fy;
    float cx = k4a.color_intrinsics.intrinsics.parameters.param.cx;
    float cy = k4a.color_intrinsics.intrinsics.parameters.param.cy;

    int u = static_cast<int>(fx * circle_center.center[0] / circle_center.center[2] + cx);
    int v = static_cast<int>(fy * circle_center.center[1] / circle_center.center[2] + cy);

    cv::circle(*color_k4a_ptr, cv::Point(u, v), 5, cv::Scalar(0, 255, 0), -1);
}
