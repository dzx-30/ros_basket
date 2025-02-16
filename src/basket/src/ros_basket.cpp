#include "../inc/ros_basket.hpp"

RosBasket::RosBasket(ros::NodeHandle &nh)
{
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 10);
    sub_cloud = nh.subscribe("pointcloud", 10, &RosBasket::clb, this);
    pub_basket = nh.advertise<sensor_msgs::PointCloud2>("basket", 10);
}

RosBasket::~RosBasket()
{
}

void RosBasket::K4a_Basket_Get()
{
    k4a.Image_to_Cv(*color_k4a_ptr, *depth_k4a_ptr);
    yolo.Yolov8_Seg_Enable(*engine_v8_seg_ptr);
    // yolo.Yolov8_Enable(*engine_v8_ptr);
    yolo.Single_Inference(*color_k4a_ptr, *objs_ptr);
    k4a.Value_Mask_to_Pcl(*cloud_seg_ptr, *objs_ptr);
    k4a.Color_With_Mask(*color_k4a_ptr, *objs_ptr);
    k4a.Depth_With_Mask(*depth_k4a_ptr, *objs_ptr);
    // cv::imshow("Color Seg", *(color_k4a_ptr));
    cv::imshow("Depth Seg", *(depth_k4a_ptr));
    cv::waitKey(1);

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*cloud_seg_ptr, *cloud_seg_ptr, valid_indices);
    std::cout << "Global PointCloud:" << cloud_seg_ptr->size() << std::endl;
    pcl::toROSMsg(*cloud_seg_ptr, cloud_msg);
    cloud_msg.header.frame_id = "odom";
    pub_cloud.publish(cloud_msg);
}

void RosBasket::clb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    cloud_in = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::VectorXf coeff;
    pcl::fromROSMsg(*msg, *cloud_in);
    pclprocess.Vg_Filter(0.03, cloud_in);
    pclprocess.Sor_Filter(50, 0.01, cloud_in);
    // std::cout << cloud_in->size() << std::endl;
    // pclprocess.Ror_Filter(35, 0.15, cloud_in);
    pclprocess.Circle_Extract(cloud_in, coeff);
    Draw_Circle(pclprocess.circle_center);
    // Draw_Circle(coeff);
    cv::imshow("Color Seg", *(color_k4a_ptr));
    cv::waitKey(1);

    pcl::toROSMsg(*cloud_in, basket_msg);
    basket_msg.header.frame_id = "odom";
    pub_basket.publish(basket_msg);
    // pcl::compute3DCentroid(*cloud_in, *centroid);
    // std::cout << "x:" << centroid->x()
    //           << ",y:" << centroid->y()
    //           << ",z:" << centroid->z() << std::endl;
}

void RosBasket::Draw_Circle(Eigen::VectorXf &coeff)
{
    if (coeff.size() < 3)
    {
        std::cerr << "Error: coeff size is less than 3!" << std::endl;
        return;
    }
    float fx = k4a.color_intrinsics.intrinsics.parameters.param.fx;
    float fy = k4a.color_intrinsics.intrinsics.parameters.param.fy;
    float cx = k4a.color_intrinsics.intrinsics.parameters.param.cx;
    float cy = k4a.color_intrinsics.intrinsics.parameters.param.cy;

    int u = static_cast<int>(fx * coeff[0] / coeff[2] + cx);
    int v = static_cast<int>(fy * coeff[1] / coeff[2] + cy);

    if (u >= 0 && u < k4a.image_k4a_depth_to_color.get_width_pixels() &&
        v >= 0 && v < k4a.image_k4a_depth_to_color.get_height_pixels())
    {

        std::cout << "Projected pixel coordinates: (" << u << ", " << v << ")" << std::endl;
    }
    else
    {
        std::cout << "Point outside image bounds." << std::endl;
    }

    cv::circle(*color_k4a_ptr, cv::Point(u, v), 5, cv::Scalar(0, 255, 0), -1);
}

void RosBasket::Draw_Circle(Circle3D circle_center)
{
    float fx = k4a.color_intrinsics.intrinsics.parameters.param.fx;
    float fy = k4a.color_intrinsics.intrinsics.parameters.param.fy;
    float cx = k4a.color_intrinsics.intrinsics.parameters.param.cx;
    float cy = k4a.color_intrinsics.intrinsics.parameters.param.cy;

    int u = static_cast<int>(fx * circle_center.center[0] / circle_center.center[2] + cx);
    int v = static_cast<int>(fy * circle_center.center[1] / circle_center.center[2] + cy);

    if (u >= 0 && u < k4a.image_k4a_depth_to_color.get_width_pixels() &&
        v >= 0 && v < k4a.image_k4a_depth_to_color.get_height_pixels())
    {

        std::cout << "Projected pixel coordinates: (" << u << ", " << v << ")" << std::endl;
    }
    else
    {
        std::cout << "Point outside image bounds." << std::endl;
    }

    cv::circle(*color_k4a_ptr, cv::Point(u, v), 5, cv::Scalar(0, 255, 0), -1);
}
