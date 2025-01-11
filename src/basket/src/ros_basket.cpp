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
    yolo.Single_Inference(*color_k4a_ptr, *objs_ptr);
    k4a.Value_Mask_to_Pcl(*cloud_seg_ptr, *objs_ptr);
    // k4a.Color_With_Mask(*color_k4a_ptr, *objs_ptr);
    // k4a.Depth_With_Mask(*depth_k4a_ptr, *objs_ptr);
    // cv::imshow("Color Seg", *(color_k4a_ptr));
    // cv::imshow("Depth Seg", *(depth_k4a_ptr));
    // cv::waitKey(1);

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*cloud_seg_ptr, *cloud_seg_ptr, valid_indices);
    std::cout << "Global PointCloud:" << cloud_seg_ptr->size() << std::endl;
    pcl::toROSMsg(*cloud_seg_ptr, cloud_msg);
    cloud_msg.header.frame_id = "odom";
    std::cout << "cloud_msg(size)" << cloud_msg.data.size() << std::endl;
    pub_cloud.publish(cloud_msg);
}

void RosBasket::clb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    cloud_in = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    std::cout << "ok!" << std::endl;
    pcl::fromROSMsg(*msg, *cloud_in);
    pclprocess.Vg_Filter(0.03, cloud_in);
    // pclprocess.Sor_Filter(50, 1, cloud_in);
    std::cout << cloud_in->size() << std::endl;
    // pclprocess.Ror_Filter(35, 0.15, cloud_in);

    pcl::toROSMsg(*cloud_in, basket_msg);
    basket_msg.header.frame_id = "odom";
    pub_basket.publish(basket_msg);
    pcl::compute3DCentroid(*cloud_in, *centroid);
    std::cout << "x:" << centroid->x()
              << ",y:" << centroid->y()
              << ",z:" << centroid->z() << std::endl;
}
