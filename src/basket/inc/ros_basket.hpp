#pragma once
#include <memory>
#include <functional>
#include <chrono>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include "camera.hpp"
#include "myinfer.hpp"
#include "pclprocess.hpp"

class RosBasket
{
private:
    K4a k4a;
    Yolo yolo;
    PclProcess pclprocess;

    ros::Publisher pub_cloud;
    ros::Subscriber sub_cloud;
    ros::Publisher pub_basket;
    sensor_msgs::PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2 basket_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
    std::shared_ptr<std::string> engine_v8_ptr = std::make_shared<std::string>("/home/dzx/Documents/yolo/RIGHT-Infer-main/workspace/best.transd.engine");
    std::shared_ptr<std::string> engine_v8_seg_ptr = std::make_shared<std::string>("/home/dzx/Documents/yolo/RIGHT-Infer-main/workspace/best_seg.transd.engine");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::shared_ptr<Eigen::Vector4f> centroid = std::make_shared<Eigen::Vector4f>();
    std::shared_ptr<cv::Mat> color_k4a_ptr = std::make_shared<cv::Mat>();
    std::shared_ptr<cv::Mat> depth_k4a_ptr = std::make_shared<cv::Mat>();
    std::shared_ptr<cv::Mat> color_rs_ptr = std::make_shared<cv::Mat>();
    std::shared_ptr<cv::Mat> depth_rs_ptr = std::make_shared<cv::Mat>();
    std::shared_ptr<yolo::BoxArray> objs_ptr = std::make_shared<yolo::BoxArray>();

public:
    RosBasket(ros::NodeHandle &nh);

    ~RosBasket();

    void K4a_Basket_Get();

    void clb(const sensor_msgs::PointCloud2::ConstPtr &msg);
};