#ifndef PCLPROCESS_HPP
#define PCLPROCESS_HPP

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>

#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>

#define TIMESTART auto Start = std::chrono::system_clock::now();
#define TIMEEND auto End = std::chrono::system_clock::now();
#define DURATION std::cout << "Duration: " << double(std::chrono::duration_cast<std::chrono::microseconds>(End - Start).count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << "s" << std::endl;

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

class PclProcess
{
private:
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;

public:
    Circle3D circle_center;

    void Input_PointCloud(std::string &pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

    void Input_PointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in);

    void Vg_Filter(float leafsize, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

    void Sor_Filter(int amount, float std, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

    void Ror_Filter(int amount, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

    void Circle_Extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, Eigen::VectorXf &coeff);

    Circle3D fitCircleLM(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, double radius, Eigen::VectorXf &coeff);

    void Draw_Circle(Eigen::VectorXf &coeff);

    PclProcess();

    ~PclProcess();
};

#endif