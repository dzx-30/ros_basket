#include "../inc/camera.hpp"
#include "../inc/myinfer.hpp"

using namespace std;

void K4a::Open()
{
    device = k4a::device::open(K4A_DEVICE_DEFAULT);
    if (!device)
    {
        COUT_RED_START
        cerr << "Open K4a Device Error!" << endl;
        COUT_COLOR_END
    }
    else
    {
        COUT_GREEN_START
        cout << "Open K4a Device Success!" << endl;
        COUT_COLOR_END
    }
}

void K4a::Installed_Count()
{
    device_count = k4a::device::get_installed_count();
    if (device_count == 0)
    {
        COUT_RED_START
        cout << "No K4a Device Found!" << endl;
        COUT_COLOR_END
    }
    else
    {
        COUT_BLUE_START
        cout << "Find " << device_count << " Device(s)" << endl;
        COUT_COLOR_END
    }
}

void K4a::Configuration()
{
    config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;

    device.start_cameras(&config);

    COUT_GREEN_START
    cout << "Start Device Success!" << endl;
    COUT_COLOR_END

    k4aCalibration = device.get_calibration(config.depth_mode, config.color_resolution);
    k4aTransformation = k4a::transformation(k4aCalibration);
    color_intrinsics = k4aCalibration.color_camera_calibration;
}

void K4a::Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth)
{
    if (device.get_capture(&capture, chrono::milliseconds(500)))
    {
        image_k4a_color = capture.get_color_image();
        image_k4a_depth = capture.get_depth_image();
        image_k4a_depth_to_color = k4aTransformation.depth_image_to_color_camera(image_k4a_depth);
        image_k4a_depth_to_pcl = k4aTransformation.depth_image_to_point_cloud(image_k4a_depth_to_color, K4A_CALIBRATION_TYPE_COLOR);

        image_cv_xyz = cv::Mat(image_k4a_depth_to_pcl.get_height_pixels(), image_k4a_depth_to_pcl.get_width_pixels(), CV_16SC3,
                               (void *)image_k4a_depth_to_pcl.get_buffer(), static_cast<size_t>(image_k4a_depth_to_pcl.get_stride_bytes()));

        image_cv_color = cv::Mat(image_k4a_color.get_height_pixels(), image_k4a_color.get_width_pixels(), CV_8UC4, image_k4a_color.get_buffer());
        cv::cvtColor(image_cv_color, image_cv_color, cv::COLOR_BGRA2BGR);

        cvtColor(image_cv_color, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 43, 46), cv::Scalar(34, 255, 255), imgThresholded);

        image_cv_depth = cv::Mat(image_k4a_depth_to_color.get_height_pixels(), image_k4a_depth_to_color.get_width_pixels(), CV_16U, image_k4a_depth_to_color.get_buffer());
        image_cv_depth.convertTo(image_cv_depth, CV_8U);
        cv::resize(image_cv_xyz, image_cv_xyz, image_cv_depth.size(), 0, 0, cv::INTER_LINEAR);
        // cv::imshow("xyz", image_cv_xyz);
    }
}

void K4a::Color_With_Mask(cv::Mat &image_cv_color, yolo::BoxArray &objs)
{
    if (objs.empty())
        return;

    yolo::Box boxBest = objs[0];
    for (auto &box : objs)
    {
        if (box.confidence > boxBest.confidence)
            boxBest = box;
    }

    if (boxBest.left >= 0 && boxBest.right < image_cv_color.cols && boxBest.top >= 0 && boxBest.bottom <= image_cv_color.rows)
    {
        uint8_t b, g, r;
        std::tie(b, g, r) = yolo::random_color(boxBest.class_label);
        cv::rectangle(image_cv_color, cv::Point(boxBest.left, boxBest.top), cv::Point(boxBest.right, boxBest.bottom),
                      cv::Scalar(b, g, r), 5);
        auto name = labels[boxBest.class_label];
        auto caption = cv::format("%s %.2f", name, boxBest.confidence);
        int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
        cv::rectangle(image_cv_color, cv::Point(boxBest.left - 3, boxBest.top - 33),
                      cv::Point(boxBest.left + width, boxBest.top), cv::Scalar(b, g, r), -1);
        cv::putText(image_cv_color, caption, cv::Point(boxBest.left, boxBest.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);
        if (boxBest.seg)
        {
            cv::Mat mask = cv::Mat(boxBest.seg->height, boxBest.seg->width, CV_8U, boxBest.seg->data);
            mask.convertTo(mask, CV_8UC1);
            cv::resize(mask, mask, cv::Size(boxBest.right - boxBest.left, boxBest.bottom - boxBest.top), 0, 0, cv::INTER_LINEAR);
            cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
            cv::addWeighted(image_cv_color(cv::Rect(boxBest.left, boxBest.top, boxBest.right - boxBest.left, boxBest.bottom - boxBest.top)), 1.0, mask, 0.8, 0.0, mask);
            mask.copyTo(image_cv_color(cv::Rect(boxBest.left, boxBest.top, boxBest.right - boxBest.left, boxBest.bottom - boxBest.top)));
        }
    }
}

void K4a::Depth_With_Mask(cv::Mat &image_cv_depth, yolo::BoxArray &objs)
{
    if (objs.empty())
        return;

    yolo::Box boxBest = objs[0];
    for (auto &box : objs)
    {
        if (box.confidence > boxBest.confidence)
            boxBest = box;
    }

    if (boxBest.left >= 0 && boxBest.right < image_cv_depth.cols && boxBest.top >= 0 && boxBest.bottom <= image_cv_depth.rows)
    {
        uint8_t b, g, r;
        std::tie(b, g, r) = yolo::random_color(boxBest.class_label);
        cv::rectangle(image_cv_depth, cv::Point(boxBest.left, boxBest.top), cv::Point(boxBest.right, boxBest.bottom),
                      cv::Scalar(b, g, r), 5);
        auto name = labels[boxBest.class_label];
        auto caption = cv::format("%s %.2f", name, boxBest.confidence);
        int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
        cv::rectangle(image_cv_depth, cv::Point(boxBest.left - 3, boxBest.top - 33),
                      cv::Point(boxBest.left + width, boxBest.top), cv::Scalar(b, g, r), -1);
        cv::putText(image_cv_depth, caption, cv::Point(boxBest.left, boxBest.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);
        if (boxBest.seg)
        {
            cv::Mat mask = cv::Mat(boxBest.seg->height, boxBest.seg->width, CV_8U, boxBest.seg->data);
            mask.convertTo(mask, CV_8UC1);
            cv::resize(mask, mask, cv::Size(boxBest.right - boxBest.left, boxBest.bottom - boxBest.top), 0, 0, cv::INTER_LINEAR);
            cv::addWeighted(image_cv_depth(cv::Rect(boxBest.left, boxBest.top, boxBest.right - boxBest.left, boxBest.bottom - boxBest.top)), 1.0, mask, 1.0, 0.0, mask);
            mask.copyTo(image_cv_depth(cv::Rect(boxBest.left, boxBest.top, boxBest.right - boxBest.left, boxBest.bottom - boxBest.top)));
        }
    }
}

// TODO:delete
void K4a::Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud, cv::Mat &image_cv_depth, yolo::BoxArray &objs)
{
    cloud.clear();

    if (objs.empty())
        return;
    yolo::Box boxBest = objs[0];
    for (auto &box : objs)
    {
        if (box.confidence > boxBest.confidence)
            boxBest = box;
    }

    k4a::image xyzImage{nullptr};
    int width = k4aCalibration.color_camera_calibration.resolution_width;
    int height = k4aCalibration.color_camera_calibration.resolution_height;

    xyzImage = image_k4a_depth_to_pcl;
    auto *xyzImageData = (int16_t *)(void *)xyzImage.get_buffer();
    for (int u = boxBest.left; u < boxBest.right; u++)
    {
        for (int v = boxBest.top; v < boxBest.bottom; v++)
        {
            int i = v * width + u;
            if (boxBest.seg->data[u, v] == 0)
            {
                pcl::PointXYZ point;
                point.x = xyzImageData[3 * i + 0];
                point.y = xyzImageData[3 * i + 1];
                point.z = xyzImageData[3 * i + 2];
                // std::cout << point.x << "," << point.y << "," << point.z << std::endl;
                cloud.push_back(point);
            }
        }
    }
    std::cout << "size:" << cloud.size() << std::endl;

    xyzImage.reset();
}

void K4a::Value_Mask_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud, yolo::BoxArray &objs)
{
    cloud.clear();
    uint16_t *depth_data = (uint16_t *)image_k4a_depth_to_color.get_buffer();

    if (objs.empty())
        return;
    yolo::Box boxBest = objs[0];
    for (auto &box : objs)
    {
        if (box.confidence > boxBest.confidence)
            boxBest = box;
    }

    for (int v = boxBest.top; v < boxBest.bottom; v++)
    {
        for (int u = boxBest.left; u < boxBest.right; u++)
        {
            // if (boxBest.seg->data[u, v] == 0)
            {
                if (imgThresholded.at<uchar>(u, v) == 0)
                {
                    float depth_value = static_cast<float>(depth_data[v * image_k4a_depth_to_color.get_width_pixels() + u] / 1000.0);
                    if (depth_value != 0)
                    {
                        float x = (u - color_intrinsics.intrinsics.parameters.param.cx) * depth_value / color_intrinsics.intrinsics.parameters.param.fx;
                        float y = (v - color_intrinsics.intrinsics.parameters.param.cy) * depth_value / color_intrinsics.intrinsics.parameters.param.fy;
                        float z = depth_value;
                        // std::cout << "x=" << x << ",y=" << y << ",z=" << z << std::endl;
                        cloud.push_back(pcl::PointXYZ(x, y, z));
                    }
                }
            }
        }
    }
}

K4a::K4a()
{
    Installed_Count();
    Open();
    Configuration();
}

K4a::~K4a()
{
    image_k4a_depth.reset();
    image_k4a_color.reset();
    capture.reset();
    device.close();
}
