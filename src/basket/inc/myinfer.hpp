#ifndef __MYINFER_HPP__
#define __MYINFER_HPP__

#include <opencv2/opencv.hpp>

#include "cpm.hpp"
#include "infer.hpp"
#include "yolo.hpp"

#include <chrono>

static const char *labels[] = {
    "Basket"};

class Yolo
{
private:
    yolo::Image cvimg(const cv::Mat &image);

public:
    void Single_Inference(cv::Mat &image, yolo::BoxArray &objs_out, std::shared_ptr<yolo::Infer> yolo);

    Yolo();

    ~Yolo();
};

#endif
