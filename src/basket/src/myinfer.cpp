#include "../inc/myinfer.hpp"

yolo::Image Yolo::cvimg(const cv::Mat &image)
{
  return yolo::Image(image.data, image.cols, image.rows);
}

void Yolo::Single_Inference(cv::Mat &image, yolo::BoxArray &objs_out, std::shared_ptr<yolo::Infer> yolo)
{
  auto Start = std::chrono::system_clock::now();

  auto objs = yolo->forward(cvimg(image));

  auto End = std::chrono::system_clock::now();
  auto Duration = std::chrono::duration_cast<std::chrono::microseconds>(End - Start);
  // std::cout << "Infer Duration: " << double(Duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << "s" << std::endl;

  objs_out = objs;
}

Yolo::Yolo()
{
}

Yolo::~Yolo()
{
}
