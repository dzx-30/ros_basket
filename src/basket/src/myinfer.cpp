#include "../inc/myinfer.hpp"

yolo::Image Yolo::cvimg(const cv::Mat &image)
{
  return yolo::Image(image.data, image.cols, image.rows);
}

void Yolo::Yolov8_Enable(std::string &engine_)
{
  type = yolo::Type::V8;
  engine = engine_;
}

void Yolo::Yolov8_Seg_Enable(std::string &engine_seg)
{
  type = yolo::Type::V8Seg;
  engine = engine_seg;
}

void Yolo::Single_Inference(cv::Mat &image, yolo::BoxArray &objs_out)
{
  auto Start = std::chrono::system_clock::now();

  auto yolo = yolo::load(engine, type);
  if (yolo == nullptr)
    return;

  auto objs = yolo->forward(cvimg(image));

  auto End = std::chrono::system_clock::now();
  auto Duration = std::chrono::duration_cast<std::chrono::microseconds>(End - Start);
  std::cout << "Infer Duration: " << double(Duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << "s" << std::endl;

  objs_out = objs;
}

Yolo::Yolo()
{
}

Yolo::~Yolo()
{
}
