cmake_minimum_required(VERSION 3.12)

project(cuda)


# set(TENSORRT_DIR "/home/dzx/install/TensorRT-8.6.1.6")
 
file(GLOB_RECURSE YOLO_SOURCES "*.cu")
message(STATUS "YOLO_SOURCES: ${YOLO_SOURCES}")

find_package(CUDA REQUIRED)

cuda_add_library(yolo STATIC ${YOLO_SOURCES})


include_directories(
    /usr/include/aarch64-linux-gnu/**
    /usr/local/cuda-11.4/targets/aarch64-linux/include/**
)

target_link_libraries(yolo 
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
)
