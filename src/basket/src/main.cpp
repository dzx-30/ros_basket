#include "../inc/camera.hpp"
#include "../inc/myinfer.hpp"
#include "../inc/pclprocess.hpp"
#include "../inc/ros_basket.hpp"
float leafsize;
int amount;
float sordistance;
float rsdistance;
int rsmax;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "point");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    RosBasket RosBasket(nh);
    nh.param<float>("leafsize", leafsize, 0.05);
    nh.param<int>("amount", amount, 50);
    nh.param<float>("sordistance", sordistance, 0.1);
    nh.param<float>("rsdistance", rsdistance, 0.2);
    nh.param<int>("rsmax", rsmax, 10000);

    // RosBasket.yol = yolo::load("/home/lush/best.engine", yolo::Type::V8);
    RosBasket.yol = yolo::load("/home/lush/Documents/basketTestActuator/src/basket/engine/best.engine", yolo::Type::V8Seg);

    while (ros::ok())
    {
        // TIMESTART
        RosBasket.K4a_Basket_Get();
        ros::spinOnce();
        // TIMEEND
        // DURATION
    }

    return 0;
}
