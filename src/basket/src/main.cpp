#include "../inc/camera.hpp"
#include "../inc/myinfer.hpp"
#include "../inc/pclprocess.hpp"
#include "../inc/ros_basket.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "point");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    RosBasket RosBasket(nh);
    RosBasket.yol = yolo::load("/home/dzx/best.engine", yolo::Type::V8Seg);

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
