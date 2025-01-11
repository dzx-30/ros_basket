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

    while (ros::ok())
    {
        RosBasket.K4a_Basket_Get();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
