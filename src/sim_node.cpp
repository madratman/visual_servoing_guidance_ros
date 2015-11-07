#include"Control_quadcopter.h"

int main(int argc, char **argv)
{
    // Initiate ROS
    ros::init(argc, argv, "sim_node");

    // Create an object of class Control_quadcopter that will take care of everything
    Control_quadcopter control_quadcopter_object;

    ros::spin();
    return 0;
}