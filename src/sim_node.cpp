#include"Control_quadcopter.h"

int main(int argc, char **argv)
{
    // Initiate ROS
    ros::init(argc, argv, "sim_node");

    // Create an object of class Control_quadcopter that will take care of everything
    Control_quadcopter control_quadcopter_object;

    ros::Rate r(control_quadcopter_object.FRAMES_PER_SECOND_); 
    // ros::Rate r(20); 
    while (ros::ok())
    {   
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}