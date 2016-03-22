#include "DJI_servoing_vel.h" // indoor
// #include "DJI_servoing_traj.h" // outdoor

int main(int argc, char **argv)
{
    // Initiate ROS
    ros::init(argc, argv, "real_life_node");

    // Create an object of class Control_quadcopter that will take care of everything
    DJI_servoing_vel DJI_servoing_vel_object;

    // ros::Rate r(DJI_servoing_vel_object.FRAMES_PER_SECOND_); 
    // ros::Rate r(DJI_servoing_vel_object.loopRate_); 
     ros::Rate r(50); 
    while (ros::ok())
    {   
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
