#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //IMU
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

class Control_quadcopter
{
	public:
	Control_quadcopter()
	{
		pose_publisher_ = n_.advertise<geometry_msgs::PoseStamped>("/command/pose", 1);
		cam_image_subscriber_ = n_.subscribe("/downward_cam/camera/image", 1, &Control_quadcopter::image_callback, this);
		quadcopter_pose_subscriber_ = n_.subscribe("/raw_imu", 1, &Control_quadcopter::quadcopter_state_callback, this);
	}

	void image_callback(const sensor_msgs::ImageConstPtr& left_img);
	void quadcopter_state_callback(const sensor_msgs::Imu& imu_data);

	private:
		ros::NodeHandle n_; 
		ros::Publisher pose_publisher_;
		ros::Subscriber cam_image_subscriber_;
		ros::Subscriber quadcopter_pose_subscriber_;
};
