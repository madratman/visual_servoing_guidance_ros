#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //IMU
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <tf/transform_datatypes.h>

class Control_quadcopter
{
	public:
	Control_quadcopter()
	{
		// pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/command/pose", 10);
		cam_image_subscriber_ = node_handle_.subscribe("/downward_cam/camera/image", 10, &Control_quadcopter::image_callback, this);
		quadcopter_pose_subscriber_ = node_handle_.subscribe("/ground_truth_to_tf/pose", 10, &Control_quadcopter::quadcopter_state_callback, this);
		// quadcopter_pose_subscriber_ = node_handle_.subscribe("/gazebo/model_states", 10, &Control_quadcopter::quadcopter_state_callback);
	}

	void image_callback(const sensor_msgs::ImageConstPtr& left_img);
	void quadcopter_state_callback(const geometry_msgs::PoseStamped pose_msg);
	// void quadcopter_state_callback(const gazebo::ModelState::ConstPtr& msg);

	private:
		// ROS publishers and subscribers
		ros::NodeHandle node_handle_; 
		ros::Publisher pose_publisher_;
		ros::Subscriber cam_image_subscriber_;
		ros::Subscriber quadcopter_pose_subscriber_;

		// Variables shared b/w callbacks, required to publish the next desired pose 
		double current_roll_, current_pitch_, current_yaw_, current_pos_x_, current_pos_y_, current_pos_z_; 
		tf::Quaternion current_quat_;
};	
