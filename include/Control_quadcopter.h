#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //IMU
#include <geometry_msgs/PointStamped.h> //IMU
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "Line_detector.h"
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

class Control_quadcopter
{
	public:
	Control_quadcopter() : it_(node_handle_)
	{
		pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/command/pose", 10);
		// cam_image_subscriber_ = node_handle_.subscribe("/downward_cam/camera/image", 10, &Control_quadcopter::image_callback, this);
		quadcopter_pose_subscriber_ = node_handle_.subscribe("/ground_truth_to_tf/pose", 10, &Control_quadcopter::quadcopter_state_callback, this);
		it_sub_ = it_.subscribeCamera("/downward_cam/camera/image", 10, &Control_quadcopter::image_callback, this);
		// quadcopter_pose_subscriber_ = node_handle_.subscribe("/gazebo/model_states", 10, &Control_quadcopter::quadcopter_state_callback);
	}

	void image_callback(const sensor_msgs::ImageConstPtr& image_message, const sensor_msgs::CameraInfoConstPtr& cam_info);
	void quadcopter_state_callback(const geometry_msgs::PoseStamped pose_msg);
	// void quadcopter_state_callback(const gazebo::ModelState::ConstPtr& msg);
	
	const int FRAMES_PER_SECOND_ = 30;

	private:
		// ROS publishers and subscribers
		ros::NodeHandle node_handle_; 
		ros::Publisher pose_publisher_;
		ros::Subscriber cam_image_subscriber_;
		ros::Subscriber quadcopter_pose_subscriber_;

		image_transport::ImageTransport it_;
  		image_transport::CameraSubscriber it_sub_; // special subscriber that can subscribe to cam image and correspondin cam_info topic

		// Variables shared b/w callbacks, required to publish the next desired pose 
		double current_roll_, current_pitch_, current_yaw_, current_pos_x_, current_pos_y_, current_pos_z_; 
		tf::Quaternion current_quat_;

		Line_detector* line_detector_ptr_; // a new Line_detector object is created at each callback. 

		// If we lose the line, we need to save the last good detected line, and the pose of the quadcopter when it was detected
		Line last_detected_line_; // The last detected line which is of a minimum good length, so that if we lose the line, we can fly back towards it. 
        bool flag_last_detected_line_ = 0; // check flag for the first time last_detected_line_ is assigned, or the quadcopter will go beserk in the initial part, where there is no line present anyway!
    	double roll_last_detected_line_, pitch_last_detected_line_, yaw_last_detected_line_, pos_x_last_detected_line_, pos_y_last_detected_line_, pos_z_last_detected_line_; 
		tf::Quaternion quat_last_detected_line_;

		// some constant params 
		const double MINIMUM_LINE_LENGTH_ = 100;
		const double HEIGHT_OF_WIRE_ = 50;

		image_geometry::PinholeCameraModel camera_model_;
        tf::Point target_point_in_world_frame_;
        tf::TransformListener tf_listener_;
};	
