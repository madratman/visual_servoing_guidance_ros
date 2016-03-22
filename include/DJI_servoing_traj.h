#include "ros/ros.h"
#include <cstdlib>
#include <algorithm>

// trajectory following and dji stuff  
#include <ca_common/Trajectory.h>
#include <ca_common/math.h>
#include <spektrum/MikrokopterStatus.h>
#include <ca_common/MissionWaypoint.h>
#include <ca_common/Mission.h>
#include <ca_common/MissionCommand.h>
#include <dji_sdk/RCChannels.h>

// line detection and Guidance stuff
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include "Line_detector.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


class DJI_servoing_traj
{	
	public:
		// using namespace CA; // TODO gotta eliminate this
		DJI_servoing_traj() : it_(node_handle_)
		{
			path_pub_    = node_handle_.advertise<ca_common::Trajectory>("/trajectory_gen/path", 100);
			subPose_     = node_handle_.subscribe("odom", 10, &DJI_servoing_traj::getPose_callback, this);
			subStart_    = node_handle_.subscribe("/spektrum/status", 1, &DJI_servoing_traj::getStart_callback, this);
			subStartDji_ = node_handle_.subscribe("/dji_sdk/rc_channels", 1, &DJI_servoing_traj::getStartDji_callback, this);
			it_sub_      = it_.subscribeCamera("/guidance/right/image_raw", 10, &DJI_servoing_traj::image_callback, this);

			node_handle_.param("/trajectory_gen/loopRate", loopRate_, 4.0);
			node_handle_.param("/trajectory_gen/nodeCount", nodeCount_, 20);
			node_handle_.param("/trajectory_gen/speed", speed_, 0.1);
			node_handle_.param("/trajectory_gen/amplitude", amplitude_, 4.0);
		}

		void getPose_callback(const nav_msgs::OdometryConstPtr &msg);
		void getStart_callback(const spektrum::MikrokopterStatus::ConstPtr &msg);
		void getStartDji_callback(const dji_sdk::RCChannels::ConstPtr &msg);
		void image_callback(const sensor_msgs::ImageConstPtr& image_message, const sensor_msgs::CameraInfoConstPtr& cam_info);

		double loopRate_;

	private:
		ros::NodeHandle node_handle_; 
		ros::Publisher path_pub_;     
		ros::Subscriber subPose_;     
		ros::Subscriber subStart_;    
		ros::Subscriber subStartDji_; 

		image_transport::ImageTransport it_;
  		image_transport::CameraSubscriber it_sub_; // special subscriber that can subscribe to cam image and correspondin cam_info topic

		int nodeCount_;
		double speed_;
		double amplitude_;

		CA::Vector3D currentXYZ_;
		CA::Vector3D hoverAtXYZ_;
		CA::Vector3D lastGoodXYZ_;
		
		CA::Trajectory path_;
		double last_heading_,hoverHeading_;
		nav_msgs::Odometry startOdom_;
		bool firstTime_ = false;
		bool started_ = false;
		bool gotPose_ = false;
		bool alreadyAuto_ = false;

		// some constant params 
		const double MINIMUM_LINE_LENGTH_ = 100;
		const double HEIGHT_OF_WIRE_ = 50;
		const int FRAMES_PER_SECOND_ = 30;

		Line_detector* line_detector_ptr_; // a new Line_detector object is created at each callback. 
		Line last_detected_line_; // The last detected line which is of a minimum good length, so that if we lose the line, we can fly back towards it. 
        bool flag_last_detected_line_ = 0; // check flag for the first time last_detected_line_ is assigned, or the quadcopter will go beserk in the initial part, where there is no line present anyway!

		image_geometry::PinholeCameraModel camera_model_;
};


