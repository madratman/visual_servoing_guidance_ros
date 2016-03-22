#include "DJI_servoing_vel.h" 
using namespace CA;
using namespace cv;
using namespace std;
#define WIDTH 320
#define HEIGHT 240

// TODO omit trackbars in the odroid. But anyway, you don't imshow so meh. 
/* Trackbars and their lower and upper limits */
int lower_hough_prob_min_no_of_intersections_trackbar = 1;     // lower limit
int upper_hough_prob_min_no_of_intersections_trackbar = 1000;  // upper limit 
int hough_prob_min_no_of_intersections_trackbar = 100;         // default

int lower_hough_prob_min_no_of_points_trackbar = 1;            // lower limit
int upper_hough_prob_min_no_of_points_trackbar = 1000;         // upper limit    
int hough_prob_min_no_of_points_trackbar = 100;                // default

int lower_hough_prob_max_gap_bw_points_trackbar = 1;           // lower limit
int upper_hough_prob_max_gap_bw_points_trackbar = 1000;        // upper limit 
int hough_prob_max_gap_bw_points_trackbar = upper_hough_prob_max_gap_bw_points_trackbar; // default

/* Window names */
const char* hough_prob_window = "Probabilistic Hough";
const char* threshold_window = "thresholded image";
const char* blur_window = "blurred image";

void DJI_servoing_vel::get_quaternion_callback(const dji_sdk::AttitudeQuaternion &msg)
{
    QuatD* orientation_ptr = new QuatD(msg.q0, msg.q1, msg.q2, msg.q3); // check inside airlab_dji_sdk::msg for definition
    Vector3D attitude;
    attitude = CA::quatToEuler(*orientation_ptr);
    last_heading_ = attitude[2]*180/M_PI;
  //  cout << "attitude [0],[1],[2] :::   " << attitude[0]*180/M_PI << ", " << attitude[1]*180/M_PI << ", " << attitude[2]*180/M_PI << endl;
	
    GuidanceRos::rpy rpy_msg;
    rpy_msg.roll = attitude[0]*180/M_PI;
    rpy_msg.pitch = attitude[1]*180/M_PI;
    rpy_msg.yaw = attitude[2]*180/M_PI;
    rpy_publisher_.publish(rpy_msg);
}

void DJI_servoing_vel::getStartDji_callback(const dji_sdk::RCChannels::ConstPtr &msg)
{
    if(msg->mode > 4000.0 && started_ == false && !alreadyAuto_) 
    {
    //     if(!gotPose_) 
    //     {
    //         ROS_ERROR_STREAM("never got pose!");
    //         return;
    //     }

//        if ((msg->header.stamp - startOdom_.header.stamp).toSec() > 0.5) {
       // if ((ros::Time::now() - startOdom_.header.stamp).toSec() > 0.5) 
       // {
         //   ROS_ERROR_STREAM("odometry msg too old!"<<(msg->header.stamp - startOdom_.header.stamp).toSec());
           // return;
        //}
        
        hoverHeading_ = last_heading_;

        firstTime_ = true;
        started_ = true;
        alreadyAuto_ = true;
    } 
    else if(!(msg->mode > 4000.0)) 
    {
        cout << "DJI_servoing_vel::getStartDji_callback, started_ = false" << endl;
        started_ = false;
        alreadyAuto_ = false;
    }
}

void DJI_servoing_vel::image_callback(const sensor_msgs::ImageConstPtr& image_message, const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    // Update the camera model 
    camera_model_.fromCameraInfo(cam_info_msg);

    // Convert the image_message to something that openCV understands
    cv_bridge::CvImagePtr cv_ptr;
    try 
    {
        cv_ptr = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge exception");
        return;
    }

    // Visualize the image
    // imshow("image_upper_cam", cv_ptr->image);

    // openCV variables
    Mat image_original = cv_ptr->image; 
    Mat image_thresh;
    Mat image_blur;
    Mat image_after_canny; 
    Mat hough_standard_result; 
    Mat hough_prob_result; 
    int image_original_width = image_original.cols;
    int image_original_height = image_original.rows;

    vector<Vec4i> opencv_lines; // lines detected by hough transform
    vector<Vec4i> unique_lines; // unique lines returned by Line_detector::remove_duplicates()
    Vec4i best_line_opencv;

    char thresh_label_hough_prob_1[50];
    char thresh_label_hough_prob_2[50];
    char thresh_label_hough_prob_3[50];
    sprintf(thresh_label_hough_prob_1, "min_no_of_intersections");  
    sprintf(thresh_label_hough_prob_2, "min_no_of_points_in_line");  
    sprintf(thresh_label_hough_prob_3, "max_gap_between_points_in_line");  
    
    // Make trackbars to easily adjust hough params on the fly. 
    // TODO low priority. add bars for 3rd and 4th param of HoughLinesP() - the rho, theta resolutions 
    // namedWindow(hough_prob_window, 1);
    // createTrackbar(thresh_label_hough_prob_1, hough_prob_window, &hough_prob_min_no_of_intersections_trackbar, upper_hough_prob_min_no_of_intersections_trackbar);
    // createTrackbar(thresh_label_hough_prob_2, hough_prob_window, &hough_prob_min_no_of_points_trackbar, upper_hough_prob_min_no_of_points_trackbar);
    // createTrackbar(thresh_label_hough_prob_3, hough_prob_window, &hough_prob_max_gap_bw_points_trackbar, upper_hough_prob_max_gap_bw_points_trackbar);

    // (do a custom hack acc to scene to filter out crap/clutter in the background) -> threshold the image 
    int threshold_value = 200;
    int threshold_type = 4;
    int const max_value = 255;
    int const max_type = 4;
    int const max_BINARY_value = 255;
    char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
    char* trackbar_value = "Value";

    // createTrackbar(trackbar_type, threshold_window, &threshold_type, max_type);
    // createTrackbar(trackbar_value, threshold_window, &threshold_value, max_value);
    // threshold(image_original, image_thresh, threshold_value, max_BINARY_value,threshold_type);
    // namedWindow(threshold_window, 1);
    // imshow(threshold_window, image_thresh);

//    medianBlur(image_original, image_blur, 3);  // change blur kernel size. keep it small. k = 9, for ex => no edges in canny map :| 
    // imshow(blur_window, image_blur);
    
    // threshold(image_blur, image_thresh, 0, 160, 0);
    // imshow(threshold_window, image_thresh);
    Canny(image_original, image_after_canny, 50, 200, 3);

    // cv::Mat skel(image_thresh.size(), CV_8UC1, cv::Scalar(0));
    // cv::Mat temp(image_thresh.size(), CV_8UC1);
    // cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    // bool done;
    // do
    // {
    //   cv::morphologyEx(image_thresh, temp, cv::MORPH_OPEN, element);
    //   cv::bitwise_not(temp, temp);
    //   cv::bitwise_and(image_thresh, temp, temp);
    //   cv::bitwise_or(skel, temp, skel);
    //   cv::erode(image_thresh, image_thresh, element);
     
    //   double max;
    //   cv::minMaxLoc(image_thresh, 0, &max);
    //   done = (max == 0);
    // } while (!done);
    // cv::imshow("Skeleton", skel);

    // Detect the lines

    cvtColor(image_after_canny, hough_prob_result, COLOR_GRAY2BGR); // minor wtf 
    HoughLinesP(image_after_canny, opencv_lines, 2, 0.05*CV_PI/180, lower_hough_prob_min_no_of_intersections_trackbar + hough_prob_min_no_of_intersections_trackbar, lower_hough_prob_min_no_of_points_trackbar + hough_prob_min_no_of_points_trackbar, lower_hough_prob_max_gap_bw_points_trackbar + hough_prob_max_gap_bw_points_trackbar);
    // 3rd and 4th argument are important in HoughLinesP. It's the resolution of rho and theta. 

    // cout << opencv_lines.size()<<endl;

    /* View original lines */
     for(int i = 0; i<opencv_lines.size(); i++)
     {   
         Vec4i l_cur = opencv_lines[i];
        // cout << "l_cur[0],l_cur[1])" << l_cur[0] << ", " << l_cur[1] << endl; 
        // cout << "l_cur[2],l_cur[3])" << l_cur[2] << ", " << l_cur[3] << endl; 
        // line(hough_prob_result, Point(l_cur[0], l_cur[1]), Point(l_cur[2], l_cur[3]), Scalar(255,0,0), 1, CV_AA);
        // circle(hough_prob_result, Point(l_cur[0],l_cur[1]), 10, Scalar(0,0,255), 1, 8); // plots red circle at first end point
        // circle(hough_prob_result, Point(l_cur[2],l_cur[3]), 10, Scalar(0,255,0), 1, 8); // plots green circle at second end point
     }

    // Future/as needs be : (Modify the Line_detector class to) return only the unique lines Filter out duplicates : combine fragment lines + multiple parallel lines. 

    trajectory_control::Command vel_and_heading_msg;
    vel_and_heading_msg.heading  ;// empty
    vel_and_heading_msg.acceleration  ;// empty

    if(1)
    {
        // Now the main servoing block method starts
        if(opencv_lines.size() != 0) // Pay attention to the loop condition and the corresponding else block. 
        {  
            cout << "here";
            // Find the best line
            line_detector_ptr_ = new Line_detector(opencv_lines, image_original_width, image_original_height);
            best_line_opencv = line_detector_ptr_->remove_duplicates();
            Line best_line_struct = line_detector_ptr_->return_best_line();

            // Plot the line, endpoints, center of image
             line(hough_prob_result, Point(best_line_opencv[0], best_line_opencv[1]), Point(best_line_opencv[2], best_line_opencv[3]), Scalar(255,0,0), 2, CV_AA);
             circle(hough_prob_result, Point(best_line_opencv[0],best_line_opencv[1]), 10, Scalar(0,0,255), 1, 8); // plots red circle at first end point
             circle(hough_prob_result, Point(best_line_opencv[2],best_line_opencv[3]), 10, Scalar(0,255,0), 1, 8); // plots green circle at second end point
          // circle(hough_prob_result, Point(image_original_width/2, image_original_height/2), 10, Scalar(255,255,255), 1, 8); // plots white circle at center of image

            // Find the various line's parameters to calculate the next pose target
            double best_angle = best_line_struct.angle_; // Note range is [0, pi] due to symmetric nature of problem. Check Line_detector.h
            double best_dist_from_origin = best_line_struct.dist_from_origin_;
            double best_line_length = best_line_struct.length_;
            cout << "best_angle " << best_angle << endl;
            cout << "difference " << 90 - best_angle << endl;
   	        cout << "best_dist_from_origin " << best_dist_from_origin << endl;
            double difference = 90 - best_angle;
            cv::Point2d midpoint_last_detected_line;
            // If the line length is greater than MINIMUM_LINE_LENGTH_, we save it as the last decent detected line and also update the corresponding pose 
            
            // TODO this 
            if(best_line_struct.length_ > MINIMUM_LINE_LENGTH_)
            {
                last_detected_line_ = best_line_struct; // member of Control_quadcopter, and not line_detector class
                flag_last_detected_line_ = 1; // This flag is useful for the initial flying part, when we don't have any lines detected previously.
            }

            // add condition on line length 

            // Check if the wire is (almost) vertical in the image. Else make it vertical
            if(abs(90 - best_angle) < 7.5) // TOCHECK. Angle ranges and sign bugs
            {   
                // Check if wire is in the center of the image. Else move left/right to bring it to center
                if(abs(best_dist_from_origin) < 10) // TOCHECK 
                {
                    cout << "vertical and center " << endl;

                    vel_and_heading_msg.velocity.x = 0;
                    vel_and_heading_msg.velocity.z = 0;

                    vel_and_heading_msg.heading = last_heading_;
                    vel_and_heading_msg.velocity.y = 0.275;
                }

                else // Move horizontally such that the wire is in the center of line
                {
                    	double working_variable;       
                        cout << "vertical but not in center " << endl;
	                    cout << "working_variable"<<(best_line_struct.start_point_.y_ + best_line_struct.end_point_.y_)/2  << endl;
                        working_variable  = (best_line_struct.start_point_.y_ + best_line_struct.end_point_.y_)/2; 
  
                    if (working_variable > 0 )
                    {
                        cout << "flying right " << endl;
			            vel_and_heading_msg.velocity.x = speed_ * 1.5 * ( working_variable/(WIDTH/2) ); // remember dist_from_orgin is positive, hence it needs a negative velocity input to fly left
		            }
 
                    else
                    {
                        cout << "flying left " << endl;
			            vel_and_heading_msg.velocity.x = speed_ * 1.5 * ( working_variable/(WIDTH/2) ); // remember dist_from_origin is negative, so it should get a +ve vel input to fly right
                    }
                    
                    vel_and_heading_msg.velocity.y = 0;
                    vel_and_heading_msg.velocity.z = 0;
                    vel_and_heading_msg.heading = last_heading_;
                }
            }

            else // if wire is not perpendicular, hold current position and yaw to align with wire 
            { 
                cout << " not vertical " << endl;

                //vel_and_heading_msg.heading = last_heading_ +(0.25* difference);// this is fucked up 
               
                vel_and_heading_msg.heading = last_heading_ -(0.3 * difference);// this is fucked up 

                vel_and_heading_msg.velocity.x = vel_and_heading_msg.velocity.y = vel_and_heading_msg.velocity.z = 0;
            }
        }

        else // No lines were found. Now, we fly towards the last decent detected line, or else we don't do anything () 
        {
            cout << "zero lines detected. " << endl;
            if(flag_last_detected_line_)
            {
                vel_and_heading_msg.velocity.x = vel_and_heading_msg.velocity.y = vel_and_heading_msg.velocity.z = 0;
                vel_and_heading_msg.heading = last_heading_;  

            }   
            else // the quad is flying around in the beginning and no target should be specified 
            {
                vel_and_heading_msg.velocity.x = vel_and_heading_msg.velocity.y = vel_and_heading_msg.velocity.z = 0;
            }
            // publish back the current pose. So as to make the quad stay at its place
        }
    }

	cout << endl; 	
//     imshow(hough_prob_window, hough_prob_result);
 //    cv::waitKey(1); 

//    cout << "DJI_servoing_vel.cpp :: vel_x, vel_y, vel_z, heading :: " << vel_and_heading_msg.velocity.x << ", " << vel_and_heading_msg.velocity.y << ", " << vel_and_heading_msg.velocity.z << ", " << vel_and_heading_msg.heading << endl; 
    vel_and_heading_pub_.publish(vel_and_heading_msg);
}
