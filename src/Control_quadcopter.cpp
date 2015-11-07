#include <stdio.h>
#include <string.h>
#include <math.h>

#include "Control_quadcopter.h"

using namespace cv;
using namespace std;
#define WIDTH 320
#define HEIGHT 240

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

void Control_quadcopter::image_callback(const sensor_msgs::ImageConstPtr& image_message)
{
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
    imshow("image_upper_cam", cv_ptr->image);

    // openCV variables
    Mat image_original = cv_ptr->image; 
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
    namedWindow(hough_prob_window, 1);
    createTrackbar(thresh_label_hough_prob_1, hough_prob_window, &hough_prob_min_no_of_intersections_trackbar, upper_hough_prob_min_no_of_intersections_trackbar);
    createTrackbar(thresh_label_hough_prob_2, hough_prob_window, &hough_prob_min_no_of_points_trackbar, upper_hough_prob_min_no_of_points_trackbar);
    createTrackbar(thresh_label_hough_prob_3, hough_prob_window, &hough_prob_max_gap_bw_points_trackbar, upper_hough_prob_max_gap_bw_points_trackbar);

    // Detect the lines
    Canny(image_original, image_after_canny, 50, 200, 3);
    cvtColor(image_after_canny, hough_prob_result, COLOR_GRAY2BGR);
    HoughLinesP(image_after_canny, opencv_lines, 2, 0.05*CV_PI/180, lower_hough_prob_min_no_of_intersections_trackbar + hough_prob_min_no_of_intersections_trackbar, lower_hough_prob_min_no_of_points_trackbar + hough_prob_min_no_of_points_trackbar, lower_hough_prob_max_gap_bw_points_trackbar + hough_prob_max_gap_bw_points_trackbar);
    // 3rd and 4th argument are important in HoughLinesP. It's the resolution of rho and theta. 

    cout << opencv_lines.size()<<endl;

    /* View original lines */
    // for(int i = 0; i<opencv_lines.size(); i++)
    // {   
    //     Vec4i l_cur = opencv_lines[i];
    //     cout << "l_cur[0],l_cur[1])" << l_cur[0] << ", " << l_cur[1] << endl; 
    //     cout << "l_cur[2],l_cur[3])" << l_cur[2] << ", " << l_cur[3] << endl; 
    //     line(hough_prob_result, Point(l_cur[0], l_cur[1]), Point(l_cur[2], l_cur[3]), Scalar(255,0,0), 1, CV_AA);
    //     circle(hough_prob_result, Point(l_cur[0],l_cur[1]), 10, Scalar(0,0,255), 1, 8); // plots red circle at first end point
    //     circle(hough_prob_result, Point(l_cur[2],l_cur[3]), 10, Scalar(0,255,0), 1, 8); // plots green circle at second end point
    // }

    // Future/as needs be : (Modify the Line_detector class to) return only the unique lines Filter out duplicates : combine fragment lines + multiple parallel lines. 

    // target_pose message to be published. It will be filled depending on the line's orientation and distance from center of the image 
    geometry_msgs::PoseStamped target_pose;
    // in degrees. For humans
    double roll_target = 0.0;
    double pitch_target = 0.0;
    double yaw_target;
    // target quaternion for robots
    tf::Quaternion quat_target;

    // Now the main servoing block method starts
    if(opencv_lines.size() != 0) // Pay attention to the loop condition above and the corresponding else block. 
    {
        // Find the best line
        line_detector_ptr_ = new Line_detector(opencv_lines, image_original_width, image_original_height);
        best_line_opencv = line_detector_ptr_->remove_duplicates();
        Line best_line_struct = line_detector_ptr_->return_best_line();

        // Plot the line, endpoints, center of image
        line(hough_prob_result, Point(best_line_opencv[0], best_line_opencv[1]), Point(best_line_opencv[2], best_line_opencv[3]), Scalar(255,0,0), 1, CV_AA);
        circle(hough_prob_result, Point(best_line_opencv[0],best_line_opencv[1]), 10, Scalar(0,0,255), 1, 8); // plots red circle at first end point
        circle(hough_prob_result, Point(best_line_opencv[2],best_line_opencv[3]), 10, Scalar(0,255,0), 1, 8); // plots green circle at second end point
        circle(hough_prob_result, Point(image_original_width/2, image_original_height/2), 10, Scalar(255,255,255), 1, 8); // plots white circle at center of image

        // Find the various line's parameters to calculate the next pose target
        double best_angle = best_line_struct.angle_;
        double best_dist_from_origin = best_line_struct.dist_from_origin_;
        double best_line_length = best_line_struct.length_;
        // cout << "best_angle " << best_angle << endl;
        // cout << "best_dist_from_origin" << best_dist_from_origin << endl;
        // cout << "intercept_ " << best_line_struct.intercept_ << endl;
        // cout << "best_line_length " << best_line_length << endl;

        // If the line length is greater than MINIMUM_LINE_LENGTH_, we save it as the last decent detected line and also update the corresponding pose 
        if(best_line_struct.length_ > MINIMUM_LINE_LENGTH_)
        {
            last_detected_line_ = best_line_struct;
            flag_last_detected_line_ = 1; // This flag is useful for the initial flying part, when we don't have any lines detected previously.

            roll_last_detected_line_ = current_roll_;
            pitch_last_detected_line_ = current_pitch_;
            yaw_last_detected_line_ = current_yaw_;
            pos_x_last_detected_line_ = current_pos_x_;
            pos_y_last_detected_line_ = current_pos_y_;
            pos_z_last_detected_line_ = current_pos_z_;
            quat_last_detected_line_ = current_quat_;
        }

        // add condition on line length 

        // Check if the wire is (almost) vertical in the image. Else make it vertical
        if(abs(current_yaw_ - best_angle) < 5)
        {   
            // Check if wire is in the center of the image. Else move left/right to bring it to center
            if(abs(current_pos_x_ - best_dist_from_origin) < 10)
            {
                cout << "vertical and center " << endl;
                target_pose.pose.position.x = current_pos_x_;
                target_pose.pose.position.y = current_pos_y_;
            
                roll_target = 0.0;
                pitch_target = 0.0;
                if(current_yaw_ > 0)
                    yaw_target = 90.0;
                else
                    yaw_target = -90;

                quat_target.setRPY(roll_target, pitch_target, yaw_target);

                // convert to geometry_msgs/Pose format
                tf::quaternionTFToMsg(quat_target, target_pose.pose.orientation);
                
                // Check how far is the wire. If high enough, stop. Else move up by 10 cm. 
                if(HEIGHT_OF_WIRE_ - current_pos_z_ < 20)
                {
                    target_pose.pose.position.z = current_pos_z_; 
                } 

                else
                {
                    target_pose.pose.position.z = current_pos_z_ + 1; 
                }  
            }

            else // Move horizontally such that the wire is in the center of line
            {
                cout << "vertical but not in center " << endl;
                target_pose.pose.position.x = current_pos_x_ + best_dist_from_origin; // todo. remove absolute from dist_from_origin_
                target_pose.pose.position.y = current_pos_y_;
                target_pose.pose.position.z = current_pos_z_; 

                roll_target = 0.0;
                pitch_target = 0.0;

                // We don't care which way the quadcopter is facing, for now. 
                // Target yaw depends on the current alignment as it's already nearly perpendicular
                if(current_yaw_ > 0)
                    yaw_target = 90.0;
                else
                    yaw_target = -90;

                quat_target.setRPY(roll_target, pitch_target, yaw_target);

                // convert to geometry_msgs/Pose format
                tf::quaternionTFToMsg(quat_target, target_pose.pose.orientation);
            }
        }

        else // if wire is not perpendicular, hold current position and yaw to align with wire 
        { 
            cout << " not vertical " << endl;
            target_pose.pose.position.x = current_pos_x_; // todo. remove absolute from dist_from_origin_
            target_pose.pose.position.y = current_pos_y_;
            target_pose.pose.position.z = current_pos_z_; 

            roll_target = 0.0;
            pitch_target = 0.0;
            yaw_target;
            if(current_yaw_ > 0)
            {
                yaw_target = 90.0;
            }
            else
            {
                yaw_target = -90;
            }

            quat_target.setRPY(roll_target, pitch_target, yaw_target);

            // convert to geometry_msgs/Pose format
            tf::quaternionTFToMsg(quat_target, target_pose.pose.orientation);
        }
    }

    else // No lines were found. Now, we fly towards the last decent detected line, or else we don't do anything () 
    {
        cout << "zero lines detected. " << endl;
        if(flag_last_detected_line_)
        {
            target_pose.pose.position.x = current_pos_x_; // todo. remove absolute from dist_from_origin_
            target_pose.pose.position.y = current_pos_y_;
            target_pose.pose.position.z = current_pos_z_; 
            roll_target = 0.0;
            pitch_target = 0.0;
            yaw_target = current_yaw_;
            quat_target.setRPY(roll_target, pitch_target, yaw_target);
            tf::quaternionTFToMsg(quat_target, target_pose.pose.orientation);
        }
        // publish back the current pose. So as to make the quad stay at its place
    }
    
    cout << "target_pose.pose.position.x " << target_pose.pose.position.x << endl; 
    cout << "target_pose.pose.position.y " << target_pose.pose.position.y << endl; 
    cout << "target_pose.pose.position.z " << target_pose.pose.position.z << endl; 
    cout << "roll_target " << roll_target << endl;
    cout << "yaw_target " << yaw_target << endl;
    cout << "pitch_target " << pitch_target << endl;

    // cout << "current yaw " << current_yaw_ << endl << endl << endl;
    imshow(hough_prob_window, hough_prob_result);
    cv::waitKey(1); 
    // Publish target pose 
	pose_publisher_.publish(target_pose);	
}

void Control_quadcopter::quadcopter_state_callback(const geometry_msgs::PoseStamped pose_msg)
{
    current_pos_x_ = pose_msg.pose.position.x;
    current_pos_y_ = pose_msg.pose.position.y;
    current_pos_z_ = pose_msg.pose.position.z;

    // get RPY from the pose msg parameter
    tf::quaternionMsgToTF(pose_msg.pose.orientation, current_quat_);
    tf::Matrix3x3(current_quat_).getRPY(current_roll_, current_pitch_, current_yaw_);
    // yaw in [-pi, pi]

    // convert to degree
    current_roll_ = current_roll_ * 180/M_PI;
    current_pitch_ = current_pitch_ * 180/M_PI;
    current_yaw_ = current_yaw_ * 180/M_PI;

    if(current_yaw_ < 0)
    {
        current_yaw_ = current_yaw_ + 180;
    }   
}