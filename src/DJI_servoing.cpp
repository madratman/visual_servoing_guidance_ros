#include "DJI_servoing.h" 

using namespace CA;
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

void DJI_servoing::getPose_callback(const nav_msgs::OdometryConstPtr &msg)
{
    //ROS_INFO("pose received sq trajectory gen");
    startOdom_ = (*msg);
    currentXYZ_ = msgc((*msg).pose.pose.position);
    QuatD orientation = CA::msgc((*msg).pose.pose.orientation);
    Vector3D attitude;
    attitude = CA::quatToEuler(orientation);
    last_heading_ = attitude[2];
    gotPose_ = true;
}

void DJI_servoing::getStart_callback(const spektrum::MikrokopterStatus::ConstPtr &msg)
{
    if(msg->isAutonomous && started_ == false) {
        if(!gotPose_) {
            ROS_ERROR_STREAM("never got pose!");
            return;
        }

//        if ((msg->header.stamp - startOdom_.header.stamp).toSec() > 0.5) {
        if ((ros::Time::now() - startOdom_.header.stamp).toSec() > 0.5) {
            ROS_ERROR_STREAM("odometry msg too old!"<<(msg->header.stamp - startOdom_.header.stamp).toSec());
            return;
        }
        hoverAtXYZ_=currentXYZ_;
        hoverHeading_=last_heading_;

        firstTime_ = true;
        started_ = true;
        ROS_WARN_STREAM("Traj gen starts at  hold hover!" <<hoverAtXYZ_);

    } else if(!msg->isAutonomous) {
        started_ = false;
    }
}

void DJI_servoing::getStartDji_callback(const dji_sdk::RCChannels::ConstPtr &msg)
{
    if(msg->mode > 4000.0 && started_ == false && !alreadyAuto_) {
        if(!gotPose_) {
            ROS_ERROR_STREAM("never got pose!");
            return;
        }

//        if ((msg->header.stamp - startOdom_.header.stamp).toSec() > 0.5) {
        if ((ros::Time::now() - startOdom_.header.stamp).toSec() > 0.5) {
            ROS_ERROR_STREAM("odometry msg too old!"<<(msg->header.stamp - startOdom_.header.stamp).toSec());
            return;
        }
        hoverAtXYZ_=currentXYZ_;
        hoverHeading_=last_heading_;

        firstTime_ = true;
        started_ = true;
        alreadyAuto_ = true;
        ROS_WARN_STREAM("Traj gen starts at  hold hover!" <<hoverAtXYZ_);

    } else if(!(msg->mode > 4000.0)) {
        started_ = false;
        alreadyAuto_ = false;
    }
}

void DJI_servoing::image_callback(const sensor_msgs::ImageConstPtr& image_message, const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
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

    // cout << opencv_lines.size()<<endl;

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

    std::vector<State, Eigen::aligned_allocator<Vector3D> > sl;
    ca_common::Trajectory t;
    t.header.seq=1;
    CA::State c;

    if(firstTime_==1)
    {
        sl.clear();

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
        double best_angle = best_line_struct.angle_; // Note range is [0, pi] due to symmetric nature of problem. Check Line_detector.h
        double best_dist_from_origin = best_line_struct.dist_from_origin_;
        double best_line_length = best_line_struct.length_;
        cout << "best_angle " << best_angle << endl;
        cout << "difference " << abs(90 - best_angle) << endl;

        cv::Point2d midpoint_last_detected_line;
        // If the line length is greater than MINIMUM_LINE_LENGTH_, we save it as the last decent detected line and also update the corresponding pose 
        if(best_line_struct.length_ > MINIMUM_LINE_LENGTH_)
        {
            last_detected_line_ = best_line_struct; // member of Control_quadcopter, and not line_detector class
            flag_last_detected_line_ = 1; // This flag is useful for the initial flying part, when we don't have any lines detected previously.

            // for returning to that pose 
            lastGoodXYZ_ = currentXYZ_;            
        }

        // add condition on line length 

        // Check if the wire is (almost) vertical in the image. Else make it vertical
        if(abs(90 - best_angle) < 5) // TOCHECK. Angle ranges and sign bugs
        {   
            // Check if wire is in the center of the image. Else move left/right to bring it to center
            if(abs(best_dist_from_origin) < 10) // TOCHECK 
            {
                cout << "vertical and center " << endl;
                // c.pose.position_m[0] =  hoverAtXYZ[0];
                // c.pose.position_m[1] =  hoverAtXYZ[1];

                c.pose.orientation_rad[2] = last_heading_;

                c.rates.velocity_mps[0] = 0;
                c.rates.velocity_mps[1] = 0;

                // Check how far is the wire. If high enough, stop. Else move up by 10 cm. 
                if(HEIGHT_OF_WIRE_ - currentXYZ_[2] < 20)
                {
                    cout << "HEIGHT_OF_WIRE_ - currentXYZ_[2]= " << HEIGHT_OF_WIRE_ - currentXYZ_[2] << endl;
                    cout << "staying here " << endl;
                    c.rates.velocity_mps[2] = 0;
                    // c.pose.position_m[2] =  hoverAtXYZ[2];
                } 

                else
                {
                    cout << "HEIGHT_OF_WIRE_ - currentXYZ_[2]= " << HEIGHT_OF_WIRE_ - currentXYZ_[2] << endl;
                    cout << "flying up " << endl;
                    c.rates.velocity_mps[2] = speed_;
                    // c.pose.position_m[2] =  hoverAtXYZ[2] + 0.1;
                }  
            }

            else // Move horizontally such that the wire is in the center of line
            {
                cout << "vertical but not in center " << endl;
                // target_pose.pose.position.x = current_pos_x_ + best_dist_from_origin; // todo. remove absolute from dist_from_origin_
                if (midpoint_last_detected_line.y > 0 )
                {
                    cout << "flying left " << endl;
                    // c.pose.position_m[0] = hoverAtXYZ[0];
                    // c.pose.position_m[1] =  -0.1 + hoverAtXYZ[1];
                    
                    c.rates.velocity_mps[0] = speed_ * std::cos(last_heading_);
                    c.rates.velocity_mps[1] = -speed_ * std::cos(last_heading_);
                }

                else
                {
                    cout << "flying right " << endl;
                    // c.pose.position_m[1] =  0.1 + hoverAtXYZ[1];

                    c.rates.velocity_mps[0] = -speed_ * std::cos(last_heading_);
                    c.rates.velocity_mps[1] = speed_ * std::cos(last_heading_);
                }
                
                // c.pose.position_m[2] = hoverAtXYZ[2];

                c.pose.orientation_rad[2] = last_heading_;

                c.rates.velocity_mps[2] = 0;
            }
        }

        else // if wire is not perpendicular, hold current position and yaw to align with wire 
        { 
            cout << " not vertical " << endl;

/*            if (current_yaw_ < 90)
                yaw_target = M_PI/180*(current_yaw_ + (90 - best_angle)); // spin too much? 
            else
                yaw_target = M_PI/180*(current_yaw_ + (90 - best_angle) -180); // spin too much? 
*/            
            c.pose.orientation_rad[2] = last_heading_ - (best_angle*M_PI/180);
            c.rates.velocity_mps[0] = c.rates.velocity_mps[1] = c.rates.velocity_mps[2] = 0;

        }
    }

    else // No lines were found. Now, we fly towards the last decent detected line, or else we don't do anything () 
    {
        cout << "zero lines detected. " << endl;
        if(flag_last_detected_line_)
        {
            // If no line is found and flag_last_detected_line_ == 1, we need to fly back to the pose corresponding to the time when a line of decent length was detected last
            // Or else, we could just fly to the point corresponding to the midpoint of the last line detected - in world coordinates
            // The latter might be a bit jarring to the previous control strategy

            // Either way, we need to find the 3D X, Y of the point. Z we know via a stereo cam in real life, or here in simulation we already know it
            // We use the image_geometry and tf packages for the same. Image geometry does that for us using the intrinsic matrix
            // For the extrinsic part, we need to use tf to transform to world coordinates 

            c.pose.position_m[0] = lastGoodXYZ_[0];
            c.pose.position_m[1] = lastGoodXYZ_[1];
            c.pose.position_m[2] = lastGoodXYZ_[2];

            cout << "flag_last_detected_line_== 1 " << endl; 
        }   
        else // the quad is flying around in the beginning and no target should be specified 
        {
            c.rates.velocity_mps[0] = 0;
            c.rates.velocity_mps[1] = speed_;
            c.rates.velocity_mps[2] = 0;
        }
        // publish back the current pose. So as to make the quad stay at its place
    }
    }


    imshow(hough_prob_window, hough_prob_result);
    cv::waitKey(1); 


    c.rates.velocity_mps.normalize();
    c.rates.velocity_mps *= speed_;

    sl.push_back(c);
    path_.setLinear(sl);
    t = path_.toMsg();
    t.header.stamp = ros::Time::now();
    path_pub_.publish(t);
}