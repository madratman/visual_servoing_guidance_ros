#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "Line_detector.h"

ros::Subscriber image_subscriber;

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

// TODO Make a class. Publishing in callback with global Publisher is meh. 
ros::Publisher quad_pose_pub; 

void image_callback(const sensor_msgs::ImageConstPtr& left_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    // try 
    // {
        cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
    // }
    // catch (cv_bridge::Exception& e) 
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }

    imshow("image_upper_cam", cv_ptr->image);

    Mat image_original = cv_ptr->image; 
    Mat image_after_canny; 
    Mat hough_standard_result; 
    Mat hough_prob_result; 

    vector<Vec4i> opencv_lines; // lines detected by hough transform
    vector<Vec4i> unique_lines; // unique lines returned by Line_detector::remove_duplicates()
    Vec4i best_line;

    vector<float> lengths;
    vector<float> angles; 
    vector<float> distance_from_origin;
    vector<int> x_1_points;
    vector<int> x_2_points;
    vector<int> y_1_points;
    vector<int> y_2_points;
    vector<float> slopes_line;

    char thresh_label_hough_prob_1[50];
    char thresh_label_hough_prob_2[50];
    char thresh_label_hough_prob_3[50];
    sprintf(thresh_label_hough_prob_1, "min_no_of_intersections");  
    sprintf(thresh_label_hough_prob_2, "min_no_of_points_in_line");  
    sprintf(thresh_label_hough_prob_3, "max_gap_between_points_in_line");  
    
    namedWindow(hough_prob_window, 1);
    createTrackbar(thresh_label_hough_prob_1, hough_prob_window, &hough_prob_min_no_of_intersections_trackbar, upper_hough_prob_min_no_of_intersections_trackbar);
    createTrackbar(thresh_label_hough_prob_2, hough_prob_window, &hough_prob_min_no_of_points_trackbar, upper_hough_prob_min_no_of_points_trackbar);
    createTrackbar(thresh_label_hough_prob_3, hough_prob_window, &hough_prob_max_gap_bw_points_trackbar, upper_hough_prob_max_gap_bw_points_trackbar);

    Canny(image_original, image_after_canny, 50, 200, 3);
    cvtColor(image_after_canny, hough_prob_result, COLOR_GRAY2BGR);
    HoughLinesP(image_after_canny, opencv_lines, 1, CV_PI/180, lower_hough_prob_min_no_of_intersections_trackbar + hough_prob_min_no_of_intersections_trackbar, lower_hough_prob_min_no_of_points_trackbar + hough_prob_min_no_of_points_trackbar, lower_hough_prob_max_gap_bw_points_trackbar + hough_prob_max_gap_bw_points_trackbar);

    // Filter out duplicates : combine fragment lines + multiple parallel lines
    Line_detector filtered_lines(opencv_lines);
    best_line = filtered_lines.remove_duplicates();
    line(hough_prob_result, Point(best_line[0], best_line[1]), Point(best_line[2], best_line[3]), Scalar(255,0,0), 1, CV_AA);
    circle(hough_prob_result, Point(best_line[0],best_line[1]), 10, Scalar(0,0,255), 1, 8); // plots red circle at first end point
    circle(hough_prob_result, Point(best_line[2],best_line[3]), 10, Scalar(0,255,0), 1, 8); // plots green circle at second end point

    cout << "best_line.angle() " << filtered_lines.return_best_angle() << endl;

    imshow(hough_prob_window, hough_prob_result);

    // if(best_line)
    // {

    // }
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "GuidanceNodeTest");
    ros::NodeHandle my_node;

    // todo : topic is "downward". Remap. 
    image_subscriber = my_node.subscribe("/downward_cam/camera/image",  10, image_callback);

    while (ros::ok())
        ros::spinOnce();

    return 0;
}
