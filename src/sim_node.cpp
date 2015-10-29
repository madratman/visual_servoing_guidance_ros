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
    unique_lines = filtered_lines.remove_duplicates();

    cout<<" no of unique_lines " << unique_lines.size()<<endl;

    for(size_t i = 0; i < unique_lines.size(); i++)
    {
        Vec4i l = unique_lines[i];
        line( hough_prob_result, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1, CV_AA);
        circle(hough_prob_result, Point(l[0],l[1]), 10, Scalar(0,0,255), 1, 8); // plots red circle at first end point
        circle(hough_prob_result, Point(l[2],l[3]), 10, Scalar(0,255,0), 1, 8); // plots green circle at second end point
    }

    // line( image_original, Point( best.start.x + this->center.x - 100, best.start.y ), Point( best.end.x + this->center.x - 100, best.end.y ), Scalar(0,255,0), 3, 8);

    // if(!opencv_lines.empty())
    // {
    //     for(size_t i = 0; i < opencv_lines.size(); i++)
    //     {
    //         Vec4i l = opencv_lines[i];
    //         line(hough_prob_result, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 3, CV_AA); // plots line in blue color
    //         circle(hough_prob_result, Point(l[0],l[1]), 10, Scalar(0,0,255), 1, 8); // plots red circle at first end point
    //         circle(hough_prob_result, Point(l[2],l[3]), 10, Scalar(0,255,0), 1, 8); // plots green circle at second end point

    //         cout << "no of detected opencv_lines is "<< opencv_lines.size()<<endl;
    //         float den = l[2] - l[0];
    //         cout << "angle of line number "<< i+1 << " is " << atan2( (l[3]-l[1]), (l[2]-l[0]) ) * 180/M_PI <<endl;
            
    //         x_1_points.push_back(l[0]);
    //         y_1_points.push_back(l[1]);
    //         x_2_points.push_back(l[2]);
    //         y_2_points.push_back(l[3]);
     
    //         angles.push_back(atan2( (l[3]-l[1]), (l[2]-l[0]) ) * 180/M_PI );
    //         // slopes_line.push_back( (l[3]-l[1]) / (l[2]-l[0]) );
    //         float current_angle = atan2( (l[3]-l[1]), (l[2]-l[0]) ) * 180/M_PI;
    //         float slope_line = (l[3]-l[1]) / (l[2]-l[0]);
    //         float c_line = (-slope_line * l[0]) + l[1];
    //         // polar coordinates "r" : distance  of line y=mx+c from origin is |c|/rt(1+m^2)
    //         distance_from_origin.push_back( abs(c_line) / sqrt(1 + pow(slope_line,2)) );

    //         // Now to keep the quad level, we first will change only the yaw. 
    //         // After aligning the line vertically on the image, we'll make it translate in the "X" direction, so as to bring it in the center.
    //         // Finally, we'll fly upwards and let the gripper do its thing automatically.
    //     }   

    //     // Following is O(nlogn + n). Can't think of a better way. Anyway, it doesn't really matter
    //     // It removes the opencv_lines that have similar angles 
        
    //     std::sort(angles.begin(), angles.end());
    //     auto last = unique(angles.begin(), angles.end(), [](double l, double r) { return std::abs(l - r) < 5; });  // lambda func
    //     // equivalent if falling within 5 degrees on either side. 

    //     auto first = angles.begin();
    //     while(first != last) 
    //     {
    //         cout << *first++ << ' ' << endl; 
    //     }

    //     // remove the duplicates (within the specified range)
    //     angles.erase(unique(angles.begin(), angles.end(), [](double l, double r) { return std::abs(l - r) < 5; }), angles.end());
    //     cout << "no of unique opencv_lines are "  << angles.size() << endl;
      
    //     // For later, if there's a need to merger the opencv_lines, we need to store the indices of the parallel ones, and then we can 
    //     // say, take the average of the endpoints.
    //     // In some cases, line might be fragmented, so we could have checks for P2 of line 1 lying in vicinity of P1 of line 2
    //     // If they are within a predefined "circle of closeness", we can merge the fragments to form a longer line. 

    //     /*
    //     vector<size_t> index(angles.size());
    //     for (size_t i = 0; i != index.size(); ++i) index[i] = i;

    //     // sort indexes based on comparing values in angles
    //     sort(index.begin(), index.end(),[&angles](size_t i1, size_t i2) {return angles[i1] < angles[i2];});

    //     for (auto i = index.begin(); i != index.end(); ++i)
    //         std::cout << *i << ' ';*/
    // }

    imshow(hough_prob_window, hough_prob_result);

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
