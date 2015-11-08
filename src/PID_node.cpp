// This node wil find the power line via the probabilistic Hough tranform by imposing some conditions so as to find a single line only. 
// It merges and eliminates near-duplicate(parallel and close due to thickness) and fragmented lines 
// Then it will compute the angle and the distance from the center of the image.

// Meanwhile it also subscribes to data from Matrice and publishes to it 
// Then it will instantiate a PID_controller object to calculate the velocity commands to be published to Matrice

// Doesn't seem a good design and this should be broken up into :
// - a Hough node which subscribes to the Guidance data, which will publish the angle and center distance
// - a PID node that subscribes and publishes to the Matrice. It will send out the position commands
// - future : a trajectory generator node

// There should be protection for edge cases and right now there should be speed and position limits. 

#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic
#include <PID_controller.h>

ros::Subscriber left_image_sub;
ros::Subscriber right_image_sub;
ros::Subscriber depth_image_sub;
ros::Subscriber imu_sub;
ros::Subscriber velocity_sub;
ros::Subscriber obstacle_distance_sub;
ros::Subscriber ultrasonic_sub;

using namespace cv;
using namespace std;
#define WIDTH 320
#define HEIGHT 240

/* Them global variables*/ 
/* TODO Remove them. Maybe create trackbars in the constructor of the HoughLine class itself.*/
/* Trackbars and their lower and upper limits */
int lower_min_no_of_intersections_trackbar = 1;
int upper_min_no_of_intersections_trackbar = 1000;
int hough_stand_trackbar = upper_min_no_of_intersections_trackbar;

int lower_hough_prob_min_no_of_intersections_trackbar = 1;
int upper_hough_prob_min_no_of_intersections_trackbar = 1000;
int hough_prob_min_no_of_intersections_trackbar = 100; //initial set point is upper limit by default

int lower_hough_prob_min_no_of_points_trackbar = 1;
int upper_hough_prob_min_no_of_points_trackbar = 1000;
int hough_prob_min_no_of_points_trackbar = 100;

int lower_hough_prob_max_gap_bw_points_trackbar = 1;
int upper_hough_prob_max_gap_bw_points_trackbar = 1000;
int hough_prob_max_gap_bw_points_trackbar = upper_hough_prob_max_gap_bw_points_trackbar;

/* Window names */
const char* hough_standard_window = "Standard Hough";
const char* hough_prob_window = "Probabilistic Hough";

/* left greyscale image */
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("left_image", cv_ptr->image);

    /* Hackish code. 
    To make it cleaner, my future self will implement a class, and load params.
    Near future self should extract a HoughLineProb sub method  */

    /* Them holder variables */
    Mat image_original = cv_ptr->image; 
    Mat image_after_canny; 
    Mat hough_standard_result; 
    Mat hough_prob_result; 

    /* Them future paramaters */

    /* Them relevant variables*/

    vector<Vec4i> lines;
    vector<float> lengths;
    vector<float> angles; 
    vector<float> distance_from_origin;
    vector<int> x_1_points;
    vector<int> x_2_points;
    vector<int> y_1_points;
    vector<int> y_2_points;
    vector<float> slopes_line;

    char thresh_label_hough_standard[50];
    char thresh_label_hough_prob_1[50];
    char thresh_label_hough_prob_2[50];
    char thresh_label_hough_prob_3[50];
    sprintf(thresh_label_hough_standard, "min_no_of_intersections");  
    sprintf(thresh_label_hough_prob_1, "min_no_of_intersections");  
    sprintf(thresh_label_hough_prob_2, "min_no_of_points_in_line");  
    sprintf(thresh_label_hough_prob_3, "max_gap_between_points_in_line");  

    // namedWindow(hough_standard_window, 1);
    // createTrackbar(thresh_label_hough_standard, hough_standard_window, &hough_stand_trackbar, upper_min_no_of_intersections_trackbar, Standard_Hough);
    
    namedWindow(hough_prob_window, 1);
    createTrackbar(thresh_label_hough_prob_1, hough_prob_window, &hough_prob_min_no_of_intersections_trackbar, upper_min_no_of_intersections_trackbar);
    createTrackbar(thresh_label_hough_prob_2, hough_prob_window, &hough_prob_min_no_of_points_trackbar, upper_hough_prob_min_no_of_points_trackbar);
    createTrackbar(thresh_label_hough_prob_3, hough_prob_window, &hough_prob_max_gap_bw_points_trackbar, upper_hough_prob_max_gap_bw_points_trackbar);
    Canny(image_original, image_after_canny, 50, 200, 3);
    cvtColor(image_after_canny,hough_prob_result, COLOR_GRAY2BGR );
    HoughLinesP(image_after_canny, lines, 1, CV_PI/180, lower_hough_prob_min_no_of_intersections_trackbar + hough_prob_min_no_of_intersections_trackbar, lower_hough_prob_min_no_of_points_trackbar + hough_prob_min_no_of_points_trackbar, lower_hough_prob_max_gap_bw_points_trackbar + hough_prob_max_gap_bw_points_trackbar);


    if(!lines.empty())
    {
        for(size_t i = 0; i < lines.size(); i++)
        {
            Vec4i l = lines[i];
            line(hough_prob_result, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 3, CV_AA); // plots line in blue color
            circle(hough_prob_result, Point(l[0],l[1]), 10, Scalar(0,0,255), 1, 8); // plots red circle at first end point
            circle(hough_prob_result, Point(l[2],l[3]), 10, Scalar(0,255,0), 1, 8); // plots green circle at second end point

            cout << "no of detected lines is "<< lines.size()<<endl;
            float den = l[2] - l[0];
            cout << "angle of line number "<< i+1 << " is " << atan2( (l[3]-l[1]), (l[2]-l[0]) ) * 180/M_PI <<endl;
            
            x_1_points.push_back(l[0]);
            y_1_points.push_back(l[1]);
            x_2_points.push_back(l[2]);
            y_2_points.push_back(l[3]);
     
            angles.push_back(atan2( (l[3]-l[1]), (l[2]-l[0]) ) * 180/M_PI );
            // slopes_line.push_back( (l[3]-l[1]) / (l[2]-l[0]) );
            float current_angle = atan2( (l[3]-l[1]), (l[2]-l[0]) ) * 180/M_PI;
            float slope_line = (l[3]-l[1]) / (l[2]-l[0]);
            float c_line = (-slope_line * l[0]) + l[1];
            // polar coordinates "r" : distance  of line y=mx+c from origin is |c|/rt(1+m^2)
            distance_from_origin.push_back( abs(c_line) / sqrt(1 + pow(slope_line,2)) );

            // Now to keep the quad level, we first will change only the yaw. 
            // After aligning the line vertically on the image, we'll make it translate in the "X" direction, so as to bring it in the center.
            // Finally, we'll fly upwards and let the gripper do its thing automatically.
        }   

        // Following is O(nlogn + n). Can't think of a better way. Anyway, it doesn't really matter
        // It removes the lines that have similar angles 
        
        std::sort(angles.begin(), angles.end());
        auto last = unique(angles.begin(), angles.end(), [](double l, double r) { return std::abs(l - r) < 5; });  // lambda func
        // equivalent if falling within 5 degrees on either side. 

        auto first = angles.begin();
        while(first != last) 
        {
            cout << *first++ << ' ' << endl; 
        }

        // remove the duplicates (within the specified range)
        angles.erase(unique(angles.begin(), angles.end(), [](double l, double r) { return std::abs(l - r) < 5; }), angles.end());
        cout << "no of parallel (or unique lines, depending on the use case are "  << angles.size() << endl;
      
        // For later, if there's a need to merger the lines, we need to store the indices of the parallel ones, and then we can 
        // say, take the average of the endpoints.
        // In some cases, line might be fragmented, so we could have checks for P2 of line 1 lying in vicinity of P1 of line 2
        // If they are within a predefined "circle of closeness", we can merge the fragments to form a longer line. 

        /*
        vector<size_t> index(angles.size());
        for (size_t i = 0; i != index.size(); ++i) index[i] = i;

        // sort indexes based on comparing values in angles
        sort(index.begin(), index.end(),[&angles](size_t i1, size_t i2) {return angles[i1] < angles[i2];});

        for (auto i = index.begin(); i != index.end(); ++i)
            std::cout << *i << ' ';*/
    }
        imshow(hough_prob_window, hough_prob_result);

    cv::waitKey(1);
}

/* right greyscale image */
void right_image_callback(const sensor_msgs::ImageConstPtr& right_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("right_image", cv_ptr->image);
    cv::waitKey(1);
}

/* depth greyscale image */
void depth_image_callback(const sensor_msgs::ImageConstPtr& depth_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth8(HEIGHT, WIDTH, CV_8UC1);
    cv_ptr->image.convertTo(depth8, CV_8UC1);
    cv::imshow("depth_image", depth8);
    cv::waitKey(1);
}

/* imu */
void imu_callback(const geometry_msgs::TransformStamped& g_imu)
{ 
    printf( "frame_id: %s stamp: %d\n", g_imu.header.frame_id.c_str(), g_imu.header.stamp.sec );
    printf( "imu: [%f %f %f %f %f %f %f]\n", g_imu.transform.translation.x, g_imu.transform.translation.y, g_imu.transform.translation.z, 
						g_imu.transform.rotation.x, g_imu.transform.rotation.y, g_imu.transform.rotation.z, g_imu.transform.rotation.w );
}

/* velocity */
void velocity_callback(const geometry_msgs::Vector3Stamped& g_vo)
{ 
    printf( "frame_id: %s stamp: %d\n", g_vo.header.frame_id.c_str(), g_vo.header.stamp.sec );
    printf( "velocity: [%f %f %f]\n", g_vo.vector.x, g_vo.vector.y, g_vo.vector.z );
}

/* obstacle distance */
void obstacle_distance_callback(const sensor_msgs::LaserScan& g_oa)
{ 
    printf( "frame_id: %s stamp: %d\n", g_oa.header.frame_id.c_str(), g_oa.header.stamp.sec );
    printf( "obstacle distance: [%f %f %f %f %f]\n", g_oa.ranges[0], g_oa.ranges[1], g_oa.ranges[2], g_oa.ranges[3], g_oa.ranges[4] );
}

/* ultrasonic */
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{ 
    printf( "frame_id: %s stamp: %d\n", g_ul.header.frame_id.c_str(), g_ul.header.stamp.sec );
    for (int i = 0; i < 5; i++)
        printf( "ultrasonic distance: [%f]  reliability: [%d]\n", g_ul.ranges[i], (int)g_ul.intensities[i] );
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "GuidanceNodeTest");
    ros::NodeHandle my_node;

    left_image_sub        = my_node.subscribe("/guidance/left_image",  10, left_image_callback);
    right_image_sub       = my_node.subscribe("/guidance/right_image", 10, right_image_callback);
    depth_image_sub       = my_node.subscribe("/guidance/depth_image", 10, depth_image_callback);
    imu_sub               = my_node.subscribe("/guidance/imu", 1, imu_callback);
    velocity_sub          = my_node.subscribe("/guidance/velocity", 1, velocity_callback);
    obstacle_distance_sub = my_node.subscribe("/guidance/obstacle_distance", 1, obstacle_distance_callback);
    ultrasonic_sub        = my_node.subscribe("/guidance/ultrasonic", 1, ultrasonic_callback);

    PID_controller pid_test;

    while (ros::ok())
        ros::spinOnce();

    return 0;
}
