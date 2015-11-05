#ifndef LINE_DETECTOR_H
#define LINE_DETECTOR_H
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include <sstream>

using namespace cv;

// from castacks::math_utils::numeric_operations
double safe_division(double a, double b) {
  if (fabs(b) > std::numeric_limits<double>::epsilon())
    return a/b;
  else
    return a/std::numeric_limits<double>::epsilon();
}

struct Coordinate
{
    double x_;
    double y_;

    Coordinate(){}
    Coordinate(double x, double y){x_=x; y_=y;}
};

struct Line
{
    struct Coordinate start_point_;
    struct Coordinate end_point_;
    double length_;
    double slope_;
    double angle_;
    double intercept_;
    double dist_from_origin_;
    int strength_;
    int struct_image_width_;
    int struct_image_height_;

    /* TODO this constructor can prove to be costly.
    Is it better to calculate all this in separate member functions?
    But these are minor calculations anyway, so meh. */
    void evaluate()
    {
        length_ = sqrt(pow(abs(end_point_.x_ - start_point_.x_), 2) + pow(abs(end_point_.y_ - start_point_.x_), 2));
        
        /* TODO a seg fault is bound occur if den = 0  or close to zero. Check what defines close */
        // return(std::min( num/std::max(den, std::numeric_limits<double>::epsilon()) , std::numeric_limits<double>::max() )
        if(!abs(start_point_.x_ - end_point_.x_) < 3) /* these are just pixel values */
        { 
            slope_ = safe_division( (end_point_.y_ - start_point_.y_) , (end_point_.x_ - start_point_.x_) );
            
            // [-pi, pi] with quadrant sign
            angle_ = std::floor( atan2( (end_point_.y_ - start_point_.y_ ),  (end_point_.x_ - start_point_.x_) ) * 180/M_PI);
          
            // [0, pi]. We don't care about quadrant  
            if(angle_ < 0)
            {
                angle_ = angle_ + 180; // still don't use atan as it's not robust to division by (close to) zero
            }
        }
        else
        { 
            slope_ = 1000.0; /* not sure about this */
            angle_ = 90;
        }

        intercept_ = (-slope_ * (start_point_.x_)) + (start_point_.y_); 
        // dist from image center : (struct_image_width_/2, image_height/2)
        // dist = | y-mx-c | / sqrt(1+m^2)
        dist_from_origin_ = abs( safe_division( ( (struct_image_height_/2) - (slope_ * struct_image_width_/2) - intercept_),  (sqrt(1 + pow(slope_,2))) ) );
        strength_ = 1; /* one line detected implies strength is 1 */
    }

    Line(){};
    
    Line(Vec4i opencv_line, int image_width, int image_height)
    {
        start_point_ = Coordinate(opencv_line[0], opencv_line[1]);
        end_point_ = Coordinate(opencv_line[2], opencv_line[3]);
        struct_image_width_ = image_width;
        struct_image_height_ = image_height; 
        evaluate();      
    }

    Line(Coordinate start_point, Coordinate end_point, int image_width, int image_height)
    {
        start_point_ = start_point;
        end_point_ = end_point;
        struct_image_width_ = image_width;
        struct_image_height_ = image_height; 
        evaluate();      
    }

};

class Line_detector
{
    public:
        Line_detector(std::vector<Vec4i> original_lines, int image_width, int image_height); /* ensure a vector is passed if only one line is spitted out by HT */
        cv::Vec4i remove_duplicates(); // returns vector of openCV lines 
        std::vector<cv::Vec4i> from_Lines_struct_to_opencv_lines(std::vector<Line> line_struct_vector); // utility method to convert from variable unique_lines_ to a vector of openCV lines (vector<Vec4i>) 
        cv::Vec4i from_Lines_struct_to_opencv_lines(Line line_struct);
        std::vector<double> return_unique_angles();
        std::vector<double> return_original_angles();
        double return_best_angle();
        double return_best_dist_from_origin();
        std::vector< std::vector<double> > return_original_angle_and_dist_from_origin();

        std::vector<Line> original_lines_;
        std::vector<Line> unique_lines_;
        Line best_line_;

    private:
        double threshold_length_; /* this threshold defines our (fake) "ROI" */  
        // dunno if this is a good idea : Rect ROI_; 
        int class_image_width_;
        int class_image_height_;
};

#endif /* LINE_DETECTOR_H */