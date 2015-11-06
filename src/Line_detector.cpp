#include "Line_detector.h"  
using namespace std;

double Line_detector::transform_x_coordinate_to_opencv(double x_orig)
{   
    return x_orig + (class_image_width_/2);
}

double Line_detector::transform_y_coordinate_to_opencv(double y_orig)
{
    return (class_image_height_/2) - y_orig;
}

Line_detector::Line_detector(std::vector<Vec4i> opencv_lines, int image_width, int image_height)
{
    class_image_width_ = image_width;
    class_image_height_ = image_height;

    for (int i=0; i<opencv_lines.size();i++)
    {
        // converts current opencv line to Line struct 
        Line current_line(opencv_lines[i], class_image_width_, class_image_height_);
        original_lines_.push_back(current_line);
    }
    std::cout << "no of original_lines_ " << original_lines_.size() << std::endl;
}

std::vector<cv::Vec4i> Line_detector::from_Lines_struct_to_opencv_lines(std::vector<Line> line_struct_vector)
{
    std::vector<cv::Vec4i> opencv_lines;

    for (int i=0; i<line_struct_vector.size();i++)
    {   
        cv::Vec4i current_line; 
        current_line[0] = transform_x_coordinate_to_opencv(line_struct_vector[i].start_point_.x_);
        current_line[1] = transform_y_coordinate_to_opencv(line_struct_vector[i].start_point_.y_);
        current_line[2] = transform_x_coordinate_to_opencv(line_struct_vector[i].end_point_.x_);
        current_line[3] = transform_y_coordinate_to_opencv(line_struct_vector[i].end_point_.y_);
    
        opencv_lines.push_back(current_line);
    }
    return opencv_lines;
}   

cv::Vec4i Line_detector::from_Lines_struct_to_opencv_lines(Line line_struct)
{
    cv::Vec4i opencv_line;

    opencv_line[0] = transform_x_coordinate_to_opencv(line_struct.start_point_.x_);
    opencv_line[1] = transform_y_coordinate_to_opencv(line_struct.start_point_.y_);
    opencv_line[2] = transform_x_coordinate_to_opencv(line_struct.end_point_.x_);
    opencv_line[3] = transform_y_coordinate_to_opencv(line_struct.end_point_.y_);
    
    return opencv_line;
}   

cv::Vec4i Line_detector::remove_duplicates()
{
    // cout << "original_lines_.size() " << original_lines_.size()<<endl;
    Coordinate new_start_point;
    Coordinate new_end_point;

    for(size_t j = 0; j < original_lines_.size(); j++)
    {
        Line current_line(original_lines_[j]);

        new_start_point = current_line.start_point_;
        new_end_point = current_line.end_point_;

        for(int i = 0; i < j; i++)
        {   
            // cout << "j, i " << j << ", " << i << endl;  
            // cout << "current_line.angle_" << current_line.angle_ << endl;
            // cout << "original_lines_[i].angle_" << original_lines_[i].angle_ << endl;
            // cout << "current_line.intercept_" << current_line.intercept_ << endl;
            // cout << "original_lines_[i].intercept_" << original_lines_[i].intercept_ << endl;

            // cout << "current_line.slope_" << current_line.slope_ << endl;
            // cout << "original_lines_[i].slope_" << original_lines_[i].slope_ << endl;

            // cout << "current_line.start_point_" << current_line.start_point_.x_ << ", " << current_line.start_point_.y_ << endl;
            // cout << "original_lines_[i].start_point_" << original_lines_[i].start_point_.x_ << ", "<< original_lines_[i].start_point_.y_ << endl;

            // cout << "current_line.end_point_" << current_line.end_point_.x_ << ", " << current_line.end_point_.y_ << endl;
            // cout << "original_lines_[i].end_point_" << original_lines_[i].end_point_.x_ << ", "<< original_lines_[i].end_point_.y_ << endl;
        
            // What is the best comparator for almost parallel lines and close
            if(abs(original_lines_[i].angle_ - current_line.angle_) < 10)
            {   
                cout <<"duplicate found, j, i =  " << j << ", "<< i <<endl;    
                // If lines are almost parallel, we combine them to find the longest line. Self explanatory code block. 
                // less than zero, as in opencv the Y axis is flipped
                if(current_line.slope_ > 0)
                {
                    new_start_point.x_ = std::min(new_start_point.x_, std::min(current_line.start_point_.x_, original_lines_[i].start_point_.x_));
                    new_start_point.y_ = std::min(new_start_point.y_, std::min(current_line.start_point_.y_, original_lines_[i].start_point_.y_));
                    new_end_point.x_ = std::max(new_end_point.x_, std::max(current_line.start_point_.x_, original_lines_[i].start_point_.x_));
                    new_end_point.y_ = std::max(new_end_point.y_, std::max(current_line.start_point_.y_, original_lines_[i].start_point_.y_));
                    cout<< "slope > 0 " <<endl;
                }
                else
                {
                    cout<< "slope < 0 " <<endl;
                    new_start_point.x_ = std::min(new_start_point.x_, std::min(current_line.start_point_.x_, original_lines_[i].start_point_.x_));
                    new_start_point.y_ = std::max(new_start_point.y_, std::max(current_line.start_point_.y_, original_lines_[i].start_point_.y_));
                    new_end_point.x_ = std::max(new_end_point.x_, std::max(current_line.start_point_.x_, original_lines_[i].start_point_.x_));
                    new_end_point.y_ = std::min(new_end_point.y_, std::min(current_line.start_point_.y_, original_lines_[i].start_point_.y_));
                }

                cout << new_start_point.x_ << ", " << new_start_point.y_ << endl;
            }
        }

        best_line_.start_point_ = new_start_point;
        best_line_.end_point_ = new_end_point;
        best_line_.evaluate(); 
        best_line_.strength_ = 1;
    }

    // convert back to openCV lines and return 
    Vec4i best_opencv_line = from_Lines_struct_to_opencv_lines(best_line_);
    
    return best_opencv_line;
}

std::vector<double> Line_detector::return_original_angles()
{
    std::vector<double> original_angles;
    for(auto current_line = original_lines_.begin(); current_line != original_lines_.end(); current_line++)
    {
        original_angles.push_back(current_line -> angle_);

    }
    return original_angles;
}

std::vector<double> Line_detector::return_unique_angles()
{
    std::vector<double> unique_angles;
    for(auto current_line = unique_lines_.begin(); current_line != unique_lines_.end(); current_line++)
    {
        unique_angles.push_back(current_line -> angle_);
    }
    return unique_angles;
}

std::vector< std::vector<double> > Line_detector::return_original_angle_and_dist_from_origin()
{
    std::vector< std::vector<double> > result;
    result.resize(original_lines_.size());

    for(int i = 0; i < original_lines_.size(); i++)
    {
        auto current_line = original_lines_[i];
        result[i].resize(2);
        result[i][0] = current_line.dist_from_origin_;
        result[i][1] = current_line.angle_;
    }
    return result;
}

double Line_detector::return_best_angle()
{
    double best_angle;
    best_angle = best_line_.angle_;
    return best_angle;
}

double Line_detector::return_best_dist_from_origin()
{
    double best_dist_from_origin_;
    best_dist_from_origin_ = best_line_.dist_from_origin_;
    return best_dist_from_origin_;
}

double Line_detector::return_best_intercept()
{
    double best_intercept;
    best_intercept = best_line_.intercept_;
    return best_intercept;
}