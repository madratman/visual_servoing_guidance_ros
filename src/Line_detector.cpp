#include "Line_detector.h"
using namespace std;

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
        current_line[0] = line_struct_vector[i].start_point_.x_;
        current_line[1] = line_struct_vector[i].start_point_.y_;
        current_line[2] = line_struct_vector[i].end_point_.x_;
        current_line[3] = line_struct_vector[i].end_point_.y_;
    
        opencv_lines.push_back(current_line);
    }
    return opencv_lines;
}   

cv::Vec4i Line_detector::from_Lines_struct_to_opencv_lines(Line line_struct)
{
    cv::Vec4i opencv_line;

    opencv_line[0] = line_struct.start_point_.x_;
    opencv_line[1] = line_struct.start_point_.y_;
    opencv_line[2] = line_struct.end_point_.x_;
    opencv_line[3] = line_struct.end_point_.y_;
    
    return opencv_line;
}   

cv::Vec4i Line_detector::remove_duplicates()
{
    std::vector<Line> no_duplicates(original_lines_.size()); 

    for(size_t j = 0; j < original_lines_.size(); j++)
    {
        Line current_line(original_lines_[j]);
        
        if(no_duplicates.size() > 0)
        {
            int counter = 0;
            for(int i = 0; i < no_duplicates.size(); i++)
            {   
                // cout << "current_line.intercept_  " << current_line.intercept_ << endl;
                // cout << "no_duplicates[ " << i <<" ].intercept_ "<< no_duplicates[i].intercept_<<endl;
                // cout << "current_line.slope_" << current_line.slope_ <<endl;
                // cout << "no_duplicates[ " << i <<" ].slope_ "<< no_duplicates[i].slope_<<endl;
                // cout << "no_duplicates[ " << i <<" ].angle_ "<< no_duplicates[i].angle_<<endl;
                /* What is the best comparator? */
                if(abs(no_duplicates[i].slope_ - current_line.slope_) < 0.2 && abs(no_duplicates[i].intercept_ - current_line.intercept_) < 10)
                {   
                    // cout <<"duplicate found, i =  " << i <<endl;
                    Coordinate new_start_point, new_end_point;
                    if(current_line.slope_ > 0)
                    {
                        new_start_point.x_ = std::min(current_line.start_point_.x_, no_duplicates[i].start_point_.x_);
                        new_start_point.y_ = std::min(current_line.start_point_.y_, no_duplicates[i].start_point_.y_);
                        new_end_point.x_ = std::max(current_line.start_point_.x_, no_duplicates[i].start_point_.x_);
                        new_end_point.y_ = std::max(current_line.start_point_.y_, no_duplicates[i].start_point_.y_);
                    }
                    else
                    {
                        new_start_point.x_ = std::min(current_line.start_point_.x_, no_duplicates[i].start_point_.x_);
                        new_start_point.y_ = std::max(current_line.start_point_.y_, no_duplicates[i].start_point_.y_);
                        new_end_point.x_ = std::max(current_line.start_point_.x_, no_duplicates[i].start_point_.x_);
                        new_end_point.y_ = std::min(current_line.start_point_.y_, no_duplicates[i].start_point_.y_);
                    }

                    no_duplicates[i].start_point_ = new_start_point;
                    no_duplicates[i].end_point_ = new_end_point;
                    no_duplicates[i].evaluate(); // default strength is one
                    no_duplicates[i].strength_ += 1;

                    counter += 1;
                    // std::cout<<counter<<std::endl;
                    break;
                    unique_lines_.push_back(no_duplicates[i]);
                }
            }
            if(counter)
                continue;
        }

        // Else just append it to no_duplicates lines' vector
        Line new_line(current_line);
        // save std::vector<Line> memeber 
        unique_lines_.push_back(new_line);
    }

   // Best line
    Line best = Line(Coordinate(0,0), Coordinate(0,0), class_image_width_, class_image_height_);

    for( int i = 0; i < unique_lines_.size() ; i++ )
    {
        if( (unique_lines_[i].strength_ * 10 + unique_lines_[i].length_) > (best.strength_ * 10 + best.length_) )
        {
            best = unique_lines_[i];
        }
    }

    // Save the best line as the member variable of Line_detector object
    best_line_ = best;

    // convert back to openCV lines and return 
    Vec4i best_opencv_line = from_Lines_struct_to_opencv_lines(best);
    
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