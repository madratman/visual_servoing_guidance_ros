#include "Line_detector.h"
using namespace std;

Line_detector::Line_detector(std::vector<Vec4i> opencv_lines)
{
    for (int i=0; i<opencv_lines.size();i++)
    {
        // converts current opencv line to Line struct 
        Line current_line(opencv_lines[i]);
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

std::vector<cv::Vec4i> Line_detector::remove_duplicates()
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
                /* What is the best comparator? */
                if(abs(no_duplicates[i].slope_ - current_line.slope_) < 0.2 && abs(no_duplicates[i].intercept_ - current_line.intercept_) < 10)
                {   
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

    // convert back to openCV lines and return 
    std::vector<Vec4i> opencv_lines = from_Lines_struct_to_opencv_lines(unique_lines_);
    return opencv_lines;
}