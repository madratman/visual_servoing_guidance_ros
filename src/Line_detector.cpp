#include "Line_detector.h"

std::vector<Line> Line_detector::remove_duplicates(std::vector<Vec4i> original_lines)
{
    std::vector<Line>no_duplicates(original_lines.size()); 

    for(size_t i = 0; i < original_lines.size(); i++)
    {
        Line current_line(original_lines[i]);
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
                    break;
                }
            }
            if(counter)
                continue;
        }

        // Else just append it to no_duplicates lines' vector
        Line new_line(current_line);
        no_duplicates.push_back(new_line);
    }

    return no_duplicates;
}