#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <string>

class PID_controller
{
    public:
        PID_controller(){};
        // PID_controller(double Kp, double Ki, double Kd, double time_step, double set_point, double min, double max);
        double calculate(double measured_value, double set_point);
        void set_gains(double Kp, double Kd);
        // ~PID_controller(){};

      private:
        double Kp_;
        // double Ki_;
        double Kd_;
        double time_step_; // parameter to calculate?
        double set_point_; // maybe it shouldn't be a parameter after all
        double min_;
        double max_;
        // double step_error_integral_;
        double error_previous_;
        std::string type_of_variable;
}; 

#endif /* PID_CONTROLLER_H */