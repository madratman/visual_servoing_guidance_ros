#include "PID_controller.h"

double PID_controller::calculate( double setpoint, double observed )
{
    double error_current = setpoint - observed;

    if(type_of_variable == "angle")
    {
        // TODO check for rad to deg if required
        // TODO should error be max 360 or 180. Right now keeping it to max 180. 
        if(error_current > 180)
           error_current = 360 - error_current;
        else if(error_current < -180)
           error_current = error_current + 360;
        else
           error_current = error_current;
    }

    double error_prop = Kp_ * error_current;

    // step_error_integral_ += error_current*time_step_;
    // double error_integral = Ki_ * step_error_integral_;

    double step_error_derivative = (error_current - error_previous_) / time_step_;
    double error_derivative = Kd_ * step_error_derivative;

    double output = error_prop + error_derivative;

    // TODO Maybe we need separate caps on the integral term.z 
    if( output > max_ )
        output = max_;
    else if( output < min_ )
        output = min_;

    error_previous_ = error_current;

    return output;
}

void PID_controller::set_gains(double Kp, double Kd)
{
    Kp_ = Kp;
    // Ki_ = Ki;
    Kd_ = Kd;
} 

// TODO could do in future as needed
// Make some get params to get the value of coeff, etc
// Maybe setGains might be needed to be broken into 3 parts. 
// Time should be taken from DJI API's return header/stamp/Hz and hence, it should be a class variable 
