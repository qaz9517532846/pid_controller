#include <pid_controller/pid_controller.h>

PID_Controller::PID_Controller(std::string name)
{
    ros::NodeHandle private_nh("~/" + name);

    Controller_input = private_nh.subscribe("/setpoint", 1000, &PID_Controller::ControlInput_Callback, this);
    Controller_output = private_nh.subscribe("/output", 1000, &PID_Controller::ControlOutput_Callback, this);

    PIDController_result = private_nh.advertise<std_msgs::Float64>("/PIDController_output", 10);

    dynamic_reconfigure::Server<pid_controller::PIDControllerConfig> reconfigure_server_;
    dynamic_reconfigure::Server<pid_controller::PIDControllerConfig>::CallbackType f;

    f = boost::bind(&PID_Controller::reconfig_callback, this, _1, _2);
    reconfigure_server_.setCallback(f);

    while(private_nh.ok())
    {
        ros::spinOnce();
        error = calculate_error(setpoint, output);
        double PID_calculate_ = P_Controller(kp_value, error) + I_Controller(ki_value, error, dt_value) + D_Controller(kd_value, error, dt_value);
        double PID_controller_result = PID_result(PID_calculate_, max_value, min_value);
        std_msgs::Float64 PID_result_;
        PID_result_.data = PID_controller_result;
        PIDController_result.publish(PID_result_);
    }
}

PID_Controller::~PID_Controller()
{

}

void PID_Controller::reconfig_callback(pid_controller::PIDControllerConfig &config, uint32_t level)
{
    kp_value = config.kp;
    ki_value = config.ki;
    kd_value = config.kd;

    dt_value = config.dt;

    max_value = config.max_value;
    min_value = config.min_value;
}

void PID_Controller::ControlInput_Callback(const std_msgs::Float64::ConstPtr& msg)
{
    setpoint = msg->data;
}

void PID_Controller::ControlOutput_Callback(const std_msgs::Float64::ConstPtr& msg)
{
    output = msg->data;
}

double PID_Controller::calculate_error(double input, double actual_output)
{
    double current_error = setpoint - output;
    return current_error;
}

double PID_Controller::P_Controller(double kp, double error)
{
    double P_output = kp * error;
    return P_output;
}

double PID_Controller::I_Controller(double ki, double error, double dt)
{
    integral += error * dt;
    double I_output = ki * integral;
    return I_output;
}

double PID_Controller::D_Controller(double kd, double error, double dt)
{
    double P_output = (error - pre_error) / dt;
    pre_error = error;
    return P_output;
}

double PID_Controller::PID_result(double PID_calculate, double max, double min)
{
    if(PID_calculate >= max)
    {
        return max;
    }
    else if (PID_calculate <= min)
    {
        return min;
    }
    else
    {
        return PID_calculate;
    }
}