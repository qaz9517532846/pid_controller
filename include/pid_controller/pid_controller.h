#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>
#include <pid_controller/PIDControllerConfig.h>

class PID_Controller
{
    public:
      PID_Controller(std::string name);
      ~PID_Controller();

      void reconfig_callback(pid_controller::PIDControllerConfig &config, uint32_t level);

    private:
      double calculate_error(double input, double actual_output);
      double P_Controller(double kp, double error);
      double I_Controller(double ki, double error, double dt);
      double D_Controller(double kd, double error, double dt);
      double PID_result(double PID_calculate, double max, double min);

      void ControlInput_Callback(const std_msgs::Float64::ConstPtr& msg);
      void ControlOutput_Callback(const std_msgs::Float64::ConstPtr& msg);

      double kp_value, ki_value, kd_value, dt_value, max_value, min_value; 
      double setpoint, output, error, integral;
      double pre_error = 0;
      
      ros::Subscriber Controller_input;
      ros::Subscriber Controller_output;

      ros::Publisher PIDController_result;
};