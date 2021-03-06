#include <pid_controller/pid_controller.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_controller");

  PID_Controller pid_controller(ros::this_node::getName());

  return 0;
}