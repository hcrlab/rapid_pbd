#include <string>

#include "rapid_pbd/ar_detection_action.h"
#include "rapid_pbd/robot_config.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;

int main(int argc, char** argv) {
  ros::init(argc, argv, "ar_detection_action_node");
  if (argc < 2) {
    ROS_ERROR("Must supply topic as arg");
    return 1;
  }
  std::string topic(argv[1]);

  std::string robot("");
  bool is_robot_specified = ros::param::get("robot", robot);
  if (!is_robot_specified) {
    ROS_ERROR("robot param must be specified.");
    return 1;
  }

  pbd::RobotConfig* robot_config;
  if (robot == "pr2") {
    robot_config = new pbd::Pr2RobotConfig();
  } else if (robot == "fetch") {
    robot_config = new pbd::FetchRobotConfig();
  } else if (robot == "baxter") {
    robot_config = new pbd::BaxterRobotConfig();
  } else {
    ROS_ERROR("Unsupported robot \"%s\"", robot.c_str());
    return 1;
  }

  pbd::ARDetectionAction action(topic);
  action.Start();
  ros::spin();
  return 0;
}
