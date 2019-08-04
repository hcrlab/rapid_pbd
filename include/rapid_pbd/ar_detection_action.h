#ifndef _RAPID_PBD_AR_DETECTION_ACTION_H_
#define _RAPID_PBD_AR_DETECTION_ACTION_H_

#include <string>

#include "actionlib/server/simple_action_server.h"
#include "rapid_pbd_msgs/DetectARTagsAction.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

namespace rapid {
namespace pbd {
class ARDetectionAction {
 public:
  ARDetectionAction(const std::string& topic);
  void Start();
  void Execute(const rapid_pbd_msgs::DetectARTagsGoalConstPtr& goal);

 private:
  std::string topic_;
  actionlib::SimpleActionServer<rapid_pbd_msgs::DetectARTagsAction> as_;
  ros::NodeHandle nh_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_AR_DETECTION_ACTION_H_
