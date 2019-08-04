#include "rapid_pbd/ar_detection_action.h"

#include <sstream>
#include <string>

#include "actionlib/server/simple_action_server.h"
#include "pcl/common/common.h"
#include "pcl_ros/transforms.h"
#include "rapid_pbd/action_names.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#include "ar_track_alvar_msgs/AlvarMarker.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

namespace rapid {
namespace pbd {

ARDetectionAction::ARDetectionAction(const std::string& topic)
    : topic_(topic),
      as_(kARDetectionActionName,
        boost::bind(&ARDetectionAction::Execute, this, _1), false),
      nh_() {}

void ARDetectionAction::Start() {
  as_.start();
}

void ARDetectionAction::Execute(
    const rapid_pbd_msgs::DetectARTagsGoalConstPtr& goal) {

  rapid_pbd_msgs::DetectARTagsResult result;
  boost::shared_ptr<const ar_track_alvar_msgs::AlvarMarkers> tags;
  tags = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>(topic_, ros::Duration(5.0));

  if (!tags) {
    ROS_ERROR("Failed to detect AR tags.");
    as_.setSucceeded(result);
    return;
  }

  for (size_t i = 0; i < tags->markers.size(); i++) {
    ar_track_alvar_msgs::AlvarMarker tag = tags->markers[i];

    rapid_pbd_msgs::Landmark landmark;
    landmark.type = rapid_pbd_msgs::Landmark::AR_TAG;

    std::stringstream ss;
    ss << "Tag " << tag.id;
    landmark.name = ss.str();
    landmark.ar_tag_id = tag.id;
    landmark.pose_stamped = tag.pose;
    landmark.pose_stamped.header = tag.header;
    landmark.surface_box_dims.x = 0.045;
    landmark.surface_box_dims.y = 0.045;
    landmark.surface_box_dims.z = 0.01;

    result.ar_tags.push_back(landmark);
  }

  bool success = result.ar_tags.size() > 0;
  if (!success) {
    ROS_ERROR("Failed to detect AR tags.");
    as_.setSucceeded(result);
    return;
  }

  ROS_INFO("Detected %ld AR tags", result.ar_tags.size());
  as_.setSucceeded(result);
}
}  // namespace pbd
}  // namespace rapid
