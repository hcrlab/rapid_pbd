#include "rapid_pbd/action_executor.h"

#include <sstream>
#include <string>
#include <limits.h>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/GripperCommandAction.h"
#include "Eigen/Eigen"
#include "rapid_pbd_msgs/SegmentSurfacesAction.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

#include "rapid_pbd/action_names.h"
#include "rapid_pbd/action_utils.h"
#include "rapid_pbd/errors.h"
#include "rapid_pbd/landmarks.h"
#include "rapid_pbd/motion_planning.h"
#include "rapid_pbd/motion_planning_constants.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd/world.h"

using actionlib::SimpleActionClient;
using actionlib::SimpleClientGoalState;
using control_msgs::FollowJointTrajectoryAction;
using rapid_pbd_msgs::Action;

namespace msgs = rapid_pbd_msgs;

namespace {
moveit_msgs::CollisionObject GetShelfWall(double max_height, const std::vector<msgs::Surface>& surfaces, double direction = 1.0) {
  geometry_msgs::Point best_position;
  // Default to farthest point in the direction
  best_position.y = direction * std::numeric_limits<float>::max();
  geometry_msgs::Quaternion best_orientation;
  best_orientation.w = 1.0;
  geometry_msgs::Vector3 best_dims;
  best_dims.y = 0.015;
  best_dims.z = max_height;
  std_msgs::Header header;

  for (size_t i = 0; i < surfaces.size(); i++) {
    geometry_msgs::Pose pose = surfaces[i].pose_stamped.pose;
    Eigen::Matrix3f rotation_matrix = Eigen::Quaternionf(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
    Eigen::Vector3f offset = rotation_matrix.col(1);
    Eigen::Vector3f center = Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Vector3f newCenter = center + direction * (offset * surfaces[i].dimensions.y / 2);

    // Find the fathest point at the opposite direction. This point would be on the edge of the most restricted shelf surfaces for motion planning. Use shelf wall from this shelf surface
    if ((direction * newCenter(1)) < (direction * best_position.y)) {
      best_position.x = newCenter(0);
      best_position.y = newCenter(1);
      best_position.z = newCenter(2);

      Eigen::Matrix3f new_rotation;
      new_rotation.col(0) = rotation_matrix.col(0);
      // Assume wall is perfectly vertical
      new_rotation.col(2) = Eigen::Vector3f(0.0, 0.0, 1.0);
      new_rotation.col(1) = new_rotation.col(2).cross(new_rotation.col(0));
      Eigen::Quaternionf quaternion(new_rotation);
      best_orientation.x = quaternion.x();
      best_orientation.y = quaternion.y();
      best_orientation.z = quaternion.z();
      best_orientation.w = quaternion.w();

      best_dims.x = surfaces[i].dimensions.x;
      header = surfaces[i].pose_stamped.header;
    }
  }

  shape_msgs::SolidPrimitive surface_shape;
  surface_shape.type = shape_msgs::SolidPrimitive::BOX;
  surface_shape.dimensions.resize(3);
  surface_shape.dimensions[0] = best_dims.x;
  surface_shape.dimensions[1] = best_dims.y;
  surface_shape.dimensions[2] = best_dims.z;

  moveit_msgs::CollisionObject surface_obj;
  surface_obj.header = header;
  // Assign shelf wall id based on direction
  surface_obj.id = direction > 0 ? "left_wall" : "right_wall";
  surface_obj.primitives.push_back(surface_shape);

  geometry_msgs::Pose pose;
  pose.position = best_position;
  pose.position.z = max_height / 2;
  pose.orientation = best_orientation;
  surface_obj.primitive_poses.push_back(pose);
  surface_obj.operation = moveit_msgs::CollisionObject::ADD;

  return surface_obj;
}

moveit_msgs::CollisionObject GetRightShelfWall(double max_height, const std::vector<msgs::Surface>& surfaces) {
  return GetShelfWall(max_height, surfaces, -1.0);
}

moveit_msgs::CollisionObject GetLeftShelfWall(double max_height, const std::vector<msgs::Surface>& surfaces) {
  return GetShelfWall(max_height, surfaces);
}
}

namespace rapid {
namespace pbd {
ActionExecutor::ActionExecutor(const Action& action,
                               ActionClients* action_clients,
                               MotionPlanning* motion_planning, World* world,
                               const RobotConfig& robot_config,
                               const RuntimeVisualizer& runtime_viz)
    : action_(action),
      clients_(action_clients),
      motion_planning_(motion_planning),
      world_(world),
      robot_config_(robot_config),
      runtime_viz_(runtime_viz) {}

bool ActionExecutor::IsValid(const Action& action) {
  if (action.type == Action::ACTUATE_GRIPPER) {
    if (action.actuator_group == Action::GRIPPER ||
        action.actuator_group == Action::LEFT_GRIPPER ||
        action.actuator_group == Action::RIGHT_GRIPPER) {
    } else {
      PublishInvalidGroupError(action);
      return false;
    }
  } else if (action.type == Action::MOVE_TO_JOINT_GOAL) {
    if (action.actuator_group == Action::ARM ||
        action.actuator_group == Action::LEFT_ARM ||
        action.actuator_group == Action::RIGHT_ARM ||
        action.actuator_group == Action::HEAD) {
    } else {
      PublishInvalidGroupError(action);
      return false;
    }
    if (!HasJointValues(action)) {
      return false;
    }
  } else if (action.type == Action::MOVE_TO_CARTESIAN_GOAL) {
    if (action.actuator_group == Action::ARM ||
        action.actuator_group == Action::LEFT_ARM ||
        action.actuator_group == Action::RIGHT_ARM ||
        action.actuator_group == Action::HEAD) {
    } else {
      PublishInvalidGroupError(action);
      return false;
    }
  } else if (action.type == Action::DETECT_SURFACE_OBJECTS) {
  } else if (action.type == Action::FIND_CUSTOM_LANDMARK) {
  } else {
    ROS_ERROR("Invalid action type: \"%s\"", action.type.c_str());
    return false;
  }
  return true;
}

std::string ActionExecutor::Start() {
  if (action_.type == Action::ACTUATE_GRIPPER) {
    ActuateGripper();
  } else if (action_.type == Action::MOVE_TO_JOINT_GOAL) {
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    GetJointPositions(action_, &joint_names, &joint_positions);
    if (action_.actuator_group == msgs::Action::ARM ||
        action_.actuator_group == msgs::Action::LEFT_ARM ||
        action_.actuator_group == msgs::Action::RIGHT_ARM) {
      return motion_planning_->AddJointGoal(joint_names, joint_positions);
    } else if (action_.actuator_group == Action::HEAD) {
      control_msgs::FollowJointTrajectoryGoal joint_goal;
      joint_goal.trajectory = action_.joint_trajectory;
      joint_goal.trajectory.header.stamp = ros::Time::now();
      SimpleActionClient<FollowJointTrajectoryAction>* client;
      client = &clients_->head_client;
      client->sendGoal(joint_goal);
    } else {
      return "Invalid actuator group";
    }
  } else if (action_.type == Action::MOVE_TO_CARTESIAN_GOAL) {
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    if (HasJointValues(action_)) {
      GetJointPositions(action_, &joint_names, &joint_positions);
    }
    return motion_planning_->AddPoseGoal(action_.actuator_group, action_.pose,
                                         action_.landmark, joint_names,
                                         joint_positions);
  } else if (action_.type == Action::DETECT_SURFACE_OBJECTS) {
    DetectSurfaceObjects();
  }
  return "";
}

bool ActionExecutor::IsDone(std::string* error) const {
  if (action_.type == Action::ACTUATE_GRIPPER) {
    if (action_.actuator_group == Action::GRIPPER) {
      return clients_->gripper_client.getState().isDone();
    } else if (action_.actuator_group == Action::LEFT_GRIPPER) {
      return clients_->l_gripper_client.getState().isDone();
    } else if (action_.actuator_group == Action::RIGHT_GRIPPER) {
      return clients_->r_gripper_client.getState().isDone();
    }
  } else if (action_.type == Action::MOVE_TO_JOINT_GOAL) {
    if (action_.actuator_group == Action::HEAD) {
      return clients_->head_client.getState().isDone();
    } else {
      // Arm motions are controlled by motion planning in the step executor.
      return true;
    }
  } else if (action_.type == Action::DETECT_SURFACE_OBJECTS) {
    bool done = clients_->surface_segmentation_client.getState().isDone();
    if (done) {
      msgs::SegmentSurfacesResultConstPtr result =
          clients_->surface_segmentation_client.getResult();
      if (result) {
        if (result->landmarks.size() == 0) {
          *error = errors::kNoLandmarksDetected;
        }
        world_->surface_box_landmarks.clear();

        // Process the landmark and estimate the max height of the scene
        double max_height = 0.0; // Assume everything is above the ground
        for (size_t i = 0; i < result->landmarks.size(); ++i) {
          msgs::Landmark landmark;
          ProcessSurfaceBox(result->landmarks[i], &landmark);
          double height = landmark.pose_stamped.pose.position.z + landmark.surface_box_dims.z / 2;
          if (height > max_height) {
            max_height = height;
          }
          world_->surface_box_landmarks.push_back(landmark);
        }


        // Clean up the existing collision surfaces in the world
        for (size_t i = 0; i < world_->surface_ids.size(); i++) {
          moveit_msgs::CollisionObject surface;
          surface.id = world_->surface_ids[i];
          surface.operation = moveit_msgs::CollisionObject::REMOVE;
          motion_planning_->PublishCollisionObject(surface);
        }
        ROS_INFO("Removed %ld collision surfaces", world_->surface_ids.size());
        world_->surface_ids.clear();

        // For each detected surface, mark the surface as a collision object
        // for the subsequent motion planning
        std::vector<msgs::Surface> surfaces = result->surfaces;
        for (size_t i = 0; i < surfaces.size(); ++i) {
          shape_msgs::SolidPrimitive surface_shape;
          surface_shape.type = shape_msgs::SolidPrimitive::BOX;
          surface_shape.dimensions.resize(3);
          surface_shape.dimensions[0] = surfaces[i].dimensions.x;
          surface_shape.dimensions[1] = surfaces[i].dimensions.y;
          surface_shape.dimensions[2] = surfaces[i].dimensions.z;

          moveit_msgs::CollisionObject surface_obj;
          surface_obj.header.frame_id =
              surfaces[i].pose_stamped.header.frame_id;

          // Give each collision surface an id
          std::stringstream ss;
          ss << kCollisionSurfaceName;
          ss << i;
          std::string obj_id = ss.str();
          // Store the collision surface id in world instance for later cleanup
          world_->surface_ids.push_back(obj_id);

          surface_obj.id = obj_id;
          surface_obj.primitives.push_back(surface_shape);
          surface_obj.primitive_poses.push_back(surfaces[i].pose_stamped.pose);
          surface_obj.operation = moveit_msgs::CollisionObject::ADD;
          motion_planning_->PublishCollisionObject(surface_obj);
        }

        // Add shelf wall as collision surfaces
        if (surfaces.size() > 1) {
          moveit_msgs::CollisionObject right_wall = GetRightShelfWall(max_height, surfaces);
          world_->surface_ids.push_back(right_wall.id);
          motion_planning_->PublishCollisionObject(right_wall);
          moveit_msgs::CollisionObject left_wall = GetLeftShelfWall(max_height, surfaces);
          world_->surface_ids.push_back(left_wall.id);
          motion_planning_->PublishCollisionObject(left_wall);
        }
        runtime_viz_.PublishSurfaceBoxes(world_->surface_box_landmarks);
        ROS_INFO("Added %ld collision surfaces", world_->surface_ids.size());
      } else {
        ROS_ERROR("Surface segmentation result pointer was null!");
        *error = "Surface segmentation result pointer was null!";
        return false;
      }
    }
    return done;
  }
  return true;
}

void ActionExecutor::Cancel() {
  if (action_.type == Action::ACTUATE_GRIPPER) {
    if (action_.actuator_group == Action::GRIPPER) {
      clients_->gripper_client.cancelAllGoals();
    } else if (action_.actuator_group == Action::LEFT_GRIPPER) {
      clients_->l_gripper_client.cancelAllGoals();
    } else if (action_.actuator_group == Action::RIGHT_GRIPPER) {
      clients_->r_gripper_client.cancelAllGoals();
    }
  } else if (action_.type == Action::MOVE_TO_JOINT_GOAL) {
    if (action_.actuator_group == Action::HEAD) {
      clients_->head_client.cancelAllGoals();
    } else {
      // Arm motions are cancelled by motion planning in the step executor.
    }
  } else if (action_.type == Action::DETECT_SURFACE_OBJECTS) {
    clients_->surface_segmentation_client.cancelAllGoals();
  }
}

void ActionExecutor::ActuateGripper() {
  control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command = action_.gripper_command;

  SimpleActionClient<control_msgs::GripperCommandAction>* client;
  if (action_.actuator_group == Action::GRIPPER) {
    client = &clients_->gripper_client;
  } else if (action_.actuator_group == Action::LEFT_GRIPPER) {
    client = &clients_->l_gripper_client;
  } else if (action_.actuator_group == Action::RIGHT_GRIPPER) {
    client = &clients_->r_gripper_client;
  } else {
    return;
  }
  client->sendGoal(gripper_goal);
}

void ActionExecutor::DetectSurfaceObjects() {
  rapid_pbd_msgs::SegmentSurfacesGoal goal;
  goal.save_cloud = false;
  clients_->surface_segmentation_client.sendGoal(goal);
}

void ActionExecutor::PublishInvalidGroupError(const Action& action) {
  ROS_ERROR("Invalid actuator_group \"%s\" for action type \"%s\".",
            action.actuator_group.c_str(), action.type.c_str());
}
}  // namespace pbd
}  // namespace rapid
