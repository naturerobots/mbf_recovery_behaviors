/*
 *  Copyright 2020, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  abstract_planner.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveback_recovery/moveback_recovery.h>
#include <mbf_msgs/RecoveryResult.h>
#include <base_local_planner/footprint_helper.h>
#include <tf2/LinearMath/Quaternion.h>

// register as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(moveback_recovery::MoveBackRecovery, mbf_costmap_core::CostmapRecovery)

namespace moveback_recovery
{

MoveBackRecovery::MoveBackRecovery(){}

MoveBackRecovery::~MoveBackRecovery(){}

void MoveBackRecovery::initialize(
    std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
  tf_ = tf;
  local_costmap_ = local_costmap;

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  if(publish_back_point_)
  {
    back_pos_pub_ = nh_.advertise<geometry_msgs::PointStamped>("back_point", 1);
  }

  ros::NodeHandle private_nh("~/" + name);

  private_nh.param("control_frequency", control_frequency_, 20.0f);
  private_nh.param("linear_vel_back", linear_vel_back_, -0.3f);
  private_nh.param("step_back_length", step_back_length_, 1.0f);
  private_nh.param("step_back_timeout", step_back_timeout_, 15.0f);

  private_nh.param("footprint_inflation", footprint_inflation_, 0.0f);
  private_nh.param("look_behind_dist", look_behind_dist_, 0.1f);

  private_nh.param("publish_back_point", publish_back_point_, false);

  lethal_cost_mul_ = 0;
  inscribe_cost_mul_ = 0;
  unknown_cost_mul_ = 0;

  initialized_ = true;
}

uint32_t MoveBackRecovery::runBehavior(std::string &message)
{
  canceled_ = false;
  geometry_msgs::PoseStamped initial_pose;
  local_costmap_->getRobotPose(initial_pose);
  tf2::Vector3 initial_position;
  tf2::fromMsg(initial_pose.pose.position, initial_position);

  ros::Rate rate(control_frequency_);

  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = linear_vel_back_;

  ros::Duration timeout(step_back_timeout_);
  ros::Time time_begin = ros::Time::now();

  while (ros::ok())
  {

    // check for timeout
    if(!timeout.isZero() && time_begin + timeout < ros::Time::now())
    {
      publishStop();
      message = "Time out, moving backwards";
      ROS_INFO_STREAM(message);
      ROS_INFO("%.2f [sec] elapsed.", timeout.toSec());
      return mbf_msgs::RecoveryResult::PAT_EXCEEDED;
    }

    // check for cancel request
    if(canceled_) {
      message = "Cancel has been requested, stopping robot.";
      ROS_INFO_STREAM(message);
      // stop the robot
      publishStop();
      return mbf_msgs::RecoveryResult::CANCELED;
    }

    geometry_msgs::PoseStamped robot_pose;
    local_costmap_->getRobotPose(robot_pose);
    tf2::Vector3 robot_position;
    tf2::fromMsg(robot_pose.pose.position, robot_position);
    double dist = (robot_position - initial_position).length();

    tf2::Quaternion quaternion;
    tf2::fromMsg(robot_pose.pose.orientation, quaternion);
    tf2::Matrix3x3 mat(quaternion);
    tf2::Vector3 back_direction = mat * tf2::Vector3(-look_behind_dist_, 0, 0);

    // check for possible collision
    tf2::Stamped<tf2::Vector3> back_point(
        robot_position + back_direction,
        ros::Time::now(),
        local_costmap_->getGlobalFrameID());

    tf2::Transform transform(quaternion, back_point);
    tf2::Stamped<tf2::Transform> stamped_transform(
        transform,
        ros::Time::now(),
        local_costmap_->getGlobalFrameID());

    geometry_msgs::PoseStamped look_behind_pose;
    tf2::toMsg(stamped_transform, look_behind_pose);

    int cost = 0;
    CostmapState cm_state = checkPoseCost(
        local_costmap_, look_behind_pose,
        footprint_inflation_, lethal_cost_mul_,
        inscribe_cost_mul_, unknown_cost_mul_,
        cost);

    if(cm_state == CostmapState::LETHAL)
    {
      message = "Stop moving backwards, since an obstacle behind the robot was detected";
      ROS_INFO_STREAM(message);
      // stop the robot
      publishStop();
      return mbf_msgs::RecoveryResult::FAILURE;
    }

    // if enabled, publish the back point where the footprint collision check is done
    if(publish_back_point_) {
      geometry_msgs::PointStamped back_point_msg;
      tf2::toMsg(back_point, back_point_msg);
      back_pos_pub_.publish(back_point_msg);
    }

    // check if the robot moved back the specified distance
    if(step_back_length_ < dist)
    {
      message = "Successfully moved backwards.";
      ROS_INFO_STREAM(message);
      // stop the robot
      publishStop();
      return mbf_msgs::RecoveryResult::SUCCESS;
    }

    cmd_vel_pub_.publish(vel_msg);
    rate.sleep();
  }
  // stop the robot
  publishStop();
  return mbf_msgs::RecoveryResult::STOPPED;
}

MoveBackRecovery::CostmapState MoveBackRecovery::checkPoseCost(
  costmap_2d::Costmap2DROS* costmap_ptr,
  const geometry_msgs::PoseStamped& pose,
  const float safety_dist,
  const float lethal_cost_mult,
  const float inscrib_cost_mult,
  const float unknown_cost_mult,
  int &total_cost) const
{
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);

  // pad raw footprint to the requested safety distance; note that we discard footprint_padding parameter effect
  std::vector<geometry_msgs::Point> footprint = costmap_ptr->getUnpaddedRobotFootprint();
  costmap_2d::padFootprint(footprint, safety_dist);

  // use a footprint helper instance to get all the cells totally or partially within footprint polygon
  base_local_planner::FootprintHelper fph;
  std::vector<base_local_planner::Position2DInt> footprint_cells =
      fph.getFootprintCells(Eigen::Vector3f(x, y, yaw), footprint, *costmap_ptr->getCostmap(), true);


  CostmapState state = CostmapState::FREE;
  if (footprint_cells.empty())
  {
    // no cells within footprint polygon must mean that robot is completely outside of the map
    state = std::max(state, CostmapState::OUTSIDE);
  }
  else
  {
    // lock costmap so content doesn't change while adding cell costs
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr->getCostmap()->getMutex()));

    // integrate the cost of all cells; state value precedence is UNKNOWN > LETHAL > INSCRIBED > FREE
    for (int i = 0; i < footprint_cells.size(); ++i)
    {
      unsigned char cost = costmap_ptr->getCostmap()->getCost(footprint_cells[i].x, footprint_cells[i].y);
      switch (cost)
      {
      case costmap_2d::NO_INFORMATION:
        state = std::max(state, CostmapState::UNKNOWN);
        total_cost += cost * (unknown_cost_mult ? unknown_cost_mult : 1.0);
        break;
      case costmap_2d::LETHAL_OBSTACLE:
        state = std::max(state, CostmapState::LETHAL);
        total_cost += cost * (lethal_cost_mult ? lethal_cost_mult : 1.0);
        break;
      case costmap_2d::INSCRIBED_INFLATED_OBSTACLE:
        state = std::max(state, CostmapState::INSCRIBED);
        total_cost += cost * (inscrib_cost_mult ? inscrib_cost_mult : 1.0);
        break;
      default:
        total_cost += cost;
        break;
      }
    }
  }

  // Provide some details of the outcome
  switch (state)
  {
  case CostmapState::OUTSIDE:
    ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is outside the map (cost = " << total_cost
                              << "; safety distance = " << safety_dist << ")");
    break;
  case CostmapState::UNKNOWN:
    ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is in unknown space! (cost = " << total_cost
                              << "; safety distance = " << safety_dist << ")");
    break;
  case CostmapState::LETHAL:
    ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is in collision! (cost = " << total_cost
                              << "; safety distance = " << safety_dist << ")");
    break;
  case CostmapState::INSCRIBED:
    ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is near an obstacle (cost = " << total_cost
                              << "; safety distance = " << safety_dist << ")");
    break;
  case CostmapState::FREE:
    ROS_DEBUG_STREAM("Pose [" << x << ", " << y << ", " << yaw << "] is free (cost = " << total_cost
                              << "; safety distance = " << safety_dist << ")");
    break;
  }
  return state;
}

uint32_t MoveBackRecovery::publishStop() const
{
  geometry_msgs::Twist zero_vel;
  zero_vel.linear.x = zero_vel.linear.y = zero_vel.linear.z = 0;
  zero_vel.angular.x = zero_vel.angular.y = zero_vel.angular.z = 0;
  cmd_vel_pub_.publish(zero_vel);
}

bool MoveBackRecovery::cancel() {
  canceled_ = true;
  return true;
}

} // namespace moveback_recovery
