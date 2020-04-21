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

void MoveBackRecovery::initialize(
    std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* /*global_costmap*/, costmap_2d::Costmap2DROS* local_costmap)
{
  tf_ = tf;
  local_costmap_ = local_costmap;

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::NodeHandle private_nh("~/" + name);

  double controller_frequency;
  private_nh.param("controller_frequency", controller_frequency, 20.0);
  controller_frequency_ = ros::Rate(controller_frequency);

  double linear_vel_back;
  private_nh.param("linear_vel_back", linear_vel_back, -0.3);
  linear_vel_back_.linear.x = linear_vel_back;

  private_nh.param("step_back_length", step_back_length_, 1.0);

  double step_back_timeout;
  private_nh.param("step_back_timeout", step_back_timeout, 15.0);
  step_back_timeout_ = ros::Duration(step_back_timeout);

  initialized_ = true;
}

uint32_t MoveBackRecovery::runBehavior(std::string &message)
{
  geometry_msgs::PoseStamped initial_pose;
  local_costmap_->getRobotPose(initial_pose);
  tf2::Vector3 initial_position;
  tf2::fromMsg(initial_pose.pose.position, initial_position);

  ros::Time time_begin = ros::Time::now();
  while (ros::ok())
  {

    // time out
    if(!step_back_timeout_.isZero() &&
        time_begin + step_back_timeout_ < ros::Time::now())
    {
      publishStop();
      message = "Time out, moving backwards";
      ROS_INFO_STREAM(message);
      ROS_INFO("%.2f [sec] elapsed.", step_back_timeout_.toSec());
      return mbf_msgs::RecoveryResult::PAT_EXCEEDED;
    }

    if(canceled_) {
      message = "Cancel has been requested, stopping robot.";
      ROS_INFO_STREAM(message);
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
    tf2::Vector3 vec = quaternion.getAxis();


    if(dist < 0.01)
    {
      message = "Successfully moved backwards.";
      ROS_INFO_STREAM(message);
      return mbf_msgs::RecoveryResult::SUCCESS;
    }

    cmd_vel_pub_.publish(linear_vel_back_);
    controller_frequency_.sleep();
  }
  publishStop();
  return mbf_msgs::RecoveryResult::STOPPED;
}

MoveBackRecovery::CostmapState MoveBackRecovery::checkPoseCost(
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
  std::vector<geometry_msgs::Point> footprint = local_costmap_->getUnpaddedRobotFootprint();
  costmap_2d::padFootprint(footprint, safety_dist);

  // use a footprint helper instance to get all the cells totally or partially within footprint polygon
  base_local_planner::FootprintHelper fph;
  std::vector<base_local_planner::Position2DInt> footprint_cells =
      fph.getFootprintCells(Eigen::Vector3f(x, y, yaw), footprint, *local_costmap_->getCostmap(), true);


  CostmapState state = CostmapState::FREE;
  if (footprint_cells.empty())
  {
    // no cells within footprint polygon must mean that robot is completely outside of the map
    state = std::max(state, CostmapState::OUTSIDE);
  }
  else
  {
    // lock costmap so content doesn't change while adding cell costs
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(local_costmap_->getCostmap()->getMutex()));

    // integrate the cost of all cells; state value precedence is UNKNOWN > LETHAL > INSCRIBED > FREE
    for (int i = 0; i < footprint_cells.size(); ++i)
    {
      unsigned char cost = local_costmap_->getCostmap()->getCost(footprint_cells[i].x, footprint_cells[i].y);
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
