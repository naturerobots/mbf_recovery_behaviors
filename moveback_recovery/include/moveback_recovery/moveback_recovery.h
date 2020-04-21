#ifndef MOVEBACK_RECOVERY_H_
#define MOVEBACK_RECOVERY_H_

#include <mbf_costmap_core/costmap_recovery.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>

namespace moveback_recovery
{

/// Recovery behavior that takes a given twist and tries to execute it for up to
/// d seconds, or until reaching an obstacle.  
class MoveBackRecovery : public mbf_costmap_core::CostmapRecovery
{
public:

  MoveBackRecovery();

  /// Initialize the parameters of the behavior
  virtual void initialize (std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* global_costmap,
                           costmap_2d::Costmap2DROS* local_costmap);

  /// Run the behavior
  virtual uint32_t runBehavior(std::string& message);

  virtual bool cancel();

  virtual ~MoveBackRecovery() { };

private:
  geometry_msgs::Pose2D getCurrentRobotPose() const;
  uint32_t moveBack() const;
  uint32_t publishStop() const;
  double getCurrentDiff(const geometry_msgs::Pose2D referencePose) const;

  ros::NodeHandle nh_;
  costmap_2d::Costmap2DROS* local_costmap_;
  tf2_ros::Buffer* tf_;
  ros::Publisher cmd_vel_pub_;
  bool initialized_;
  bool canceled_;

  double controller_frequency_;
  double linear_vel_back_;
  double step_back_length_;
  double step_back_timeout_;
};

} // namespace moveback_recovery

#endif // MOVEBACK_RECOVERY_H_