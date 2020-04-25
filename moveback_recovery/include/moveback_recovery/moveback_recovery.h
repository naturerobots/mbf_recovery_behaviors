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

  virtual ~MoveBackRecovery();

  /// Initialize the parameters of the behavior
  virtual void initialize (std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* global_costmap,
                           costmap_2d::Costmap2DROS* local_costmap);

  /// Run the behavior
  virtual uint32_t runBehavior(std::string& message);

  virtual bool cancel();

  enum class CostmapState{
    FREE      =  0, // robot is completely in traversable space
    INSCRIBED =  1, // robot is partially in inscribed space
    LETHAL    =  2, // robot is partially in collision
    UNKNOWN   =  3, // robot is partially in unknown space
    OUTSIDE   =  4  // robot is completely outside the map
  };

  MoveBackRecovery::CostmapState checkPoseCost(
      costmap_2d::Costmap2DROS* costmap_ptr,
      const geometry_msgs::PoseStamped& pose,
      const float safety_dist,
      const float lethal_cost_mult,
      const float inscrib_cost_mult,
      const float unknown_cost_mult,
      int &total_cost) const;


private:

  uint32_t publishStop() const;

  ros::NodeHandle nh_;
  costmap_2d::Costmap2DROS* local_costmap_;
  costmap_2d::Costmap2DROS* global_costmap_;
  tf2_ros::Buffer* tf_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher back_pos_pub_;
  bool initialized_;
  bool canceled_;

  float step_back_length_;
  float control_frequency_;
  float step_back_timeout_;
  float linear_vel_back_;

  float footprint_inflation_;
  float look_behind_dist_;

  bool publish_back_point_;

  float lethal_cost_mul_;
  float inscribe_cost_mul_;
  float unknown_cost_mul_;
};

} // namespace moveback_recovery

#endif // MOVEBACK_RECOVERY_H_