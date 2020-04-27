# Move Base Flex Recovery Behaviors

This repository contains recovery behaviors for [Move Base Flex (MBF)](https://github.com/magazino/move_base_flex).

## MoveBack Recovery Behavior
The `moveback_recovery` behavior moves the robot backwards until:
 - the configured distance is reached
 - the timeout duration is elapsed
 - an obstacle in the costmap is blocking the robot
 - it is canceled by the executive node by canceling the MBF recovery action

### Installation

`sudo apt install ros-melodic-moveback-recovery`

### Parameters
- **`control_frequency`** The cycle frequency executing the break checks, (default: 20 Hz)
- **`linear_vel_back`** The velocity for driving the robot backwards, (default: -0.3 m/sec)
- **`step_back_length`**  The distance to move the robot backwards, (default: 1 m)
- **`step_back_timeout`** The timeout before stopping the robot, (default: 15 sec)
- **`footprint_inflation`** The footprint inflation which is used when check the costmap for obstacles, (default: 0.0 m)
- **`look_behind_dist`** The robot pose checking distance to use for the costmap footprint checks, (default: 0.1 m)
- **`publish_back_point`** Publishes a point at the distance to the robot footprint position as point, (default: false)

### Plugin Loading
In the following we show a configuration example:

In an arbitrary file `my_pkg/config/recovery_behaviors.yaml` we configure a couple of recovery behaviors and give them an arbitrary name.
The type must be set to `moveback_recovery/MoveBackRecovery`.
```
recovery_behaviors:
  - name: 'moveback_recovery'
    type: 'moveback_recovery/MoveBackRecovery'
```
In the corresponding launch file, e.g., `my_pkg/launch/move_base_flex.launch` you now can load the parameter file into the namespace of the MBF node.
``` 
<node pkg="mbf_costmap_nav" type="mbf_costmap_nav" name="move_base_flex" output="screen">
  ...
  <rosparam file="$(find my_pkg)/config/recovery_behaviors.yaml" command="load" />
  ...
</node>
```
