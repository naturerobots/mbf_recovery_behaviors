# Moveback recovery
A simple recovery behavior that moves the robot back. It implements the move base flex recovery interface.

## Parameters
- **`control_frequency`** The cycle frequency executing the break checks, (default: 20 Hz)
- **`linear_vel_back`** The velocity for driving the robot backwards, (default: -0.3 m/sec)
- **`step_back_length`**  The distance to move the robot backwards, (default: 1 m)
- **`step_back_timeout`** The timeout before stopping the robot, (default: 15 sec)
- **`footprint_inflation`** The footprint inflation which is used when check the costmap for obstacles, (default: 0.0 m)
- **`look_behind_dist`** The robot pose checking distance to use for the costmap footprint checks, (default: 0.1 m)
- **`publish_back_point`** Publishes a point at the distance to the robot footprint position as point, (default: false)
                                                                                                              - **`control_frequency`** The cycle frequency executing the break checks, (default: 20 Hz)
                                                                                                              - **`linear_vel_back`** The velocity for driving the robot backwards, (default: -0.3 m/sec)
                                                                                                              - **`step_back_length`**  The distance to move the robot backwards, (default: 1 m)
                                                                                                              - **`step_back_timeout`** The timeout before stopping the robot, (default: 15 sec)
                                                                                                              - **`footprint_inflation`** The footprint inflation which is used when check the costmap for obstacles, (default: 0.0 m)
                                                                                                              - **`look_behind_dist`** The robot pose checking distance to use for the costmap footprint checks, (default: 0.1 m)
                                                                                                              - **`publish_back_point`** Publishes a point at the distance to the robot footprint position as point, (default: false)

