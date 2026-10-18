# Vitulus base
Wheel ROS controller for VITULUS. It is a driver for moteus controllers.

it is necessary to map the USB port where fdcanusb is connected to /dev/fdcanusb before you run this controller.


## Node [/cmd_vel_mux] — a person beats the software

The only publisher of the wheel command `/mobile_base_controller/cmd_vel`
(started by `launch/base_control.launch`, `respawn`; `cmd_vel_mux:=false` = the
old direct `/cmd_vel`). Inputs by priority:

| input | topic | priority |
|---|---|---|
| DS4 gamepad | `/ds4/cmd_vel` | 100 (human) |
| web joystick / keyboard | `/teleop/cmd_vel` | 90 (human) |
| all software (MBF, docking, mower recovery, agent supervisor) | `/cmd_vel` | 10 |

A human input counts only when deflected (dead zone `~deadzone_linear` 0.02 m/s,
`~deadzone_angular` 0.05 rad/s) or, for the DS4, on PS / L1 / R1
(`~takeover_buttons`, read from `/ds4/status`) — the zeros a switched-on DS4
sends with centred sticks never block the software. The first deflection is a
takeover: the software's goals are cancelled once (`/move_base_flex/{exe_path,
move_base,recovery}/cancel`, `/smach_goal/cancel`, `/dock_smach/stop`, and
`/mower_smach/stop` while a mowing program runs — skipped only when
`/mower_smach/busy` is false and `/mower_smach/active_program` empty, unknown
counts as running) and `/cmd_vel_mux/takeover` names the source. The human
keeps the wheels `~hold_s` (1.5 s) after the last deflection (released sticks =
zeros = stop); the software is accepted again only after it has been quiet for
`~rearm_quiet_s` (1 s), so nothing drives on by itself. Drive motors are never
switched off. Status: `/cmd_vel_mux/active` (latched `ds4|teleop|nav|nav_blocked`),
`/cmd_vel_mux/status` (JSON). Test: `test/test_cmd_vel_mux.py` (own roscore on
:11977, never the robot master).

## Node [/vitulus_base_node]


Publications: 
 * /base/front_left_wheel_vel [std_msgs/Float32]
 * /base/front_right_wheel_vel [std_msgs/Float32]
 * /base/rear_left_wheel_vel [std_msgs/Float32]
 * /base/rear_right_wheel_vel [std_msgs/Float32]
 * /joint_states [sensor_msgs/JointState]
 * /mobile_base_controller/cmd_vel_out [geometry_msgs/TwistStamped]
 * /mobile_base_controller/odom [nav_msgs/Odometry]
 * /mobile_base_controller/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /mobile_base_controller/parameter_updates [dynamic_reconfigure/Config]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf/tfMessage]

Subscriptions: 
 * /base/front_left_wheel_angle [std_msgs/Float32]
 * /base/front_left_wheel_real_vel [std_msgs/Float32]
 * /base/front_right_wheel_angle [std_msgs/Float32]
 * /base/front_right_wheel_real_vel [std_msgs/Float32]
 * /base/rear_left_wheel_angle [std_msgs/Float32]
 * /base/rear_left_wheel_real_vel [std_msgs/Float32]
 * /base/rear_right_wheel_angle [std_msgs/Float32]
 * /base/rear_right_wheel_real_vel [std_msgs/Float32]
 * /mobile_base_controller/cmd_vel [geometry_msgs/Twist] — only from cmd_vel_mux

Services: 
 * /controller_manager/list_controller_types
 * /controller_manager/list_controllers
 * /controller_manager/load_controller
 * /controller_manager/reload_controller_libraries
 * /controller_manager/switch_controller
 * /controller_manager/unload_controller
 * /mobile_base_controller/set_parameters
 * /start
 * /stop
 * /vitulus_base_node/get_loggers
 * /vitulus_base_node/set_logger_level


## Install

`cd ~/catkin_ws/src/vitulus`

`git clone https://github.com/lacina-dev/vitulus_base.git`

`cd ~/catkin_ws`

`catkin build`

## Run

`roslaunch vitulus_base base_control.launch run_description:=true`

## Info

 More details.
 [Motor assembly and controllers connection](https://docs.google.com/document/d/1Q6g1YeftRPKx67mn_4nCeVRBswe78MrCWejogsvNu1s/edit?usp=sharing)

 More about VITULUS? See my website.
 [https://lacina.dev](https://lacina.dev)

 Questions? Try Discord.
 [Discord channel](https://discord.gg/YqeNV5hEVN)

 
