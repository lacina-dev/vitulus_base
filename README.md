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
| drive_primitive (closed-loop direct drive) | `/drive_primitive/cmd_vel` | 50 |
| all software (MBF, docking, mower recovery, agent supervisor) | `/cmd_vel` | 10 |

A human input counts only when deflected (dead zone `~deadzone_linear` 0.02 m/s,
`~deadzone_angular` 0.05 rad/s) or, for the DS4, on PS / L1 / R1
(`~takeover_buttons`, read from `/ds4/status`) — the zeros a switched-on DS4
sends with centred sticks never block the software. The first deflection is a
takeover: the software's goals are cancelled once (`/move_base_flex/{exe_path,
move_base,recovery}/cancel`, `/smach_goal/cancel`, `/dock_smach/stop`, and
`/drive_primitive/stop`, `/mower_smach/stop` while a mowing program runs — skipped only when
`/mower_smach/busy` is false and `/mower_smach/active_program` empty, unknown
counts as running) and `/cmd_vel_mux/takeover` names the source. The human
keeps the wheels `~hold_s` (1.5 s) after the last deflection (released sticks =
zeros = stop); the software is accepted again only after it has been quiet for
`~rearm_quiet_s` (1 s), so nothing drives on by itself. Drive motors are never
switched off. Between software inputs the higher priority wins while it talks:
an input that published anything (zeros too) within `~software_hold_s` (0.5 s)
owns the wheels, lower ones are dropped (non-zero drops listed in
`nav_blocked`). Status: `/cmd_vel_mux/active` (latched
`ds4|teleop|primitive|nav|nav_blocked`),
`/cmd_vel_mux/status` (JSON). Test: `test/test_cmd_vel_mux.py` (own roscore on
:11977, never the robot master).

## Node [/drive_primitive] — closed-loop direct driving

One small motion at a time, closed on odometry (`~odom_topic` /odometry/odom,
fallback /mobile_base_controller/odom), on its own mux input
`/drive_primitive/cmd_vel` (above `/cmd_vel`, always below the humans). Works
without the agent (any node or `rostopic pub` may command it).

* `/drive_primitive/command` (std_msgs/String, JSON): `{"id", "kind":
  "straight"|"rotate"|"velocity", "distance_m" (±0.02..2 m), "speed" (m/s
  0.05..0.3 or rad/s 0.1..0.8), "angle_deg" (±1..360, + = left), "v", "w",
  "duration_s" (≤ 5 s), "keepalive_s" (0 = none), "require_supervisor",
  "cancel_nav", "max_duration_s"}`; an invalid or unsafe command is `REJECTED`
  and nothing moves; a new command supersedes the running one.
* `/drive_primitive/stop` (Bool True), `/drive_primitive/keepalive` (String id).
* `/drive_primitive/status` (String JSON, latched, on change + 2 Hz): `{"id",
  "state": IDLE|RUNNING|SUCCEEDED|ABORTED|REJECTED, "reason", "kind", "target",
  "done_m", "done_deg", "elapsed_s", "clearance_m", "t"}`.

Stops (ABORTED + reason) on: human takeover (`/cmd_vel_mux/active` ds4/teleop or
`/cmd_vel_mux/takeover`), stop, lost keepalive, test_supervisor not RUNNING or
stale (with `require_supervisor`), `/base/motor_power_state` false, odometry or
lidar (`~scan_topic` /rplidar/scan_filtered) older than 0.5 s, an obstacle in the
direction of travel (footprint-wide corridor + 5 cm out to `~stop_distance`
0.30 m + braking distance; for turning any point inside the circumscribed circle
+ `~rotate_clearance` 0.10 m), no progress for `~stuck_s` 3 s, timeout. Limits
`~max_linear` 0.3 m/s, `~max_angular` 0.8 rad/s, ramps `~acc_linear` 0.25 m/s²,
`~acc_angular` 0.8 rad/s², tolerance 2 cm / 2°. `drive_primitive:=false` in
`base_control.launch` leaves it out. Tests: `test/test_drive_primitive.py`
(offline, simulated odometry).

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

 
