# Vitulus base
Wheel ROS controller for VITULUS.

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
 * /cmd_vel [unknown type]


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

 
