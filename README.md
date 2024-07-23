# ROS2_turtlebot3

colcon build --symlink-install --packages-select robot_action
colcon build --symlink-install --packages-select robot_go_pkg

#서버 먼저, 클라이언트 나중에
ros2 run robot_go_pkg server_node 
ros2 run robot_go_pkg cli_node 
