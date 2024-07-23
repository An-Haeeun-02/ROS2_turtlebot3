# ROS2_turtlebot3

<pre><code>colcon build --symlink-install --packages-select robot_action </code></pre>
<pre><code> colcon build --symlink-install --packages-select robot_go_pkg</code></pre>

**#서버 먼저, 클라이언트 나중에**
<pre><code> ros2 run robot_go_pkg server_node </code></pre>
<pre><code> ros2 run robot_go_pkg cli_node </code></pre>
