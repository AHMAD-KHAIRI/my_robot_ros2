Terminal 1:
ros2 launch my_robot_bringup my_robot.launch.xml

(wait for controllers)

Terminal 2:
ros2 launch my_robot_moveit_config moveit_rviz.launch.py use_sim_time:=true
