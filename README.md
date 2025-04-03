# Group17_final
No need to have extra information
1. launch the env by : ros2 launch final_project final_project.launch.py use_sim_time:=True
2. Lauch my node: ros2 launch group17_final my_robot_node.launch.py use_sim_time:=True 
3. if fail to run the node launch file: ros2 run group17_final my_robot_node --ros-args --params-file <path to my package>/group17_final/config/waypoint_params.yaml -p use_sim_time:=True 

