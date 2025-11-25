# Homework3    
### PX4-Autopilot setup
The code was developed based on the 1.16.0 version of PX4-Autopilot. This package should be clone from the git repo    
`git clone https://github.com/PX4/PX4-Autopilot.git --recursive`    
`cd PX4-Autopilot`    
`git checkout v1.16.0`    
`make submodulesclean`     

### Our code setup    

The uploaded files should replace the corresponding ones in the following directories:      

`custom_drone` and `custom_drone_base` should be added in `PX4-Autopilot/Tools/simulation/gz/models/`        

`airframes` should replace the corresponding folder in `PX4-Autopilot/ROMFS/px4fmu_common/init.dz-posix/`    

`dds_topics.yaml` should replace the corresponding file in `PX4-Autopilot/src/modules/uxrce_dds_client/`    

`offboard_rl`, `force_land` should replace the corresponding folders in the src folder     

`bag_files` folder should be added in the src folder    

`DDS_run.sh` file should be added in the src folder         

`px4_msgs`, `read_rpy` folders should be added in the src folder



### Build    
In the ros2_ws directory run:   
`colcon build && . install/setup.bash`

## How to run simulations   

- Start the QGroundControl application
- 1st terminal: from the folder `/home/user/ros2_ws/src/PX4-Autopilot` run
     `make px4_sitl gz_custom_drone`
     This runs the gazebo simulation for the custom drone
- 2nd terminal: from the folder `/home/user/ros2_ws/src/` run
   `./DDS_run.sh`
  This activates the uxrce_dds bridge
- 3rd terminal: from the folder `/home/user/ros2_ws/src/bag_files` run
   `./run_bag.sh`
  This runs the ros2 bag tool
- 4th terminal: will be used to launch the force_land node in the force_land folder or
   the go_to_point node in the offboard_rl folder
  respectively:
  - `ros2 run force_land force_land`
  - `ros2 run offboard_rl go_to_point` (the node asks to press enter to compute and send the trajectory which is actually hard coded in the function for simplicity)

Up to the third terminal the steps always repeat themselves: the bag tool sneaks on the topics needed for all the plots; only the way to control the robot changes: 
- for the first point the QGroundInterface command for position is used to place the robot at the right altitude
- for the second point the force_land node is to be activated and then either the position control (for altitude) or the joystick controller itself can be used on QGroundControl (however it might give some problems in testing since the commands sent, being continuous, are likely to disable the forced landing almost as soon as it is activated, too quickly to be seen)
- for the third point the go_to_point is used to compute the desired trajectory which is actually hard coded for simplicity (before running the node the drone should be armed)

After all the simulations are stopped, in the `/home/user/ros2_ws/src/bag_files` one can run `python3 plot_data.py` to visualize (in different windows) all the requested plots.
