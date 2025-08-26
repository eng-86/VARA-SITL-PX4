# VARA-SITL-PX4
This work addresses the challenge of maintaining reliable, scalable, and fully distributed formation control for MASs in the presence of sever actuator faults and agent loss. A novel Virtual Anchor Reconfiguration Algorithm (VARA) is proposed that enables real-time, distributed formation adaptation when an agent becomes uncontrollable.
It is based on PX4 multi vehicles GAZEBO classic IRIS simulation with ROS1. The ROS version is Melodic running on ubunto 18.04. The reposatory of PX4 has a version of 13.x. The main DHOSMO code is written in Python.

(https://www.youtube.com/watch?v=UrNyW3NZcBQ&t=70s "Demo video)

## PX4 Configuration:
1. Follows the main [PX4](https://docs.px4.io/main/en/sim_gazebo_classic/multi_vehicle_simulation.html) to manage your multi-vehicle toolchain.
2. Replace multi_uav_mavros_sitl.launch in your main PX4 reposatoty to be able to launch 6 IRIS quadcopters.
3. Build your PX4 code.

## ROS Configuration
1. Create a new ROS packge **vehicle_data**
2. Build your catkin_ws by following the official [ROS page](http://wiki.ros.org/catkin/Tutorials).
3. Add to your catkin_ws src the following:
   * main script file: VARA.py
   * include script file: include.py
   * ROS bag logger script file: log.py

## Code Running
1. Navigate to your PX4 repository folder in a new terminal and add the following:
   ```
   DONT_RUN=1 make px4_sitl_default gazebo
   source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
   roslaunch px4 multi_uav_mavros_sitl.launch
     ```
2. Navigate to your catkin workspace, and in four different terminals add the following:
   ```
   python VARA.py 0 # 0 for uav0, repeat for other uavs
   ```
3. You can run log.py script to log your results into a ROS Bag file.
4. You can run rqt viewer to view online ROS topics amnd messages.
