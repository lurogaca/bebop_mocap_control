# bebop_mocap_control
automatic control of bebop - optitrack MCS

# PID Controller 
* In this program, the drone will start at any location and navigate toward the origin (0,0) 
* In order to run the Autonomous Program, two networks are required in order to connect to the motion capture system and drone, plug in USB wifi antena ( we used LYNEC 150 Mbps)

-> Connect to the Optitrack network  

-> Connect to the drone

**Open a new terminal : Run ROS**

```
$ roscore
```
**Open a new terminal : Run Mocap** 

```
$ source /opt/ros/noetic/setup.bash
$ roslaunch mocap_optitrack mocap.launch
```
**Open a new terminal : Connect to Drone**

```
$ source /home/labuser/bebop_ws/devel/setup.bash
$ roslaunch bebop_node.launch
```
**Open a new terminal : Launch controller**

```
$ source /home/labuser/bebop_ws/devel/setup.bash
$ roslaunch bebop_tools joy_teleop.launch
```
**Open a new terminal : Run PID Controller Program**

```
$source /home/labuser/bebop_ws/devel/setup.bash
$rosrun bebop_tools pid_tuning.py
```
-> take off drone 

-> Initialize program by pressing 's' + enter 

-> land drone
