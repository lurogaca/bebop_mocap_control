# bebop_mocap_control
automatic control of bebop - optitrack MCS

# PID Controller 

-> Connect to the Optitrack network  

-> Connect to the drone

Open a new terminal 

```
$ roscore
```
Open a new terminal 

```
$ source /opt/ros/noetic/setup.bash
$ roslaunch mocap_optitrack mocap.launch
```
Open a new terminal 

```
$ source /home/labuser/bebop_ws/devel/setup.bash
$ roslaunch bebop_node.launch
```
Open a new terminal 

```
$ source /home/labuser/bebop_ws/devel/setup.bash
$ roslaunch bebop_tools joy_teleop.launch
```
Open a new terminal 

```
$source /home/labuser/bebop_ws/devel/setup.bash
$rosrun bebop_tools pid_tuning.py
```
-> take off drone 

-> Initialize program by pressing 's' + enter 

-> land drone
