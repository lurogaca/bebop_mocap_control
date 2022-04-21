# bebop_mocap_control
automatic control of bebop - optitrack MCS


**Starting your terminal**
  
On your computer home screen hit these buttons at the same time. This will open a new terminal for you. 

     ctrl alt t
    
Once you have your terminal open, you must source your workspace and let it know you will be running ROS.  


    source /opt/ros/noetic/setup.bash
  
    roscore
    
    
**Inputting the PID into your work space**

_In a new terminal:_
    
    source /opt/ros/noetic/setup.bash
  
    cd ~/catkin_ws_bebop/src
  
    cd bebop_autonomy
  
    cd bebop_tools
  
    cd launch
   
    touch pid_tuning.py
  
    gedit pid_tuning.py
  
When you are in the text editor for pid_tuning.py paste the following code into the editor and save.

    #!/usr/bin/env python

    import rospy

    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Empty
    from sensor_msgs.msg import Joy

    import csv
    import time
    import tf
    import threading as th



    vicon_x = 0
    vicon_y = 0
    vicon_z = 0

    vicon_ang_x = 0
    vicon_ang_y = 0
    vicon_ang_z = 0
    vicon_ang_w = 0
    yaw_degrees = 0
    last_position = 0
    current_time = 0.0
    last_time = 0.0
    last_position_y = 0


    def heading_comp(actual, waypoint):
        difference = waypoint - actual
        while difference < -180:
            difference += 360
        while difference > 180:
            difference -= 360


    def odom_callback(msg):
        global last_position_y, last_time, current_time, last_position, vicon_x, vicon_y, vicon_z, vicon_ang_x, vicon_ang_y, vicon_ang_z, vicon_ang_w, yaw_degrees
        last_error_x = 0
        last_error_y = 0
        pub_bebop_vel = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        twist_bebop = Twist()


        vicon_x = msg.pose.pose.position.y
        vicon_y = msg.pose.pose.position.x
        #vicon_z = msg.pose.pose.position.z


        print("position x ==> " + str(vicon_x))
        print("position y ==> " + str(vicon_y))

        vicon_ang_x = msg.pose.pose.orientation.x
        vicon_ang_y = msg.pose.pose.orientation.y
        vicon_ang_z = msg.pose.pose.orientation.z
        vicon_ang_w = msg.pose.pose.orientation.w


        quaternion = [vicon_ang_x, vicon_ang_y, vicon_ang_z, vicon_ang_w]

        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]


        yaw_degrees = yaw * (180 / 3.1415926535) - 90.0

        heading_waypoint = -10


        linear_kp_x = 0.15
        linear_kp_y = 0.15

        kd = 0.45


        linear_x_desired = 0
        linear_y_desired = 0


        linear_error_x = (linear_x_desired - vicon_x)
        linear_error_y = (linear_y_desired - vicon_y)

        x_dot = vicon_x - last_position
        y_dot = vicon_y - last_position_y
        print("x_dot ==> " + str(x_dot))
        print("y_dot ==> " + str(y_dot))

        heading_u = heading_comp(yaw_degrees, heading_waypoint)


        ns = msg.header.stamp.nsecs / 1000000000
        current_time = msg.header.stamp.secs + ns

        print("current time ==> " + str(current_time))

        delta_time = current_time - last_time
        print("delta_time ==> " + str(delta_time))

        if(delta_time <= 0.0):
            linear_u_x = linear_kp_x * linear_error_x
            linear_u_y = linear_kp_y * linear_error_y
        else:
            linear_u_x = linear_kp_x * linear_error_x + kd * (x_dot/delta_time)
            linear_u_y = linear_kp_y * linear_error_y + kd * (y_dot/delta_time)

        twist_bebop.linear.x = linear_u_x
        twist_bebop.linear.y = -linear_u_y
        pub_bebop_vel.publish(twist_bebop)

        print("velocity x ==> " + str(linear_u_x))
        print("velocity y ==> " + str(linear_u_y))
        last_error_x = linear_error_x
        last_error_y = linear_error_y
        last_position = vicon_x
        last_position_y = vicon_y
        last_time = current_time
        print("==========================================")

    def landing():
        pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        print("Landing")
        pub_land.publish(Empty())

    def main():
        # Initialize node
        rospy.init_node('logger', anonymous=True)


        # Subscribing to the mocap_node and the Odometry topic
        rospy.Subscriber('/mocap_node/bebop_1/Odom', Odometry, odom_callback)
        pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 10)

        try:
            rospy.spin()
         # Ctrl + c i believe to land
        except KeyboardInterrupt:
            print("Landing")
            pub_land.publish(Empty())
    if __name__ == '__main__':
        starting = input("Press s to start: ")
        if starting == "s":
            main()
            landing()



**Launching the Mocap**

_In a new terminal_

    source /opt/ros/noetic/setup.bash
    
    cd /opt/ros/noetic/share/mocap_optitrack
    
    cd config
    
    sudo gedit mocap.yaml

Once you are in the text editor for the mocap.yaml make sure that the name of your "robot_1" or "marker_1" is changed to "bebop_1" as shown below:


    
![mocap_yaml](https://user-images.githubusercontent.com/103153240/164558718-527cbfd9-3588-4783-ad9b-ff044f507d6d.png)

Once you have changed the names to "bebop_1" go back to your terminal and hit these buttons:

    CTRL C 
    
You should still be in the mocap_optitrack config folder

    cd ..
    
    cd launch
    
    roslaunch mocap.launch
    
 **Launching the bebop_node**
 
 _In a new terminal:_
 
    source /opt/ros/noetic/setup.bash
    
    source /home/labuser/catkin_ws_bebop/devel/setup.bash
    
    cd ~/catkin_ws_bebop/src
    
    cd bebop autonomy
    
    cd bebop_driver
    
    cd launch
    
    roslaunch bebop_node.launch
    
 **launching joy_teleop**
 
_In a new terminal:_
    
    source /opt/ros/noetic/setup.bash
    
    source /home/labuser/catkin_ws_bebop/devel/setup.bash
    
    cd ~/catkin_ws_bebop/src
    
    cd bebop autonomy
    
    cd bebop_tools
    
    cd launch
    
    roslaunch joy_teleop.launch
    
**Running the PID**

_In a new terminal:_

    source /opt/ros/noetic/setup.bash
    
    source /home/labuser/catkin_ws_bebop/devel/setup.bash
    
    cd ~/catkin_ws_bebop/src
    
    cd bebop autonomy
    
    cd bebop_tools
    
    cd launch
    
    rosrun bebop_tools pid_tuning.py
    
 Once you have hit enter on the last code you will see a prompt ask you to hit "s" to start. You need to first get the drone into the air and hovering before you can start the automation.
 To get the bebop drone in the air hit these buttons on the Xbox controller at the same time:
 
    RB Y
    
RB standing for Right Bumper.

You may now press "s" to start the PID System. Once the drone has centered itself on the origin you will hit the following buttons on the Xbox controller to land the drone.

    RB A
    
    
    
