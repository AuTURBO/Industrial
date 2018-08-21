# Run  
roscore  
roslaunch turtlebot3_bringup turtlebot3_robot.launch  
roslaunch open_manipulator_moveit open_manipulator_demo.launch use_gazebo:=false  

# turtlebot3_with_pick_manipulator_core_MG946 Folder
joint1 MG946 -> opencr pwm 3 pin  
joint2 MG946 -> opencr pwm 5 pin  
joint3 MG946 -> opencr pwm 6 pin  
joint4 MG946 -> opencr pwm 9 pin  
gripper MG946 -> opencr pwm 11 pin  

# init_servo_motor Folder
motor test code

<img src="http://emanual.robotis.com/assets/images/parts/controller/opencr10/exam_pwm_01.png" width="50%" height="50%">  


# Test  
//rosrun rosserial_python serial_node.py /dev/ttyUSB0  
//rosrun rosserial_python serial_node.py /dev/ttyACM0  

