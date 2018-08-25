# Run  
roscore  
roslaunch turtlebot3_bringup turtlebot3_robot.launch  
roslaunch open_manipulator_moveit open_manipulator_demo.launch use_gazebo:=false  

# turtlebot3_with_pick_manipulator_core_hybrid
It use OpenCR Board. 
joint1 XL430-W250-T(turtle bot3 burter wheel motor)  -> opencr TTL pin  
joint2 MG946 -> opencr pwm 3 pin  
joint3 MG946 -> opencr pwm 5 pin  
joint4 MG946 -> opencr pwm 6 pin  
gripper MG946 -> opencr pwm 11 pin  

# turtlebot3_with_manipulator_setup_motor
you can set XL430-W250-T motor as manipulator joint1 setting by OpenCR board . 
It change motor ID, Operating Mode etc 

# turtlebot3_with_pick_manipulator_core
It use OpenCR Board.
joint1 MG946 -> opencr pwm 3 pin  
joint2 MG946 -> opencr pwm 5 pin  
joint3 MG946 -> opencr pwm 6 pin  
joint4 MG946 -> opencr pwm 9 pin  
gripper MG946 -> opencr pwm 11 pin  

# init_servo_motor Folder
It use OpenCR Board.
MG946 motor test code

<img src="http://emanual.robotis.com/assets/images/parts/controller/opencr10/exam_pwm_01.png" width="70%" height="70%">  

Click image to link to YouTube video.  
[![Video Label](http://img.youtube.com/vi/KAPcRl6xvGQ/0.jpg)](https://youtu.be/KAPcRl6xvGQ?t=0s) 


# Test  
//rosrun rosserial_python serial_node.py /dev/ttyUSB0  
//rosrun rosserial_python serial_node.py /dev/ttyACM0  

