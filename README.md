# In1House_Competition1Day1_version1
 this reposatry is the tasks evaluation of test project inhouse competition -Day 1

 must install the reposatry on home/pi folder
then enter the workspace 
$cd catkin_ws

build the work space 
$ sudo su
$ catkin build -cs

 Task 1 is build the components into a chassis// and the instructor will evaluate it

 task 2 programming the sensor, to get the output of the sensor, 
 write this command on Tirmminal 1
 $ sudo su
 $ rosocre

 write this command on Tirmminal 2
$ rosrun vmxpi_ros_bringup Task2_sensors_node 

-----------------------------------------------------------
task 3 control the DC motor using the releop_twit package, to evaluate the ouput

to control the directrion of the motor prss i and m
to control the speed press q and z

 write this command on Tirmminal 1
 $ sudo su
 $ rosocre

 write this command on Tirmminal 2
$ rosrun vmxpi_ros_bringup Task3_teleop_node 

write this on Tirmminal 3 
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

-----------------------------------------------------------
task 4 control the Servo motor by using the NavX sensor, to evaluate the output 

moving the angle of the robot will result on moving the angle of the servo motor 

 write this command on Tirmminal 1
 $ sudo su
 $ rosocre

 write this command on Tirmminal 2
$ rosrun vmxpi_ros_bringup Task4_servo_node 



