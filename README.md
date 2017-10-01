# Robotnik AGVS & Neuronics Katana Arm Gazebo Simulation

This stack contains essential parts of both of this robots to perform a Gazebo simulation.

To birng up simulation:
* roslaunch agvs_gazebo agvs_w_ns.launch  This will launch Gazebo and spawn a AGVS.
* roslaunch agvs_gazebo 1st_katana_w_ns.launch  This will spawn a Katana Arm with its controllers.
* Optionally you can also start 2nd_katana_w_ns.launch but it will cause a performance loss.
* roslaunch agvs_robot_control agvs_robot_control.launch  This will load controllers of AGVS. This is needed for keyboard control.
* roslaunch agvs_keyboard_controller controller.launch  AGVS can be controlled by agvs_keyboard_controller. Please follow instructions.
* roslaunch katana_tutorials follow_joint_trajectory_client.launch  This let us send messages arms to perform pick and/or place operations. Will be used for autonomous demo.

AGVS gmapping example:
* roslaunch agvs_gazebo agvs_office.launch
* roslaunch agvs_robot_control agvs_robot_control.launch
* roslaunch agvs_gazebo agvs_gmapping.launch


This package uses Robotnik AGVS (http://wiki.ros.org/Robots/Agvs) and Neuronics Katana Arm (http://wiki.ros.org/katana_driver). 

May 2017, Eskişehir Osmangazi Univesity Formal Languages ​​and Automata Mini Project

Team members:
 * [Azmi Cibi](https://azmicibi.gitlab.io/cv/) 
 * [Erkin Semiz](https://github.com/ErkinSemiz)
 * [Tahir Özdemir](http://cv.tahirozdemir.com)
 * [Mahmut Bilgehan Serbest](https://github.com/Bilgehan-Serbest)
