#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

#include "std_msgs/String.h"

#include <std_msgs/Int32.h>
#include <unistd.h>
#include <vector>
#include <robotnik_msgs/enable_disable.h>
#include <robotnik_msgs/set_digital_output.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#include "ackermann_msgs/AckermannDriveStamped.h"
#include <std_srvs/Empty.h>


#define DEFAULT_AXIS_LINEAR_X		1
#define DEFAULT_AXIS_ANGULAR		0	
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		1.0


#define MAX_LINEAR_SPEED	2.0 //m/s
#define MAX_ANGULAR_SPEED	2.0 // rads/s


#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class AGVSControl
{
public:
  AGVSControl();
  void keyLoop();
  void watchdog();
	void autonomousDemo();
private:

  std::string cube;
  ros::NodeHandle nh_,ph_;
  double linear_, angular_;
  ros::Time first_publish_;
  ros::Time last_publish_;
  double l_scale_, a_scale_;
  
	ros::Publisher vel_pub_;
ros::Publisher vel_pub_2;
  ros::Publisher arm_pub;
  ros::Publisher arm_pub2;
  void publish(double, double);
  boost::mutex publish_mutex_;

};

AGVSControl::AGVSControl():
  ph_("~"),
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);
	ph_.param("cube", cube, cube);

  vel_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/agvs_robot/command", 1);
	vel_pub_2 = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/agvs_robot_control/command", 1);
	arm_pub = nh_.advertise<std_msgs::String>("/katana1/chatter", 1);
  arm_pub2 = nh_.advertise<std_msgs::String>("/katana2/chatter", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "publisher");
  AGVSControl agvs_control;
  ros::NodeHandle n;

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&AGVSControl::keyLoop, &agvs_control));
  
  
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&AGVSControl::watchdog, &agvs_control));

  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;
      
  return(0);
}


void AGVSControl::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && 
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish(0, 0);
}

void AGVSControl::autonomousDemo(){
		puts("Demo is starting...");
		a_scale_ = 1;
		l_scale_ = 1;
 		ros::Time time;
		std_msgs::String msg;
		msg.data = "1";
	  puts("Demo started.");
		std::string spawnCube = "rosrun gazebo_ros spawn_model -file '"+cube+"' -sdf -x 5.5 -y 0.0 -z 1  -model cube";
		std::string spawnCube2 = "rosrun gazebo_ros spawn_model -file '"+cube+"' -sdf -x 8 -y 0.0 -z 0.11 -model cube2";
				
		while(1){
			system("rosservice call /gazebo/set_model_state '{model_state: { model_name: agvs, pose: { position: { x: 0.0, y: 0.0 ,z: 0.0 }, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'"); //fix robot position
      system(spawnCube2.c_str());
			time = ros::Time::now();
			
			while(ros::Time::now() - time < ros::Duration(5)){ publish(0, 1);}
	
			time = ros::Time::now();
			arm_pub2.publish(msg);
			
			while(ros::Time::now() - time < ros::Duration(16)) {	publish(0, 0); }

			//place a cube
			system("rosservice call gazebo/delete_model '{model_name: cube2}'");
			system(spawnCube.c_str());
			time = ros::Time::now();
			
			while(ros::Time::now() - time < ros::Duration(5)) {	publish(0, -1); }

			time = ros::Time::now();				
   	  arm_pub.publish(msg);
			
			while(ros::Time::now() - time < ros::Duration(4)) {	publish(0, 0); }

			//delete cube
			system("rosservice call gazebo/delete_model '{model_name: cube}'");

	}
	

}

void AGVSControl::keyLoop()
{
  char c;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

	puts("Control Robot from Keyboard");
	puts("-------------------------");
	puts("   q    w    e ");
	puts("   a    s    d");
	puts("y/h : increase/decrease linear speed by 0.1");
	puts("u/j : increase/decrease angular speed by 0.1");
	puts("z   : start autonomous demo");
  ackermann_msgs::AckermannDriveStamped ref_msg;
  double desired_linear_speed = 0.0, desired_angular_position = 0.0;
	ref_msg.header.stamp = ros::Time::now();
	ref_msg.drive.jerk = 0.0; 
	ref_msg.drive.acceleration = 0.0; 
	ref_msg.drive.steering_angle_velocity = 0.0;
			
  while (ros::ok())
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
//Linear speed control
      case 'y':
        l_scale_+=0.01;
				if(l_scale_ > MAX_LINEAR_SPEED)
					l_scale_ = MAX_LINEAR_SPEED;
				std::cout<<"Linear Scale="<<l_scale_<<std::endl;
        break;
      case 'h':
        l_scale_-=0.01;
				if(l_scale_ < 0.5)
					l_scale_ = 0.5;
				std::cout<<"Linear Scale="<<l_scale_<<std::endl;
        break;

//Angular speed control
      case 'u':
        a_scale_+=0.01;
				if(a_scale_ > MAX_ANGULAR_SPEED)
					a_scale_ = MAX_ANGULAR_SPEED;
				std::cout<<"Angular Scale="<<a_scale_<<std::endl;
        break;
      case 'j':
        a_scale_-=0.01;
				if(a_scale_ < 0.5)
					a_scale_ = 0.5;
				std::cout<<"Angular Scale="<<a_scale_<<std::endl;
        break;

//Robot control
      case 'q':
        ROS_DEBUG("UP-LEFT");
        angular_ = 1.0;
 				linear_ = 1.0;
        break;
      case 'e':
        ROS_DEBUG("UP-RIGHT");
        angular_ = -1.0;
 				linear_ = 1.0;
        break;
      case 'w':
				ROS_DEBUG("UP");
        linear_ = 1.0;
        break;
      case 'a':
        ROS_DEBUG("DOWN-LEFT");
        angular_ = 1.0;
 				linear_ = -1.0;
        break;
      case 'd':
        ROS_DEBUG("DOWN-RIGHT");
        angular_ = -1.0;
 				linear_ = -1.0;
        break;
      case 's':
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        break;
			case 'z':
				ROS_DEBUG("Autonomous Demo");
        autonomousDemo();
        break;
    }
    boost::mutex::scoped_lock lock(publish_mutex_);
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) { 
      first_publish_ = ros::Time::now();
    }
    last_publish_ = ros::Time::now();
    publish(angular_, linear_);
  }

  return;
}

void AGVSControl::publish(double angular, double linear)  
{
	ackermann_msgs::AckermannDriveStamped ref_msg; //Create msg to publish
	ref_msg.header.stamp = ros::Time::now(); //set values
	ref_msg.drive.jerk = 0.0; 
	ref_msg.drive.acceleration = 0.0; 
	ref_msg.drive.steering_angle_velocity = 0.0;

	ref_msg.drive.steering_angle = angular * a_scale_; //calculate angular speed
	ref_msg.drive.speed = linear * l_scale_; //calculate linear speed
	vel_pub_.publish(ref_msg); //publish msg
  vel_pub_2.publish(ref_msg);
  return;
}

