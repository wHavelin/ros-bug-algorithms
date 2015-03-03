
/**
 * Bug 0
 * 
 * Will Havelin
 * February 2015.
 *
 */

#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <iostream>
#include <tf/tf.h>
#include <tf2/exceptions.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <exception>

laser_geometry::LaserProjection projector;
sensor_msgs::PointCloud cloud;

int cloudData  = 0;
const double ROBOT_WIDTH = .2;
const double STOP_DISTANCE = 1;
double myHeading = 0.0;
double myX = -15.27;
double myY = 13.266;
double my_last_x = 0.0; //Relative to start
double my_last_y = 0.0;
double avoidStartX = 0;
double avoidStartY = 0;
double desiredHeading = 1.0;
double goal_x = 8;
double goal_y = 15;
bool oldData = true;
bool move = false;
bool wayToRightBlocked = false;
bool avoidingObstacle = false;

void move_forward(double & velocity){
  velocity = 0.25;
}

bool facingRightDirection(){
  if(std::abs(myHeading - desiredHeading)< .1)
    return true;
  else
    return false;
}
void calculateDesiredHeading(){
  desiredHeading = -1 * atan2(goal_y - myY,myX-goal_x);
  std::cout << desiredHeading << "\n";
}

void turn_left(double & angular){
      angular = 0.5;
  //std::cout << "Turned left\n";
}

        void turn_right(double & angular){
	  angular = -0.5;
	 }

void turn_towards_heading(double & myAngular){
  //std::cout << "My X is " << myX << "\n";
  //std::cout << "My Y is " << myY << "\n";
  //std::cout <<"My heading is " <<  myHeading << "\n";
  //std::cout <<"My desired heading is " <<  desiredHeading << "\n";
  if(myHeading <  desiredHeading - .1){
    turn_left(myAngular);
    move = false;
  }
  else if(myHeading > desiredHeading + .1){
    turn_right(myAngular);
    move = false;
  }
  if(facingRightDirection())
    move = true;
}

bool mainValueWithinRange(double mainValue){
  if(mainValue < 0){ 
    return false;
  }
  if(mainValue > STOP_DISTANCE){
    return false;
  }
  return true;
}

bool otherValueWithinRange(double otherValue){
  if(otherValue < STOP_DISTANCE / -2){
     return false;
    }
  if (otherValue > STOP_DISTANCE / 2){
      return false;
    }
  return true;
}


bool checkIfPointToRight(geometry_msgs::Point32 p){
  if(p.y>(STOP_DISTANCE*-2.1) && p.y < 0 && (std::abs(p.x)< (1*STOP_DISTANCE))){
      return true;
  }
  else
    return false;
}

bool goingToHitPoint(geometry_msgs::Point32 p){
  if(mainValueWithinRange(p.x) && otherValueWithinRange(p.y))
     return true;
  else
    return false;
}

void getInfo(const nav_msgs::Odometry::ConstPtr& pos){
  myX += my_last_x;
  my_last_x = (pos->pose.pose.position.x);
  myX -= my_last_x;
  myY += my_last_y;
  my_last_y = (pos->pose.pose.position.y);
  myY -= my_last_y;
  myHeading = tf::getYaw(pos->pose.pose.orientation);
}

void getScan(const sensor_msgs::LaserScan::ConstPtr & scanData){
  tf::TransformListener listener;
  try{
  projector.transformLaserScanToPointCloud("world" , *scanData, cloud, listener);
  //std::cout << "Got Scan!\n";
  }
  catch(tf2::LookupException e){
    //std::cout << "lookupException\n";
  }
}


bool goingToHitObstacle(){
for(int i = 0; i < cloud.points.size(); i++) {
    if(goingToHitPoint(cloud.points[i])){
       return true;
    }
 }
  return false;
}

void avoidObstacle(){
  //std::cout << "Obstacle Detected at Heading\n " << myHeading << "\n";
  avoidingObstacle = true;
  desiredHeading += .1;
  if(desiredHeading > 3.15){
    desiredHeading -= 6.28; // 2 PI
  }
}

int main(int argc, char **argv)
{
  goal_x = std::atof(argv[1]);
  goal_y = std::atof(argv[2]);
  myX = std::atof(argv[3]);
  myY = std::atof(argv[4]);
   /// Name your node
	ros::init(argc, argv, "bug0");
	/// Every ros node needs a node handle, similar to your usual  file handle.
	ros::NodeHandle nh_;

	/// Publisher object that decides what kind of topic to publish and how fast.
	ros::Publisher cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Subscriber odom_sub = nh_.subscribe("/odom", 1000,getInfo);
        ros::Subscriber scan_sub = nh_.subscribe("/base_scan", 1000,getScan);
	// We will be sending commands of type "twist"
	geometry_msgs::Twist base_cmd;
	
		/// The main loop will run at a rate of 10Hz, i.e., 10 times per second.
	ros::Rate loop_rate(10);
	/// Standard way to run ros code. Will quite if ROS is not OK, that is, the master is dead.

        calculateDesiredHeading();
	while (ros::ok())
	{
	  if(std::abs(myX - goal_x) < .3 && std::abs(myY - goal_y) < .3){
	    return 0;
	  }
	  base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
	  turn_towards_heading(base_cmd.angular.z);
	  if(facingRightDirection()){
	    if(avoidingObstacle){
	      wayToRightBlocked = false;
	       for(int i = 0; i < cloud.points.size(); i++){
	       	  if(checkIfPointToRight(cloud.points[i])){
	       	    wayToRightBlocked = true;
	       	  }
	       	}
	       	if(!wayToRightBlocked){
	       	  avoidingObstacle = false;
	       	  calculateDesiredHeading();
	       	}
	    }
	    if(goingToHitObstacle()){
	      avoidObstacle();
	    }
	    move_forward(base_cmd.linear.x);
	  }
	  		/// Here's where we publish the actual message.
		cmd_vel_pub_.publish(base_cmd);
		/// Spin the ros main loop once 
		ros::spinOnce();
		/// Sleep for as long as needed to achieve the loop rate.
		loop_rate.sleep();
	  }
		
	return 0;
       
}

