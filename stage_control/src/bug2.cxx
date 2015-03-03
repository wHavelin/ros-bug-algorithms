
/**
 * Bug 2
 * 
 * Will Havelin
 * February 2015.
 *
 */

#include <vector>
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

std::vector<geometry_msgs::Point32> lineToGoal;
const double STOP_DISTANCE = 1;
double myHeading = 0.0;
double myX = -15.27;
double myY = 13.266;
double my_last_x = 0.0; //Relative to start
double my_last_y = 0.0;
double avoidStartX = 0;
double avoidStartY = 0;
double desiredHeading = 1.0;
double goalX = 8;
double goalY = 15;
bool move = false;
bool startedGoingAround = false;
bool wayToLeftBlocked = false;
bool wentLeftLastTime = false;
bool wayToRightBlocked = false;
bool wayForwardBlocked = false;
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
  desiredHeading = -1 * atan2(goalY - myY,myX-goalX);
  std::cout << desiredHeading << "\n";
}

void turn_left(double & angular){
      angular = 1;
  //std::cout << "Turned left\n";
}

        void turn_right(double & angular){
	  angular = -1;
	 }

void turn_towards_heading(double & myAngular){
  if(desiredHeading > 3.14){
    desiredHeading -= 6.28;
  }
  else if(desiredHeading <= -3.14){
    desiredHeading += 6.28;
  }
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
  if(p.y < 0 && p.y > STOP_DISTANCE  * -1.3 && otherValueWithinRange(p.x)){
      return true;
  }
  else
    return false;
}

bool checkIfPointToLeft(geometry_msgs::Point32 p){
  if(p.y >  0 && p.y < STOP_DISTANCE  * 1.3 && otherValueWithinRange(p.x)){
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

void goForward(){
  //std::cout << "Going forward\n";
}

void goLeft(){
  wentLeftLastTime = true;
  desiredHeading += 1.57;
  //std::cout << "Going Left\n";
}

void goRight(){
  wentLeftLastTime = false;
  desiredHeading -= 1.57;
  //std::cout << "Going Right\n";
}

void goBack(){
  wentLeftLastTime = false;
  desiredHeading *= -1;
  //std::cout << "Going Back\n";
}

void figureOutNewWayToGo(bool oldLeft, bool oldRight, bool oldForward){
  if(wayForwardBlocked){
    if(wayToLeftBlocked){
      if(wayToRightBlocked){
	goBack();
	return;
      }
      else{
	goRight();
	return;
      }
    }
    else if(wayToRightBlocked){
      goLeft();
      return;
    }
    else if(wentLeftLastTime){
      goRight();
      return;
    }
    else{
      goLeft();
      return;
    }
  }
  else if(wayToLeftBlocked){
    if(wayToRightBlocked){
      goForward();
      return;
    }
    else{
      if(oldLeft){
	goLeft();
	return;
      }
      else{
	goForward();
	return;
      }
    }
  }
  else if(wayToRightBlocked){
    if(oldLeft){
      goLeft();
      return;
    }
    else{
      goForward();
      return;
    }
  }
  else{
    if(oldRight = true){
      goRight();
      return;
    }
    else{
      goLeft();
      return;
    }
  }
}

void circumnavigateObstacle(){
  if(std::abs(myX - avoidStartX) < .7 && std::abs(myY - avoidStartY)< .7){ 
    if(startedGoingAround){
      //std::cout << "done with first loop\n";
    }
  }
  else{
    if(!startedGoingAround && (std::abs(myX - avoidStartX) > 1 || std::abs(myY - avoidStartY) > 1)){
	startedGoingAround = true;
	//std::cout << "Started going around\n";
    }
  }
  
  bool oldLeft = wayToLeftBlocked;
  bool oldRight = wayToRightBlocked;
  bool oldForward = wayForwardBlocked;
  wayToLeftBlocked = false;
  wayToRightBlocked = false;
  wayForwardBlocked = false;
  for(int i = 0; i < cloud.points.size(); ++i){
    if(checkIfPointToLeft(cloud.points[i])){
      wayToLeftBlocked = true;
    }
    if(checkIfPointToRight(cloud.points[i])){
      wayToRightBlocked = true;
    }
    if(goingToHitPoint(cloud.points[i])){
      wayForwardBlocked = true;
    }
  }
  if(oldLeft != wayToLeftBlocked || oldRight != wayToLeftBlocked || oldForward != wayForwardBlocked){
    figureOutNewWayToGo(oldLeft, oldRight, oldForward);
  }
  else{
    //std::cout << "Staying the course\n";
  }
}

bool onLineToGoal(){
  for(int i = 0; i < lineToGoal.size(); ++i){
    if(std::abs(myX - lineToGoal[i].x) < .1){
      if(std::abs(myY - lineToGoal[i].y) < .1){
	return true;
      }
    }
  }
  return false;
}

double calculateDistanceFromGoal(){
  return std::sqrt(std::pow(std::abs(myX -goalX), 2) +  std::pow(std::abs(myY -goalY), 2));
}

  void createLineToGoal(){
    double x = myX;
    double y = myY;
    geometry_msgs::Point32 p;
    while(x < goalX){
      x += .1; 
      y += ((goalY -myY)* .1) / (goalX - myX);
      p.x = x;
      p.y = y;
      lineToGoal.push_back(p);
    }
    y = myY;
    x = myX;
    while(y < goalY){
      y += .1;
      x += ((goalX - myX) * .1) / (goalY - myY);
      p.x = x;
      p.y = y;
      lineToGoal.push_back(p);
    }
  }
int main(int argc, char **argv)
{
  goalX = std::atof(argv[1]);
  goalY = std::atof(argv[2]);
  myX = std::atof(argv[3]);
  myY = std::atof(argv[4]);
   /// Name your node
	ros::init(argc, argv, "bug2");
	/// Every ros node needs a node handle, similar to your usual  file handle.
	ros::NodeHandle nh_;

	/// Publisher object that decides what kind of topic to publish and how fast.
	ros::Publisher cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Subscriber odom_sub = nh_.subscribe("/odom", 1000,getInfo);
        ros::Subscriber scan_sub = nh_.subscribe("/base_scan", 1000,getScan);
	// We will be sending commands of type "twist"
	geometry_msgs::Twist base_cmd;
	
		/// The main loop will run at a rate of 10Hz, i.e., 10 times per second.
	ros::Rate loop_rate(50);
	/// Standard way to run ros code. Will quite if ROS is not OK, that is, the master is dead.

	createLineToGoal();
        calculateDesiredHeading();
	while (ros::ok())
	{
	  base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
	  //std::cout << desiredHeading << "\n";
	  turn_towards_heading(base_cmd.angular.z);
	  if(std::abs(myX - goalX) < .3 && std::abs(myY - goalY) < .3){
	    //std::cout << "reached the goal";
	    return 0;
	  }
	  if(facingRightDirection()){
	    if(avoidingObstacle){
	      circumnavigateObstacle();
	      if(onLineToGoal() && startedGoingAround){
		avoidingObstacle = false;
		calculateDesiredHeading();
		//std::cout << "done with obstacle continuing to goal\n";
	      }
	    }
	    if(goingToHitObstacle()){
	      if(!avoidingObstacle){
		std::cout << "Started Avoiding Obstacle\n" << myX << "," << myY << "\n";
		avoidStartX = myX;
		avoidStartY = myY;
		startedGoingAround = false;
		avoidingObstacle = true;
	      }
	    }
	    else{
	      move_forward(base_cmd.linear.x);
	    }
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

