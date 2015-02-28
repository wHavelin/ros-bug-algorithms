
/**
 * Bug 1
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

const double ROBOT_WIDTH = .2;
const double STOP_DISTANCE = 1;
const double ARBITRARY_HIGH_NUMBER = 10000;
const int LOOP_RATE = 10; //How many times per second the main while loop runs
double leastDistToGoal = ARBITRARY HIGH NUMBER;
double desiredHeading = 1.0;
double goal_x = 8;
double goal_y = 15;
bool avoidingObstacle = false;

void calculateDesiredHeading(){
  desiredHeading = -1 * atan2(goal_y - myY,myX-goal_x);
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

void circumnavigateObstacle(){
  getOpenDirections();
  figureOutWhichDirectionToGo();
  if(inFirstLoop){
    double distToGoal = distanceToGoal();
    if(isDoneFirstLoop()){
      inFirstLoop = false;
    }
  }
  else {
    if(isCloseEnough(distToGoal, leastDistToGoal)){
      avoidingObstacle = false;
      leastDistToGoal = ARBITRARY_HIGH_NUMBER;
    } 
  }
}

void handlePubSub(ros::NodeHandle nh_){
  ros::Publisher cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Subscriber odom_sub = nh_.subscribe("/odom", 1000,getInfo);
  ros::Subscriber scan_sub = nh_.subscribe("/base_scan", 1000,getScan);
}

int main(int argc, char **argv)
{
  goalX = std::atof(argv[1]); //Sets goalX to first command line argument
  goalY = std::atof(argv[2]); //Sets goalY to second command line argument

  ros::init(argc, argv, "bug0");
  ros::NodeHandle nh_;

  handlePubSub(nh_);

  geometry_msgs::Twist base_cmd;
	
  ros::Rate loop_rate(LOOP_RATE);
  calculateDesiredHeading();

  while (ros::ok()){
      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
      turn_towards_heading(base_cmd.angular.z);
      else if(facingRightDirection()){
	if(avoidingObstacle){
	   circumnavigateObstacle();
	}
	if(goingToHitObstacle()){
	   avoidObstacle();
	}
	move_forward(base_cmd.linear.x);
      }
      cmd_vel_pub_.publish(base_cmd);
      ros::spinOnce();
      loop_rate.sleep();
  }	
  return 0;     
}

