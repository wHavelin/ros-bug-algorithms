
/**
 * Bug Algorithm Super Class  
 * Will Havelin
 * February 2015.
 *
 */

#include <Robot.cxx>
#include <laser_geometry.h>
#include <sensor_msgs.h>

class BugAlgorithm{

private:
  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud cloud;

  String alogrithmName = "No Algorithm";
  const double STOP_DISTANCE = 1;
  const double ARBITRARY_HIGH_NUMBER = 10000;
  const double CLOSE_ENOUGH_CONSTANT = .1;
  const int LOOP_RATE = 10; //How many times per second the main while loop runs

  double desiredHeading;
  double goalX;
  double goalY;
  bool avoidingObstacle;
  Robot robot;
  bool[] directionIsOpen = new bool[8]; // 0 is forward, 1 is forward/right, 2 is right and so on

public:
  BugAlgorithm(double startX, double startY,double goalX, double goalY){
    robot = new Robot(startX, startY);
    this.goalX = goalX;
    this.goalY = goalY;
    avoidingObstacle = false;
    leastDistToGoal = ARBITRARY_HIGH_NUMBER;
    desiredHeading = calculateHeadingToGoal();
  }

  double calculateHeadingFromPointToPoint(double p1x, double p1y, double p2x, double p2y){
    return -1 * atan2(p2y - p1y, p1x - p2x);
  }

  double calculateHeadingToGoal(){
    return calculateHeadingFromPointToPoint(robot.getX(),robot.getY(),goalX, goalY);
  }

  bool valueWithinRange(double value, double lowerBound, double upperBound){
    if(value < lowerBound){
      return false;
    }
    if(value > upperBound){
      return false;
    }
    return true;
  }

  bool forwardValueWithinRange(double pointX){
    double distance = pointX - robot.getX();
    return valueWithinRange(distance, 0 , STOP_DISTANCE);
  }

  bool sideValueWithinRange(double pointY){
    double distance = pointY - robot.getY();
    return valueWithinRange(distance, (-.5 * STOP_DISTANCE),(.5 * STOP_DISTANCE);
  }

  bool goingToHitPoint(geometry_msgs::Point32 p){
    if(forwardValueWithinRange(p.x) && sideValueWithinRange(p.y))
      return true;
    else
      return false;
  }

  void updateRobotStateWithOdometry(const nav_msgs::Odometry::ConstPtr& pos){
    robot.updateStateWithOdometry(pos->pose.pose);
  }

  void getScan(const sensor_msgs::LaserScan::ConstPtr & scanData){
    tf::TransformListener listener;
    try{
      projector.transformLaserScanToPointCloud("world" , *scanData, cloud, listener);
    }
    catch(tf2::LookupException e){    
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

  bool isCloseEnough(double value1, double value2){
    if(std::abs(value1 - value2) < CLOSE_ENOUGH_CONSTANT){
      return true;
    }
    else{
      return false;
    }
  }

  void avoidObstacle(){
    avoidingObstacle = true;
    desiredHeading += .1;
    if(desiredHeading > 3.15){
      desiredHeading -= 6.28; // 2 PI
    }
  }

  void createPublishersAndSubscribers(ros::NodeHandle nh_){
    ros::Publisher robotMovementPublisher = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber odomSubcriber = nh_.subscribe("/odom", 1000,updateRobotStateWithOdometry);
    ros::Subscriber scanSubcriber = nh_.subscribe("/base_scan", 1000,getScan);
  }

  void setRobotMovementMessageToZero(geometry_msgs::Twist robotMovementMessage){
    robotMovementMessage.linear.x = 0;
    robotMovementMessage.linear.y = 0;
    robotMovementMessage.angular.z = 0;
  }

  void runAlgorithm(){
    //MEANT TO BE OVERIDDEN BY SUBCLASSSES
  }

  void doAlgorithmSpecificIntitalization(){
    //MEANT TO BE OVERIDDEN BY SUBCLASSES
  }

  void runBugAlgorithm(){
     ros::init(argc, argv, algorithmName);
    ros::NodeHandle nh_;

    createPublishersAndSubscribers(nh_);

    geometry_msgs::Twist robotMovementMessage;
	
    ros::Rate loop_rate(LOOP_RATE);
    
    doAlgorithmSpecificInitializitation();

    while (ros::ok()){
      setRobotMovementMessageToZero(robotMovementMessage);
      runAlgorithm(); // run algorithm is overidden by subclasses
      robotMovementPublisher.publish(robotMovementMessage);
      ros::spinOnce();
      loop_rate.sleep();
    }	
  }


}
