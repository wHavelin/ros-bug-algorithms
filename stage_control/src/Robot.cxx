#include <geometry_msgs/Pose.h>

class Robot{

private:
  const double FORWARD_VELOCITY = .25;
  const double HEADING_MARGIN_OF_ERROR = .1;
  const double TURN_VELOCITY = .75;

public:
  double myX;
  double myY; 
  double myHeading = 0;
  double myLastX = 0; //Relative to self
  double myLastY = 0;

  public Robot(double startX, double startY){
    myX = startX;
    myY = startY;
  }

  public void moveForward(double & velocity){
    velocity = FORWARD_VELOCITY;
  }

  public void turnTowardsHeading(double desiredHeading, double & angular){
    if(myHeading <  desiredHeading - HEADING_MARGIN_OF_ERROR){
      angular = -1 * TURN_VELOCITY;
    }
    else if(myHeading > desiredHeading + HEADING_MARGIN_OF_ERROR){
      angular = TURN_VELOCITY;
    }
  }

  public void updateStateWithOdometry(geometry_msgs::Pose odomPose){
    myX += myLastX;
    myLastX = odomPose.position.x;
    myX -= myLastX;

    myY += myLastY;
    myLastY = odomPose.position.y;
    myY -= myLastY;
    
    myHeading = tf::getYaw(odomPose.orientation);
  } 

  public bool isFacingRightDirection(double desiredHeading){
    if(std::abs(myHeading - desiredHeading) < HEADING_MARGIN_OF_ERROR){
      return true;
    }
    else{
      return false;
    }
  }

}
