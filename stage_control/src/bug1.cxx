
/**
 * Bug 1
 * 
 * Will Havelin
 * February 2015.
 *
 */

#include <BugAlgorithm.cxx>

class Bug1 extends BugAlgorithm{

 private:
  double leastDistToGoal;
  bool inFirstLoop;
  double forwardVelocityMessage;
  double angularVelocityMessage;

public:
  void circumnavigateObstacle(){
    // figureOutWhichDirectionToGo();
    // if(inFirstLoop){
    //   double distToGoal = distanceToGoal();
    //   if(isDoneFirstLoop()){
    // 	inFirstLoop = false;
    //   }
    // }
    // else {
    //   if(isCloseEnough(distToGoal, leastDistToGoal)){
    // 	avoidingObstacle = false;
    // 	leastDistToGoal = ARBITRARY_HIGH_NUMBER;
    //   } 
    // }
  }
  
  void doAlgorithmSpecificIntitialization(){
    leastDistToGoal = ARBITRARY_HIGH_NUMBER;
    desiredHeading = calculateHeadingToGoal();
  }

  void runAlgorithm(){
    if(!robot.isFacingRightDirection(desiredHeading)){
      robot.turnTowardsHeading(desiredHeading, angularVelocityMessage);
    }
    else{
      if(avoidingObstacle){
	circumnavigateObstacle();
      }
      else{
	robot.moveForward(forwardVelocityMessage);
      }
    }
  }

  Bug1(){
    
  }

  int main(int argc, char **argv){
    goalX = std::atof(argv[1]); //Sets goalX to first command line argument
    goalY = std::atof(argv[2]); //Sets goalY to second command line argument

    Bug1 bug1();
    bug1.runBugAlgorithm(); //Super Class method
  }
}

