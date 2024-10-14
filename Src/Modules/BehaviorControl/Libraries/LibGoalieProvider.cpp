/**
 * @file LibGoalieProvider.cpp
 * 
 * See LibGoalie
 *
 * @author Francesco Petri
 */

#include "LibGoalieProvider.h"
#include "Tools/Math/BHMath.h"    // between

MAKE_MODULE(LibGoalieProvider, behaviorControl);

void LibGoalieProvider::update(LibGoalie& libGoalie)
{
  libGoalie.goaliePosition = updateGoalie();
  libGoalie.isGoalieInStartingPosition = isGoalieInStartingPosition();
  libGoalie.isGoalieInAngle = isGoalieInAngle();
  libGoalie.isBallInKickAwayRange = isBallInKickAwayRange();
  libGoalie.isGoalieInKickAwayRange = isGoalieInKickAwayRange();
}


LibGoalieProvider::LibGoalieProvider()
{
  SPQR::ConfigurationParameters();
}


Vector2f LibGoalieProvider::updateGoalie() const
{
  Pose2f globBall = theLibMisc.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());

                  //-5055
  float deltaX = ((theFieldDimensions.xPosOwnGoal - globBall.translation.x()));// * 0.5f);
  float deltaY = ((globBall.translation.y()));// * 0.5f);

  if(deltaX > theFieldDimensions.xPosOwnGoalArea)
      deltaX = theFieldDimensions.xPosOwnGoalArea - 200 ;

  if (deltaY < theFieldDimensions.yPosRightGoal + 200 )
      deltaY = theFieldDimensions.yPosRightGoal + 200;

  if (deltaY > theFieldDimensions.yPosLeftGoal - 200 )
      deltaY = theFieldDimensions.yPosLeftGoal - 200;

  else if((deltaY < -200 && deltaY > theFieldDimensions.yPosRightGoal + 200) || (deltaY > 200 && deltaY < theFieldDimensions.yPosRightGoal - 200))
    deltaY = deltaY/2;


  return Vector2f(deltaX, deltaY);
}

bool LibGoalieProvider::isGoalieInStartingPosition() const {
  if( theLibMisc.isValueBalanced(theRobotPose.translation.x(), SPQR::GOALIE_BASE_POSITION_X+1000, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
          theLibMisc.isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y+1000, SPQR::GOALIE_POSE_Y_TOLLERANCE) )
    return true;
  else
    return false;
}

bool LibGoalieProvider::isGoalieInAngle() const
{
  if (between(theRobotPose.rotation, Angle::fromDegrees(-10.f), Angle::fromDegrees(10.f) ))
    return true;
  else
    return false;
}

bool LibGoalieProvider::isBallInKickAwayRange() const
{
  if( theBallModel.estimate.position.norm() < SPQR::GOALIE_KICK_AWAY_RANGE )
    return true;
  else
    return false;
}

bool LibGoalieProvider::isGoalieInKickAwayRange() const
{
  Pose2f gloBall = theLibMisc.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
  // [2023] new based on BHuman. Old is around the very beginning of the repo2023 history, or in repo2022.
  return theLibPosition.isNearOwnGoalArea(gloBall.translation, kickAwayTolerance, kickAwayTolerance);
}
