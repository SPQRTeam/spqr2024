/**
 * @file Interceptor1Card.cpp
 *
 * This file implements a basic behavior for Interceptor.
 *
 * @author Flavio Volpi
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/BallModel.h"

#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Geometry.h"
#include <Eigen/Geometry>
#include <iostream>

using Line2 = Eigen::Hyperplane<float,2>;

CARD(Interceptor1Card,
{,
  CALLS(Activity),
  CALLS(LookAtBall),
  CALLS(WalkToPoint),
  REQUIRES(FieldBall),
  REQUIRES(BallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(BallSpecification),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
  }),
});

class Interceptor1Card : public Interceptor1CardBase
{


    
  bool preconditions() const override
  {

    /* INTERCEPTOR 1
     * Interceptor that achieves the end position of the ball.
     * More or less it seems to work.
     * Potentially, can be added FieldBall.isRollingTowardsOwnGoal
    **/

    bool condition = getCondition();
    return condition;
  }

  bool postconditions() const override
  {
    /* INTERCEPTOR 1*/

    bool condition = getCondition();
    return !condition;
  }

  option
  {
    theActivitySkill(BehaviorStatus::Dummy);

    initial_state(start)
    {
      transition
      {
        
      }

      action
      {
        theLookAtBallSkill();
        
        Vector2f target = theFieldBall.recentBallEndPositionRelative();
        theWalkToPointSkill(target);
      }
    } 
  }

  bool getCondition() const
  {
    Vector2f target = theFieldBall.recentBallEndPositionRelative();
    float timeWalking = abs(target.norm() / theWalkingEngineOutput.maxSpeed.translation.x());
    float timeBallToRest = abs(theBallModel.estimate.velocity.norm() / (theBallSpecification.friction*1000));
    
    bool condition = (timeWalking <= timeBallToRest); 
    return condition;
  }
};

MAKE_CARD(Interceptor1Card);
