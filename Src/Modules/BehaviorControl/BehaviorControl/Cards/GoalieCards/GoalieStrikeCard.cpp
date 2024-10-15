/**
 * @file GoalieStrikeCard.cpp
 *
 * This file implements a basic striker behavior for the striker.
 *
 * @author Emanuele Antonioni
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Representations/BehaviorControl/Libraries/LibDefender.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/Modeling/OpponentGoalModel.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/spqr_representations/GameState.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Tools/Math/BHMath.h"

#include <iostream>

CARD(GoalieStrikeCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(GoToBallAndDribble),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(Dribble),
  CALLS(LookActive),
  CALLS(LookAtBall),
  CALLS(WalkToBallAndKick),
  CALLS(WalkPotentialField),
  CALLS(LookAtGlobalBall),

  REQUIRES(LibMisc),
  REQUIRES(OpponentGoalModel),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(LibStriker),
  REQUIRES(LibObstacles),
  REQUIRES(GameInfo),
  REQUIRES(GameState),
  REQUIRES(OwnTeamInfo),
  REQUIRES(PathPlanner),
  REQUIRES(BallModel),
  REQUIRES(TeamBallModel),
  USES(LibDefender),
  REQUIRES(RobotInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(100) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
    (float)(80) kickIntervalThreshold,
  }),
});

class GoalieStrikeCard : public GoalieStrikeCardBase
{
  bool preconditions() const override
  {
    bool shouldDive = (theTeamBallModel.velocity.x() < -350 && theFieldBall.teamEndPositionOnField.x() < -4400 && theFieldBall.teamEndPositionOnField.y() < 1200 && theFieldBall.teamEndPositionOnField.y() > -1200);
    return (theGameState.state==GameState::State::ownGoalKick) || (theFieldBall.isInsideGoalRectangle && !shouldDive);
  }

  bool postconditions() const override
  {
    bool shouldDive = (theTeamBallModel.velocity.x() < -350 && theFieldBall.teamEndPositionOnField.x() < -4400 && theFieldBall.teamEndPositionOnField.y() < 1200 && theFieldBall.teamEndPositionOnField.y() > -1200);
    return !((theGameState.state==GameState::State::ownGoalKick) || (theFieldBall.isInsideGoalRectangle && !shouldDive));
  }

  option
  {
    theActivitySkill(BehaviorStatus::Striker);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto goToBallAndKick;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(goToBallAndKick)
    {
      transition
      { 
        float interval = theLibStriker.goalTargetWithArea(false, false).second.interval;
      }

      action
      {
        bool doItAsap = theLibObstacles.nearestOpponent().translation.norm() < 750.f;

        Vector2f ballGoalv = theFieldBall.positionOnField - Vector2f(0, 0);
        Vector2f ballRobotv = theFieldBall.positionOnField - theRobotPose.translation;
        ballGoalv = ballGoalv.rotate(-pi_2);

        bool kickRight = ballRobotv.dot(ballGoalv) < 0;

        KickInfo::KickType typeKick = theLibStriker.getKick(doItAsap,kickRight);

        Vector2f target = theLibStriker.goalTarget(doItAsap, true);
        Angle angle = theLibMisc.calcAngleToTarget(target);

        theGoToBallAndKickSkill(angle, typeKick, !doItAsap);
      }
    }
  }
};

MAKE_CARD(GoalieStrikeCard);
