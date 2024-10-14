#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/Infrastructure/RefereeEstimator.h"
#include "Tools/Settings.h"

CARD(StandbyCard,
{,
  CALLS(Activity),
  CALLS(LookActive),
  CALLS(LookAtPoint),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkPotentialField),
  CALLS(WalkToKickoffPose),
  CALLS(Say),
  REQUIRES(BallSpecification),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(LibPosition),
  REQUIRES(LibMisc),
  REQUIRES(RefereeEstimator),

  LOADS_PARAMETERS(
  {,
    (float) xGlobReferee,
    (float) yGlobReferee,
  }),
});

class StandbyCard : public StandbyCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_STANDBY;
  }

  bool postconditions() const override
  {
    return theGameInfo.state != STATE_STANDBY;
  }

  template <typename T> int sign(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::standby);
    theSaySkill("STANDBY");
    LocalPose2f relative = theLibMisc.glob2Rel(0, -sign(theRobotPose.translation.y())*theFieldDimensions.yPosLeftSideline); // relative coordinate of opposite T-Junction
    theLookAtPointSkill(Vector3f(relative.translation.x(), relative.translation.y(), 2500.0), HeadMotionRequest::upperCamera);
    theStandSkill();
  }
};

MAKE_CARD(StandbyCard);
