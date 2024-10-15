/**
 * @file SearcherActiveCard.cpp
 *
 *
 * @author Daniele Affinita
 */


#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/BehaviorControl/Libraries/LibSearcher.h"
#include "Representations/spqr_representations/SearcherModel.h"
#include "Representations/Infrastructure/FrameInfo.h"

CARD(SearcherActiveCard,
{,
    CALLS(Activity),
    CALLS(LookLeftAndRight),
    CALLS(Stand),
    CALLS(WalkAtRelativeSpeed),
    CALLS(WalkToPoint),
    CALLS(LookForward),
    CALLS(LookAtPoint),
    CALLS(LookAtAngles),

    USES(LibMisc),
    REQUIRES(PlayerRole),
    REQUIRES(TeamData),
    REQUIRES(FieldDimensions),
    REQUIRES(RobotPose),
    REQUIRES(TeamBallModel),
    REQUIRES(LibSearcher),
    REQUIRES(SearcherModel),
    REQUIRES(FrameInfo),

    LOADS_PARAMETERS(
    {,
        (uint16_t) INITIAL_WAIT_TIME,
        (float) BALL_ALIGN_THRESHOLD,
        (uint16_t) HYSTERESIS_TIME,
        (uint16_t) HYSTERESIS_DISTANCE,
    }),
});

class SearcherActiveCard : public SearcherActiveCardBase
{
    unsigned lastPositionUpdateTime = 0; //giving some hysteresis to searchToTarget state
    Vector2f currentTarget;
    bool preconditions() const override{
        return true;
    }

    bool postconditions() const override{
        return true;
    }

    option
    {
        theActivitySkill(BehaviorStatus::SearcherActive);
        initial_state(start)
        {
            transition
            {
                if(state_time > INITIAL_WAIT_TIME)
                    goto searchToTarget; 
            }

            action
            {
                theLookLeftAndRightSkill();
                theStandSkill();
            }
        }
        
        state(searchToTarget)
        {
            transition
            {
                if(theLibMisc.distance(theRobotPose.translation, currentTarget) < 2*theSearcherModel.cellLengthX){
                    goto turnAround;
                }
            }

            action
            {  
                // hysteresis
                if(theFrameInfo.getTimeSince(lastPositionUpdateTime) > HYSTERESIS_TIME || theLibMisc.distance(theRobotPose.translation, currentTarget) < HYSTERESIS_DISTANCE){
                    currentTarget = theLibSearcher.getActiveSearcherPosition();
                    lastPositionUpdateTime = theFrameInfo.time;
                }

                theWalkToPointSkill(theLibMisc.glob2Rel(currentTarget.x(), currentTarget.y()), 0.7f);
                //theLookLeftAndRightSkill(true, 50_deg, 23_deg, 80_deg);
                theLookAtAnglesSkill(0_deg, 50_deg, 90_deg);
            }
        }

        state(turnAround)
        {
            transition
            {
                if(theLibMisc.distance(theRobotPose.translation, currentTarget) > 2*theSearcherModel.cellLengthX)
                    goto searchToTarget;
            }

            action
            {
                float turn_direction;
                currentTarget = theLibSearcher.getActiveSearcherPosition();
                lastPositionUpdateTime = theFrameInfo.time;
                theLookForwardSkill();
                if(theLibMisc.angleToTarget(currentTarget.x(), currentTarget.y()) > 0)
                {
                    turn_direction = 1.f;
                }
                else
                {
                    turn_direction = -1.f;
                }
                theWalkAtRelativeSpeedSkill(Pose2f(turn_direction, 0.f, 0.f));
            }
        }
    }
};

MAKE_CARD(SearcherActiveCard);
