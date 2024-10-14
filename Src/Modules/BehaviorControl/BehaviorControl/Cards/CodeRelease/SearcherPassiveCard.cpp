/**
 * @file SearcherPassiveCard.cpp
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

CARD(SearcherPassiveCard,
{,
    CALLS(Activity),
    CALLS(LookLeftAndRight),
    CALLS(Stand),
    CALLS(WalkAtRelativeSpeed),
    CALLS(WalkToPoint),
    CALLS(LookForScan),

    USES(LibMisc),
    REQUIRES(PlayerRole),
    REQUIRES(TeamData),
    REQUIRES(FieldDimensions),
    REQUIRES(RobotPose),
    REQUIRES(TeamBallModel),

    LOADS_PARAMETERS(
    {,
        (int) INITIAL_WAIT_TIME,
        (int) GUARD_OFFT_X,
        (int) GUARD_OFFT_Y,
        (int) POSITION_TAKEN_TH,
        (int) TURN_EVERY,
        (int) ROTATION_TH,
    }),
});

class SearcherPassiveCard : public SearcherPassiveCardBase
{
    const float guard_y = theFieldDimensions.yPosLeftPenaltyArea - GUARD_OFFT_Y, //TODO: check it
                guard_x = theFieldDimensions.xPosOwnPenaltyArea - GUARD_OFFT_X; //TODO: check it
    private:

    // if both is true, y doesn't care
    bool isInPosition(Pose2f p, float y, bool both=false){
        if(both){
            return (p.translation-Vector2f(guard_x, guard_y)).norm() < POSITION_TAKEN_TH || 
                   (p.translation-Vector2f(guard_x, -guard_y)).norm() < POSITION_TAKEN_TH;
        }

        return (p.translation-Vector2f(guard_x, y)).norm() < POSITION_TAKEN_TH;
    }

    public:

    Vector2f chosen_position;
    bool preconditions() const override{
        return thePlayerRole.role == PlayerRole::passiveSearcher;
    }

    bool postconditions() const override{
        return true;
    }

    option
    {
        theActivitySkill(BehaviorStatus::SearcherPassive);
        initial_state(start)
        {
            transition
            {
                if(state_time > INITIAL_WAIT_TIME)
                    goto takePosition; 
            }

            action
            {
                theLookLeftAndRightSkill();
                theStandSkill();
            }
        }

        state(takePosition)
        {
            transition
            {
                if(isInPosition(theRobotPose, 0, true))
                {
                    goto turnAround;
                }
            }

            action
            {   

                float chosen_y;

                if(theRobotPose.translation.y() > 0){
                    chosen_y = guard_y;
                    for(Teammate teammate : theTeamData.teammates){
                        if(isInPosition(teammate.theRobotPose, guard_y)){
                            chosen_y = -guard_y;
                        }
                    }
                }

                else{
                    chosen_y = -guard_y;
                    for(Teammate teammate : theTeamData.teammates){
                        if(isInPosition(teammate.theRobotPose, -guard_y)){
                            chosen_y = guard_y;
                        }
                    }
                }

                chosen_position = Vector2f(guard_x, chosen_y);
                theLookLeftAndRightSkill();
                theWalkToPointSkill(theLibMisc.glob2Rel(chosen_position.x(), chosen_position.y()));
            }
        }

        state(turnAround)
        {
            transition
            {
                if(state_time > 1000 && abs(theRobotPose.rotation.toDegrees()) < ROTATION_TH)
                    goto faceOpponentGoal;
            }

            action
            {
                float turn_direction = chosen_position.y() > 0 ? 1 : -1;
                theLookForScanSkill(180_deg);
                theWalkAtRelativeSpeedSkill(Pose2f(turn_direction));
            }
        }

        state(faceOpponentGoal)
        {
            transition
            {
                if(state_time > TURN_EVERY)
                    goto turnAround;
            }

            action
            {                
                theStandSkill();
                theLookForScanSkill(180_deg);
            }
        }
    }
};

MAKE_CARD(SearcherPassiveCard);
