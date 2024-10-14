/**
 * @file Representations/Modeling/OpponentGoalModel.cpp
 * 
 * Implementation of the drawing functions of the OpponentGoalModel
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "OpponentGoalModel.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void OpponentGoalModel::draw() const
{
  #ifdef TARGET_SIM
  // drawing of the model in the field view
  const OpponentGoalModel& theOpponentGoalModel = static_cast<const OpponentGoalModel&>(Blackboard::getInstance()["OpponentGoalModel"]);
  if(theOpponentGoalModel.graphicalDebug)
  {
    DEBUG_DRAWING3D("representation:OpponentGoalModel", "field")
    {
      const PlayerRole& thePlayerRole = static_cast<const PlayerRole&>(Blackboard::getInstance()["PlayerRole"]);
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      const GameInfo& theGameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]);
        
      if(freeGoalTargetableAreas.size()!=0 && theGameInfo.state == STATE_PLAYING && (thePlayerRole.role==PlayerRole::RoleType::striker)) 
      {
        const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
        int red = 55;
        int green = 255;
        int inc = 200/freeGoalTargetableAreas.size();
        for(const auto& targetableArea : freeGoalTargetableAreas)
        {
          if(theRobotPose.translation.x()>0){
            if(theOpponentGoalModel.showRays)
            {
              LINE3D("representation:OpponentGoalModel", theRobotPose.translation.x(), theRobotPose.translation.y(), 10, theFieldDimensions.xPosOpponentGroundLine, targetableArea.begin, 10, 1, ColorRGBA(red,green,0));
              LINE3D("representation:OpponentGoalModel", theRobotPose.translation.x(), theRobotPose.translation.y(), 10, theFieldDimensions.xPosOpponentGroundLine, targetableArea.end, 10, 1, ColorRGBA(red,green,0));    
            }
            if(theOpponentGoalModel.showWalls)
            {
              int WALL_DEPTH = 20;
              QUAD3D2("representation:OpponentGoalModel", 
                      Vector3f(theFieldDimensions.xPosOpponentGroundLine, targetableArea.begin, 0), \
                      Vector3f(theFieldDimensions.xPosOpponentGroundLine, targetableArea.end, 0), \
                      Vector3f(theFieldDimensions.xPosOpponentGroundLine, targetableArea.end, 800), \
                      Vector3f(theFieldDimensions.xPosOpponentGroundLine, targetableArea.begin, 800), \
                      10, ColorRGBA(red,green,0,90));
              QUAD3D2("representation:OpponentGoalModel", 
                      Vector3f(theFieldDimensions.xPosOpponentGroundLine, targetableArea.begin, 800), \
                      Vector3f(theFieldDimensions.xPosOpponentGroundLine, targetableArea.end, 800), \
                      Vector3f(theFieldDimensions.xPosOpponentGroundLine, targetableArea.end, 0), \
                      Vector3f(theFieldDimensions.xPosOpponentGroundLine, targetableArea.begin, 0), \
                      10, ColorRGBA(red,green,0,90));
              
              /*Vector3f a(theFieldDimensions.xPosOpponentGroundLine-WALL_DEPTH, targetableArea.begin, 0);
              Vector3f b(theFieldDimensions.xPosOpponentGroundLine-WALL_DEPTH, targetableArea.begin, 800);
              Vector3f c(theFieldDimensions.xPosOpponentGroundLine-WALL_DEPTH, targetableArea.end, 0);
              Vector3f d(theFieldDimensions.xPosOpponentGroundLine-WALL_DEPTH, targetableArea.end, 800);
              Vector3f e(theFieldDimensions.xPosOpponentGroundLine+WALL_DEPTH, targetableArea.begin, 0);
              Vector3f f(theFieldDimensions.xPosOpponentGroundLine+WALL_DEPTH, targetableArea.begin, 800);
              Vector3f g(theFieldDimensions.xPosOpponentGroundLine+WALL_DEPTH, targetableArea.end, 0);
              Vector3f h(theFieldDimensions.xPosOpponentGroundLine+WALL_DEPTH, targetableArea.end, 800);
              /**CUBE3D("representation:OpponentGoalModel", a, \
                                                         b, \
                                                         c, \
                                                         d, \
                                                         e, \
                                                         f, \
                                                         g, \
                                                         h, \                                                         
                                                         4, ColorRGBA(red,green,0,127));**/
            }
            else
            {
              LINE3D("representation:OpponentGoalModel", theFieldDimensions.xPosOpponentGroundLine, targetableArea.begin, 10, theFieldDimensions.xPosOpponentGroundLine, targetableArea.end, 10, 2, ColorRGBA(red,green,0));
            }
            red+=inc;
            green-=inc;
          }
        }
      
        const BallSpecification& theBallSpecification = static_cast<const BallSpecification&>(Blackboard::getInstance()["BallSpecification"]);
        if(theOpponentGoalModel.showWalls)
        {
          SPHERE3D("representation:OpponentGoalModel", theOpponentGoalModel.utilityGoalTarget.translation.x(), theOpponentGoalModel.utilityGoalTarget.translation.y(), 400, theBallSpecification.radius, ColorRGBA(255,0,0,200));
          SPHERE3D("representation:OpponentGoalModel", theOpponentGoalModel.shootASAPGoalTarget.translation.x(), theOpponentGoalModel.shootASAPGoalTarget.translation.y()+2, 400, theBallSpecification.radius, ColorRGBA(255,255,0,200));
          SPHERE3D("representation:OpponentGoalModel", theFieldDimensions.xPosOpponentGroundLine, theOpponentGoalModel.myGazeProjection, 400, theBallSpecification.radius, ColorRGBA(0,255,255,127));
        }
        else
        {
          SPHERE3D("representation:OpponentGoalModel", theOpponentGoalModel.utilityGoalTarget.translation.x(), theOpponentGoalModel.utilityGoalTarget.translation.y(), 10, theBallSpecification.radius, ColorRGBA(255,0,0,200));
          SPHERE3D("representation:OpponentGoalModel", theOpponentGoalModel.shootASAPGoalTarget.translation.x(), theOpponentGoalModel.shootASAPGoalTarget.translation.y()+2, 10, theBallSpecification.radius, ColorRGBA(255,255,0,200));
          SPHERE3D("representation:OpponentGoalModel", theFieldDimensions.xPosOpponentGroundLine, theOpponentGoalModel.myGazeProjection,10,theBallSpecification.radius, ColorRGBA(0,255,255,127));
        }
        
      }
      if(thePlayerRole.role==PlayerRole::RoleType::striker || thePlayerRole.role==PlayerRole::RoleType::undefined || theOpponentGoalModel.graphicalDebug) LINE3D("representation:OpponentGoalModel", theFieldDimensions.xPosOpponentGroundLine - theOpponentGoalModel.goalTargetDistanceThreshold, theFieldDimensions.yPosLeftFieldBorder, 10, theFieldDimensions.xPosOpponentGroundLine - theOpponentGoalModel.goalTargetDistanceThreshold, theFieldDimensions.yPosRightFieldBorder, 10, 1, ColorRGBA(255,255,255));
    }
  }
  #endif
}