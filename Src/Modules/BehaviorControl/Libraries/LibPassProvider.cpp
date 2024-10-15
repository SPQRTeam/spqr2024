/**
 * @file LibPassProvider.cpp
 * 
 * See LibPass
 *
 * @author Francesco Petri
 */

#include "LibPassProvider.h"


MAKE_MODULE(LibPassProvider, behaviorControl);

void LibPassProvider::update(LibPass& libPass)
{
  DECLARE_DEBUG_DRAWING3D("module:LibPassProvider:passageModelBased", "field");

  libPass.findPassingLine = [this](Vector2f target, std::vector<Vector2f> mates) -> Vector2f {
    return findPassingLine(target, mates);
  };
  libPass.poseToPass = [this, &libPass]() -> Pose2f {
    return poseToPass(libPass);
  };
  libPass.strikerPassShare = [this]() -> std::tuple<int,int,Pose2f> {
    return strikerPassShare();
  };
  libPass.getPassUtility = [this](Vector2f position, int timeSLR) -> float {
    return getPassUtility(position, timeSLR);
  };
  libPass.getInversePassUtility = [this](Vector2f position) -> float {
    return getInversePassUtility(position);
  };
  libPass.getBestPassage = [this]() -> std::tuple<Vector2f, float> {
    return getBestPassage();
  };
  libPass.getBestPassageSpecial = [this]() -> std::tuple<Vector2f, float> {
    return getBestPassageSpecial();
  };
  libPass.getKickType = [this](Vector2f target) -> KickInfo::KickType {
    return getKickType(target);
  };

  // isTargetToPass provided by calls to poseToPass
}


bool LibPassProvider::findPassingLineAux(std::vector<Obstacle> opponents, Vector2f& target, const Vector2f& translation) {
  if(opponents.empty()) {
    target = translation;
    return true;
  }

  // compare against translation
  Eigen::ParametrizedLine<float,2> line = Eigen::ParametrizedLine<float,2>::Through(theRobotPose.translation, translation);

  // in case of failure
  std::vector<Obstacle> originalVector = opponents;

  for(auto it=opponents.begin(); it!=opponents.end(); ++it) {
    if(line.distance(it->center) > 400.f) {
      // call this with reduced obstacles
      opponents.erase(it);
      if(findPassingLineAux(opponents, target, translation)) {
        target = translation;
        return true;
      }
    }
    opponents = originalVector;
  }
  return false;
};

Vector2f LibPassProvider::findPassingLine(Vector2f target, std::vector<Vector2f> mates) {
  bool found = false;
  Vector2f originalTarget = target;
  Vector2f leftTarget = target + Vector2f(0,0);
  Vector2f rightTarget = target - Vector2f(0,0);

  // normalization factor for distance w.r.t. the receiver
  // used to avoid throwing the ball on the back of the receiver
  float nobacknormalization = 1;
  bool isThrowingOnTheBack = false;

  // only use opponents ahead
  // or if behind the opponent penalty mark also use those behind
  std::vector<Obstacle> opponents;
  for(const auto& obs : theTeamPlayersModel.obstacles) {
    if(obs.type == Obstacle::opponent) {
      // if theRobot is inside penalty area then push robots from 2000f to theRobot position
      // else push those ahead only
      if(theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark &&
        obs.center.x() < theRobotPose.translation.x() &&
        obs.center.x() > 2000.f) {
        opponents.push_back(obs);
      } else if(obs.center.x() > theRobotPose.translation.x() &&
        obs.center.x() < target.x()) {
        opponents.push_back(obs);
      }
    }
  }

  // now cycle over the merged obstacles
  while(!found &&
        leftTarget.y() < theFieldDimensions.yPosLeftSideline &&
        rightTarget.y() > theFieldDimensions.yPosRightSideline) {

    // check if we are throwing the ball on the back of the receiver
    Eigen::ParametrizedLine<float,2> toReceiver1 = Eigen::ParametrizedLine<float,2>::Through(theRobotPose.translation, leftTarget);
    Eigen::ParametrizedLine<float,2> toReceiver2 = Eigen::ParametrizedLine<float,2>::Through(theRobotPose.translation, rightTarget);
    for(Vector2f mate : mates) {
      // do not do this for back passes
      if(mate.x() < theRobotPose.translation.x()) {
        continue;
      }

      nobacknormalization = 1+(std::abs(mate.x() - theRobotPose.translation.x())/theFieldDimensions.xPosOpponentGroundLine);
      if(std::abs(toReceiver1.distance(mate)) < 200.f*nobacknormalization ||
              std::abs(toReceiver2.distance(mate)) < 200.f*nobacknormalization) {
        isThrowingOnTheBack = true;
        break;
      }
    }

    if(isThrowingOnTheBack) {
      // increase left and right
      isThrowingOnTheBack = false;
    } else {
      // compute the actual passage
      if(target.y() > theRobotPose.translation.y()) {
        if(findPassingLineAux(opponents, target, rightTarget) ||
          findPassingLineAux(opponents, target, leftTarget)) {
          return target;
        }
      } else {
        if(findPassingLineAux(opponents, target, leftTarget) ||
          findPassingLineAux(opponents, target, rightTarget)) {
          return target;
        }
      }
    }

    // increase left and right
    leftTarget = leftTarget + Vector2f(0,50);
    rightTarget = rightTarget - Vector2f(0,50);
  }
  // maybe next time
  return Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0);
}

Pose2f LibPassProvider::poseToPass(LibPass& libPass) {
  // custom team mate definition
  struct eMate {
    Vector2f position;
    float utility;
    int number;
  };

  // generate the list of valid teammates
  std::vector<eMate> orderedMates;
  // generate auxiliary list of vector2f of available mates
  std::vector<Vector2f> auxMates;
  // for rear pass do not consider forward passage, do this only for forward pass
  float passForward = 600.f;

  for(const auto& mate : theTeamData.teammates) {
    // enable rear passage if inside the opponent area
    // or if not robots have to be ahead of us
    if((theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark && mate.theRobotPose.translation.x() > 2000.f)
        || mate.theRobotPose.translation.x() > theRobotPose.translation.x()) {
      eMate emate;
      emate.position = mate.theRobotPose.translation;
      emate.utility = std::numeric_limits<float>::max();
      emate.number = mate.number;
      // std::cout<<"mate number "<<mate.number<<std::endl;
      // std::cout<<"mate role "<<mate.theTeamBehaviorStatus.role.getName()<<std::endl;   // QUESTA RIGA NON FUNZIONA SE TEAMBEHAVIORSTATUS NON È IN RETE (ma si può sistemare usando direttamente TypeRegistry::getEnumName su mate.role)
      // order w.r.t. distance from the closest opponent
      for(const auto& obs : theTeamPlayersModel.obstacles) {
        if(obs.type == Obstacle::opponent &&
            obs.center.x() <= mate.theRobotPose.translation.x()) {
          // compute emate utility w.r.t. obs
          float newUtility = (float)(std::pow(emate.position.x()-obs.center.x(),2) +
            std::pow(emate.position.y()-obs.center.y(),2));
          if(newUtility < emate.utility) {
            emate.utility = newUtility;
          }
        }
      }
      // order w.r.t utility
      bool added = false;
      if(orderedMates.empty()) {
        orderedMates.push_back(emate);
        auxMates.push_back(mate.theRobotPose.translation);
      } else {
        for(auto it = orderedMates.begin(); it != orderedMates.end(); ++it) {
          if(it->utility < emate.utility) {
            orderedMates.insert(it, emate);
            added = true;
            break;
          }
        }
      }
      if(!added) {
        // actual ordered list of mates
        // TODO remove to optimize
        orderedMates.push_back(emate);
        // use for the chek of robot backs (see find passing line)
        auxMates.push_back(mate.theRobotPose.translation);
      }
    }
  }

  // now call the findPassingLine starting from the best mate
  for(const auto& mate : orderedMates) {
    if(theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark) {
      passForward = 0.f;
    }
    Vector2f target = findPassingLine(mate.position, auxMates);
    // make forward pass
    if(mate.position.x() < theFieldDimensions.xPosOpponentPenaltyMark) {
      // increase the x
      target.x() = target.x()+passForward;
    }

    // if target is the center of the goal then no valid passing line, continue
    if(target != Vector2f(theFieldDimensions.xPosOpponentGroundLine+passForward,0.f)){
      libPass.isTargetToPass = mate.number;
      return target;
    }
  }
  return Vector2f(theFieldDimensions.xPosOpponentGroundLine,0.f);
};

std::tuple<int,int,Pose2f> LibPassProvider::strikerPassShare() const {
  for(const auto& teammate : theTeamData.teammates) {
    // [2022] cleared lots of commented stuff. Check spqrnao2022 if you REALLY need it.
    if(teammate.role == PlayerRole::RoleType::striker) {
      return std::tuple<int,int,Pose2f>(teammate.thePassShare.readyPass,teammate.thePassShare.passingTo,teammate.thePassShare.passTarget);
    }
  }
  return std::tuple<int,int,Pose2f>(0,0,theRobotPose);
};

float LibPassProvider::getInversePassUtility(Vector2f position) const {

  Vector2f globalBall = theFieldBall.recentBallPositionOnField();
  Vector2f opponentGoal(theFieldDimensions.xPosOpponentGoal, 0.f);
  float minDistanceGain = 1000.f;

  if((position-opponentGoal).norm() + minDistanceGain > (globalBall-opponentGoal).norm())
    return 0.f;

  Vector2f ballPosition = theFieldBall.recentBallPositionOnField();

  float norm = (position-ballPosition).norm();

  float bestPassageDistance = 3000.f; //TODO: move to cfg
  float passageVariance = 3000.f; //TODO: move to cfg
  float closestInterceptorThreshold = 1000.f; //TODO: move to cfg
  
  float utilityDistance = gaussianProbability(norm, passageVariance, bestPassageDistance); // TODO: stimare dai dati
  float normalizationFactor = gaussianProbability(bestPassageDistance, passageVariance, bestPassageDistance);
  
  utilityDistance /= normalizationFactor;
  
  Vector2f base = globalBall;
  Vector2f direction = position - globalBall;
  Geometry::Line line(base, direction);

  float closestInterceptor = INFINITY;
  
  for(const Obstacle& opp : theTeamPlayersModel.obstacles)
    if (opp.type == Obstacle::opponent) // TODO: considera se i teammates stanno in mezzo.
      closestInterceptor = std::min(closestInterceptor, Geometry::getDistanceToEdge(line, opp.center));
  
  float utilityClosestInterceptor = std::min(closestInterceptor, closestInterceptorThreshold) / closestInterceptorThreshold; 

  return utilityClosestInterceptor*utilityDistance;
}

float LibPassProvider::getPassUtility(Vector2f position, int timeSLR) const{

  float bestPassageDistance = 3000.f;
  float passageVariance = 3000.f;
  float closestInterceptorThreshold = 1000.f;
  float minDistanceGain = 1000.f;

  Vector2f globalBall = theFieldBall.recentBallPositionOnField();

  Vector2f localPosition = theLibMisc.glob2Rel(position.x(), position.y()).translation;

  Vector2f opponentGoal(theFieldDimensions.xPosOpponentGoal, 0.f);

  if ( (opponentGoal-position).norm() + minDistanceGain > (opponentGoal-theRobotPose.translation).norm() )
    return 0;
  float norm = localPosition.norm();

  float utilityDistance = gaussianProbability(norm, passageVariance, bestPassageDistance); // TODO: stimare dai dati
  float normalizationFactor = gaussianProbability(bestPassageDistance, passageVariance, bestPassageDistance);

  utilityDistance /= normalizationFactor; // between 0 and 1
  
  Vector2f base = globalBall;
  Vector2f direction = position - globalBall;
  Geometry::Line line(base, direction);

  float closestInterceptor = INFINITY;
  
  for(const Obstacle& opp : theTeamPlayersModel.obstacles)
    if (opp.type == Obstacle::opponent) // TODO: considera se i teammates stanno in mezzo.
      closestInterceptor = std::min(closestInterceptor, Geometry::getDistanceToEdge(line, opp.center));
  
  float utilityClosestInterceptor = std::min(closestInterceptor, closestInterceptorThreshold) / closestInterceptorThreshold; 

  // TODO: sarebbe bene usare le percezioni locali ... (come? boh)

  // taking into account how old the information is 
  float utilityUpdated = 1.f - mapToRange(static_cast<float>(timeSLR), 0.f, static_cast<float>(theMessageManagement.sendInterval), 0.f, 1.f);

  return utilityClosestInterceptor*utilityDistance*utilityUpdated;

};

ColorRGBA valueToRGB(float value) {
    float ratio = std::min(std::max(0.0f, value), 1.0f);

    int r = static_cast<int>(255 * (1.0f - ratio));
    int g = static_cast<int>(255 * ratio);
    int b = 0;

    return ColorRGBA(static_cast<unsigned char>(r), static_cast<unsigned char>(g), static_cast<unsigned char>(b));
}



std::tuple<Vector2f, float> LibPassProvider::getBestPassage() const{
  Vector2f best_position = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosCenterGoal);

  // TODO: move to cfg
  float killer_pass_distance = 400;

  float best_utility = 0.f;

  for(const Teammate& mate : theTeamData.teammates){
    int time_SLR = theFrameInfo.getTimeSince(mate.timeWhenLastPacketReceived);
    // Utility for a passage on the exact position of the teammate
    float utility_mate = LibPassProvider::getPassUtility(mate.theRobotPose.translation, time_SLR);
    // Utility for a killer pass (passaggio filtrante)
    Vector2f opponent_goal = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosCenterGoal); 
    Vector2f direction = (opponent_goal - mate.theRobotPose.translation);
    direction /= direction.norm();

    Vector2f position_killer = mate.theRobotPose.translation + direction*killer_pass_distance; 
    float utility_killer = LibPassProvider::getPassUtility(position_killer, time_SLR);

    Vector2f theBall = theFieldBall.recentBallPositionOnField();

    if(utility_killer > best_utility){
      best_utility = utility_killer;
      best_position = position_killer;
    } 
  }

  return std::tuple<Vector2f, float>(best_position, best_utility);
};

std::tuple<Vector2f, float> LibPassProvider::getBestPassageSpecial() const{
  Vector2f best_position = Vector2f::Zero();

  // TODO: move to cfg
  float killer_pass_distance = 300;

  float best_utility = -9999.f;

  for(const Teammate& mate : theTeamData.teammates){
    int time_SLR = theFrameInfo.getTimeSince(mate.timeWhenLastPacketReceived);
    // Utility for a passage on the exact position of the teammate
    float utility_mate = LibPassProvider::getPassUtility(mate.theRobotPose.translation, time_SLR);
    // Utility for a killer pass (passaggio filtrante)
    Vector2f opponent_goal = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosCenterGoal); 
    Vector2f direction = (opponent_goal - mate.theRobotPose.translation);
    direction /= direction.norm();

    Vector2f position_killer = mate.theRobotPose.translation + direction*killer_pass_distance; 
    float utility_killer = LibPassProvider::getPassUtility(position_killer, time_SLR);

    Vector2f theBall = theFieldBall.recentBallPositionOnField();  
    if(thePlayerRole.role == thePlayerRole.RoleType::striker){
      // POINT3D("module:LibPassProvider:passageModelBased", position_killer[0], position_killer[1], 2.f, 15.f, valueToRGB(utility_killer));
      CYLINDERARROW3D("module:LibPassProvider:passageModelBased", Vector3f(theBall.x(),theBall.y(),10), Vector3f(mate.theRobotPose.translation.x(), mate.theRobotPose.translation.y(), 10.f), 5.f, 65.f, 35.f, valueToRGB(utility_mate));
      CYLINDERARROW3D("module:LibPassProvider:passageModelBased", Vector3f(theBall.x(),theBall.y(),10), Vector3f(position_killer.x(), position_killer.y(), 10.f), 5.f, 65.f, 35.f, valueToRGB(utility_killer));
    }

    float utility = 0.f;
    Vector2f position = Vector2f::Zero();    
    (utility_mate > utility_killer) ? (utility = utility_mate, position = mate.theRobotPose.translation)
                                    : (utility = utility_killer, position = position_killer);



    if(utility > best_utility){
      best_utility = utility;
      best_position = position;
    } 
    
  }

  return std::tuple<Vector2f, float>(best_position, best_utility);
};

KickInfo::KickType LibPassProvider::getKickType(Vector2f target) const {
  
  /**  KICKTYPES:
   * * Walk Kick Types:
   * 
   * * KickType                                     | rotation [deg] | range [mm] | execution time [ms] | real range [mm] (2024)
   *   --------------------------------------------------------------------------------------------------------------------------
   *   walkForwardsRight/Left                       | 0              | 400-1000   | 600                 |
   *   walkForwardsRightLong/LeftLong               | 0              | 2500-3500  | 350                 |
   *   walkSidewardsRightFootToRight/LeftFootToLeft | 90/-90         | 2000       | 400                 |
   *   walkTurnRightFootToLeft/LeftFootToRight      | -45/45 deg     | 400-2100   | 600                 |
   *   walkForwardStealBallRight/Left               | -90/90 deg     | 200        | 0                   |
   *   walkForwardsRightAlternative/LeftAlternative | 3/-3 deg       | 2100       | 600                 |
   *  
   *   // walkForward
   *    // value || trials
   *    // ------||---------
   *    // 400   || 45
   *    // 500   || 54
   *    // 600   || 63
   *    // 700   || 86 | 76 70
   *    // 800   || 87 100 79 98
   *    // 900   || 80 97 86 94 | 145 90
   *    // 1000  || 95 115 106 123
   *
   *   // walkForwardLong
   *    // value || trials
   *    // ------||---------
   *    // 2500  || 270 165 170 200
   *    // 2600  || 240 200 180 260
   *    // 2700  || 280 300 250 275
   *    // 2800  || 210 230 200 150 280
   *    // 2900  || 270 200 120 120 120
   *    // 3000  || 240 140 150 200 
  */

  if(thePlayerRole.role == thePlayerRole.RoleType::striker){
    Vector2f inFrontOfRobot = theLibMisc.rel2Glob(1000, 0).translation;
    // CYLINDERARROW3D("module:LibPassProvider:passageModelBased", Vector3f(theRobotPose.translation.x(), theRobotPose.translation.y(), 10), Vector3f(inFrontOfRobot.x(), inFrontOfRobot.y(), 10), 5.f, 65.f, 35.f, ColorRGBA::black);
    // CYLINDERARROW3D("module:LibPassProvider:passageModelBased", Vector3f(theRobotPose.translation.x(), theRobotPose.translation.y(), 10), Vector3f(target.x(), target.y(), 10), 5.f, 65.f, 35.f, ColorRGBA::black);
  }

  float distance         = (theRobotPose.inversePose * target).norm();
  float angle            = std::abs(theLibMisc.radiansToDegree((theRobotPose.inversePose * target).angle()));  
  Vector2f ball_relative = theFieldBall.recentBallPositionRelative();

  // compute left or right foot
  bool left = false;
  if(ball_relative.y() > 0) left = true;

  // LONG_DISTANCES: 2000-3500
  // ! walkForwardsLeftLong/RightLong have a range of 2500-3500, so this condition is a bit forced
  if (distance>=2000 && distance<3500) {
    if (left) return KickInfo::KickType::walkForwardsLeftLong;
    else return KickInfo::KickType::walkForwardsRightLong;
  }

  // SHORT_DISTANCES: 400-2000
  // TODO: this should be splitted into 400-1000 (short) and 1000-2000 (mid), but for now we don't have a walkkick with range 1000-2000
  else{

    // Forward: 400-1000
    if (angle<=50){
      if (left) return KickInfo::KickType::walkForwardsLeft;
      else return KickInfo::KickType::walkForwardsRight;
    }

    // Forward-Turn: 400-2100
    else if (angle>50 && angle<=80 && ball_relative.norm() < 500){
      if (left) return KickInfo::KickType::walkTurnRightFootToLeft;
      else return KickInfo::KickType::walkTurnLeftFootToRight;
    }

    // Sideward: 2000
    else if (angle>80 && angle<=135 && ball_relative.norm() < 500){
      if (left) return KickInfo::KickType::walkSidewardsLeftFootToLeft;
      else return KickInfo::KickType::walkSidewardsRightFootToRight;
    }

    // Backward
    else {
      // TODO: Colpo di tacco !! :D
      if (left) return KickInfo::KickType::walkForwardsLeft;
      else return KickInfo::KickType::walkForwardsRight;
    }

  }

  // default
  if (left) return KickInfo::KickType::walkForwardsLeft;
  else return KickInfo::KickType::walkForwardsLeft;
};