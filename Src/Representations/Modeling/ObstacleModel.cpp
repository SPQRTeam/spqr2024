#include "ObstacleModel.h"
#include "Platform/SystemCall.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Approx.h"
#include "Tools/Modeling/Obstacle.h"
#include "Tools/Module/Blackboard.h"

#include <iostream>

void ObstacleModel::operator>>(BHumanMessage& m) const
{
  std::sort(const_cast<std::vector<Obstacle>&>(obstacles).begin(), const_cast<std::vector<Obstacle>&>(obstacles).end(), [](const Obstacle& a, const Obstacle& b) {return a.center.squaredNorm() < b.center.squaredNorm(); });

  Streaming::streamIt(*m.theBHumanStandardMessage.out, "theObstacleModel",  *this);
}

void ObstacleModel::operator<<(const BHumanMessage& m)
{
  Streaming::streamIt(*m.theBHumanStandardMessage.in, "theObstacleModel",  *this);
}

#define TWOSHORTS2INT_XMASK 0b11111111111111110000000000000000
#define TWOSHORTS2INT_YMASK 0b00000000000000001111111111111111
Vector2f demangle(int xy) {
  int x = xy & TWOSHORTS2INT_XMASK;
  int y = xy & TWOSHORTS2INT_YMASK;
  Vector2f v = Vector2f();
  v.x() = (float) x;
  v.y() = (float) y;
  return v;
}
void ObstacleModel::operator<<(const DiscretizedObstacleModel& discomod)
{
  obstacles.clear();
  obstacles.reserve(discomod.obstacles.size());
  for (const DiscretizedObstacle& disco : discomod.obstacles) {
    Obstacle obstacle;
    #if DISCOMOD_VER==0
    obstacle.center = disco.center;
    obstacle.covariance = disco.covariance;
    obstacle.left = disco.left;
    obstacle.right = disco.right;
    obstacle.velocity = disco.velocity;
    #elif DISCOMOD_VER==1
    obstacle.center = disco.center.cast<float>();
    obstacle.covariance = disco.covariance.cast<float>();
    obstacle.left = disco.left.cast<float>();
    obstacle.right = disco.right.cast<float>();
    obstacle.velocity = disco.velocity.cast<float>();
    #elif DISCOMOD_VER==101
    obstacle.covariance = disco.covariance.cast<float>();
    obstacle.center = disco.center.cast<float>();
    Vector2f offset = Vector2f();   // arbitrary
    offset.x() = 10.0f;
    offset.y() = 0.0f;
    obstacle.left = obstacle.center + offset;
    obstacle.right = obstacle.center - offset;
    obstacle.velocity = Vector2f::Zero();
    #elif DISCOMOD_VER==102
    obstacle.covariance = Matrix2f::Identity(2,2);
    obstacle.center = disco.center.cast<float>();
    Vector2f offset = Vector2f();   // arbitrary
    offset.x() = 10.0f;
    offset.y() = 0.0f;
    obstacle.left = obstacle.center + offset;
    obstacle.right = obstacle.center - offset;
    obstacle.velocity = Vector2f::Zero();
    #elif DISCOMOD_VER==2
    obstacle.covariance = disco.covariance.cast<float>();
    obstacle.center = demangle(disco.center);
    obstacle.left = demangle(disco.left);
    obstacle.right = demangle(disco.right);
    obstacle.velocity = demangle(disco.velocity);
    #elif DISCOMOD_VER==3
    obstacle.covariance = disco.covariance;
    obstacle.center = disco.center.cast<float>();
    Vector2f offset = Vector2f();   // arbitrary
    offset.x() = 10.0f;
    offset.y() = 0.0f;
    obstacle.left = obstacle.center + offset;
    obstacle.right = obstacle.center - offset;
    obstacle.velocity = Vector2f::Zero();
    #elif DISCOMOD_VER==4
    obstacle.covariance = Matrix2f::Identity(2,2) * 0.01;
    obstacle.center = disco.center;
    Vector2f offset = Vector2f();   // arbitrary
    offset.x() = 10.0f;
    offset.y() = 0.0f;
    obstacle.left = obstacle.center + offset;
    obstacle.right = obstacle.center - offset;
    obstacle.velocity = Vector2f::Zero();
    #endif
    obstacle.lastSeen = disco.lastSeen;
    obstacle.type = disco.type;
    obstacles.push_back(obstacle);
  }

  // OUTPUT_TEXT("AND THE RECEIVED SIZE IS dom-" << (int) discomod.obstacles.size() << " om-" << (int) obstacles.size());
  // std::cout << "AND THE RECEIVED SIZE IS dom-" << (int) discomod.obstacles.size() << " om-" << (int) obstacles.size() << std::endl;
  // if (obstacles.size() > 0) {
  //   OUTPUT_TEXT("AND THE FIRST CENTER IS " << (float) (obstacles[0].center.x()));
  // }
}

void DiscretizedObstacleModel::operator>>(BHumanMessage& m) const
{
  std::sort(
    const_cast<std::vector<DiscretizedObstacle>&>(obstacles).begin(),
    const_cast<std::vector<DiscretizedObstacle>&>(obstacles).end(),
    [](const DiscretizedObstacle& a, const DiscretizedObstacle& b) {return a.center.squaredNorm() < b.center.squaredNorm(); }
  );

  // unsigned size_before = m.theBHumanStandardMessage.sizeOfBHumanMessage();
  Streaming::streamIt(*m.theBHumanStandardMessage.out, "theDiscretizedObstacleModel",  *this);
  // unsigned size_after = m.theBHumanStandardMessage.sizeOfBHumanMessage();
  // OUTPUT_TEXT("obstacles-" << (int) obstacles.size() << " bytes-" << (size_after-size_before));
}

void DiscretizedObstacleModel::operator<<(const BHumanMessage& m)
{
  Streaming::streamIt(*m.theBHumanStandardMessage.in, "theDiscretizedObstacleModel",  *this);
}

void ObstacleModel::verify() const
{
  DECLARE_DEBUG_RESPONSE("representation:ObstacleModel:verify");
  for(const auto& obstacle : obstacles)
  {
    ASSERT(std::isfinite(obstacle.center.x()));
    ASSERT(std::isfinite(obstacle.center.y()));

    ASSERT(std::isfinite(obstacle.left.x()));
    ASSERT(std::isfinite(obstacle.left.y()));

    ASSERT(std::isfinite(obstacle.right.x()));
    ASSERT(std::isfinite(obstacle.right.y()));

    ASSERT(std::isfinite(obstacle.velocity.x()));
    ASSERT(std::isfinite(obstacle.velocity.y()));

    DEBUG_RESPONSE("representation:ObstacleModel:verify")
      if((obstacle.left - obstacle.right).squaredNorm() < sqr(2000.f))
        OUTPUT_WARNING("Obstacle too big!");

    ASSERT(std::isnormal(obstacle.covariance(0, 0)));
    ASSERT(std::isnormal(obstacle.covariance(1, 1)));
    ASSERT(std::isfinite(obstacle.covariance(0, 1)));
    ASSERT(std::isfinite(obstacle.covariance(1, 0)));
    ASSERT(Approx::isEqual(obstacle.covariance(0, 1), obstacle.covariance(1, 0), 1e-20f));
  }
}

void ObstacleModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:rectangle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:centerCross", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:leftRight", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:circle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:covariance", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:velocity", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:fallen", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:ObstacleModel", "robot");

  const ColorRGBA ownColor = ColorRGBA::fromTeamColor(Blackboard::getInstance().exists("OwnTeamInfo") ?
      static_cast<const OwnTeamInfo&>(Blackboard::getInstance()["OwnTeamInfo"]).fieldPlayerColour : TEAM_BLACK);

  const ColorRGBA opponentColor = ColorRGBA::fromTeamColor(Blackboard::getInstance().exists("OpponentTeamInfo") ?
      static_cast<const OpponentTeamInfo&>(Blackboard::getInstance()["OpponentTeamInfo"]).fieldPlayerColour : TEAM_RED);

  ColorRGBA color;
  for(const auto& obstacle : obstacles)
  {
    switch(obstacle.type)
    {
      case Obstacle::goalpost:
      {
        color = ColorRGBA::white;
        break;
      }
      case Obstacle::fallenTeammate:
      case Obstacle::teammate:
      {
        color = ownColor;
        break;
      }
      case Obstacle::fallenOpponent:
      case Obstacle::opponent:
      {
        color = opponentColor;
        break;
      }
      case Obstacle::fallenSomeRobot:
      case Obstacle::someRobot:
      {
        color = ColorRGBA(200, 200, 200); // gray
        break;
      }
      default:
      {
        color = ColorRGBA::violet;
        break;
      }
    }
    const Vector2f& center = obstacle.center;
    const Vector2f& left = obstacle.left;
    const Vector2f& right = obstacle.right;

    CYLINDER3D("representation:ObstacleModel", center.x(), center.y(), -210, 0, 0, 0, (left - right).norm(), 10, color);
    CROSS("representation:ObstacleModel:centerCross", center.x(), center.y(), Obstacle::getRobotDepth(), 10, Drawings::solidPen, color);

    float obstacleRadius = (left - right).norm() * .5f;
    Angle robotRotation = Blackboard::getInstance().exists("RobotPose") ? static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]).rotation : Angle();
    Vector2f frontRight(-Obstacle::getRobotDepth(), -obstacleRadius);
    frontRight = center + frontRight;
    RECTANGLE2("representation:ObstacleModel:rectangle", frontRight, obstacleRadius * 2, obstacleRadius * 2, -robotRotation, 16, Drawings::PenStyle::solidPen, ColorRGBA::black, Drawings::solidBrush, color);

    LINE("representation:ObstacleModel:leftRight", center.x(), center.y(), left.x(), left.y(), 20, Drawings::dottedPen, color);
    LINE("representation:ObstacleModel:leftRight", center.x(), center.y(), right.x(), right.y(), 20, Drawings::dottedPen, color);
    CIRCLE("representation:ObstacleModel:circle", center.x(), center.y(), obstacleRadius, 10, Drawings::dottedPen, color, Drawings::noBrush, color);
    COVARIANCE_ELLIPSES_2D("representation:ObstacleModel:covariance", obstacle.covariance, center);

    if(obstacle.velocity.squaredNorm() > 0)
      ARROW("representation:ObstacleModel:velocity", center.x(), center.y(),
            center.x() + 2 * obstacle.velocity.x(), center.y() + 2 * obstacle.velocity.y(), 10, Drawings::solidPen, ColorRGBA::black);

    if(obstacle.type >= Obstacle::fallenSomeRobot)
      DRAW_TEXT("representation:ObstacleModel:fallen", center.x(), center.y(), 100, color, "FALLEN");
  }
}
