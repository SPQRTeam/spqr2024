/**
 * @file ObstacleModel.h
 *
 * Declaration of struct ObstacleModel.
 *
 * @author Florian Maaß
 */
#pragma once

#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Modeling/Obstacle.h"

// 0 is a separator
#define DISCOMOD_VER 4

// for the network
// the real ObstacleModel is below

typedef Eigen::Matrix<short, 2, 2> Matrix2short;
typedef Eigen::Matrix<short, 2, 1> Vector2short;

STREAMABLE(DiscretizedObstacle,
{,
  #if DISCOMOD_VER==0
  (Matrix2f)(Matrix2f::Zero()) covariance,
  (Vector2f)(Vector2f::Zero()) center,
  (Vector2f)(Vector2f::Zero()) left,
  (Vector2f)(Vector2f::Zero()) right,
  (Vector2f)(Vector2f::Zero()) velocity,
  #elif DISCOMOD_VER==1
  (Matrix2short)(Matrix2short::Zero()) covariance,
  (Vector2short)(Vector2short::Zero()) center,
  (Vector2short)(Vector2short::Zero()) left,
  (Vector2short)(Vector2short::Zero()) right,
  (Vector2short)(Vector2short::Zero()) velocity,
  #elif DISCOMOD_VER==101
  (Matrix2short)(Matrix2short::Zero()) covariance,
  (Vector2short)(Vector2short::Zero()) center,
  #elif DISCOMOD_VER==102
  (Vector2short)(Vector2short::Zero()) center,
  #elif DISCOMOD_VER==2
  (Matrix2short)(Matrix2short::Zero()) covariance,
  (int)(0) center,
  (int)(0) left,
  (int)(0) right,
  (int)(0) velocity,
  #elif DISCOMOD_VER==3
  (Matrix2f)(Matrix2f::Zero()) covariance,  // to send as 16 bits per float in teamMessage.def
  (Vector2short)(Vector2short::Zero()) center,
  #elif DISCOMOD_VER==4
  (Vector2f)(Vector2f::Zero()) center,
  #endif
  (unsigned int)(0) lastSeen,
  (Obstacle::Type) type,
});

STREAMABLE(DiscretizedObstacleModel, COMMA public BHumanMessageParticle<idCompressedObstacleModel>
{
  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override;

  DiscretizedObstacleModel() = default,

  (std::vector<DiscretizedObstacle>) obstacles,
});

/**
 * @struct ObstacleModel
 *
 * A struct that represents all kind of obstacles seen by vision, detected by arm contact,
 * foot bumper contact.
 */

STREAMABLE(ObstacleModel, COMMA public BHumanMessageParticle<idObstacleModel>
{
  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override;

  // torchipeppo 2023
  void operator<<(const DiscretizedObstacleModel& discomod);

  ObstacleModel() = default;
  void draw() const;
  void verify() const,

  (std::vector<Obstacle>) obstacles, /**< List of obstacles (position relative to own pose) */
});
