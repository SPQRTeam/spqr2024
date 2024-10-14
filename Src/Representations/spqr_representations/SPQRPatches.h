/**
 * @file ObstaclesFieldPercept.h
 *
 * This file declares a representation that lists the obstacles that were detected in
 * the current image in robot-relative field coordinates. Only obstacles the lower
 * end of which were visible are represented.
 *
 * @author Andre Mühlenbrock
 * @author Tim Laue
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Representations/Infrastructure/CameraImage.h"

STREAMABLE(SPQRPatch,
{
  ENUM(PatchType, 
  {,
     framePatroling,
      kalmanPrediction,
      lastFramePrediction,
      bottomObstacles,
  });
  SPQRPatch() = default;
  SPQRPatch(Vector2s offset, unsigned short width, unsigned short height, unsigned short minNeighbors, unsigned short minBallSize, unsigned short maxBallSize, bool isHighResolution, PatchType type);
  SPQRPatch(const CameraImage& image, Vector2s offset, unsigned short width, unsigned short height, PatchType type, unsigned short minNeighbors),
  (Vector2s) offset,
  (unsigned short) width,
  (unsigned short) height,
  (unsigned short) minNeighbors,
  (unsigned int) minBallSize,
  (unsigned int) maxBallSize,
  (bool) isHighResolution,
  (PatchType) type,
});

STREAMABLE(SPQRPatches,
{
  void draw() const,
  (std::vector<SPQRPatch>) patches, /**< All the obstacles found in the current image. */
});