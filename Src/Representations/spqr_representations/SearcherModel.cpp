/**
 * @file SearcherModel.cpp (fork of GlobalFieldCoverage.cpp)
 * @author Daniele Affinita
 */

#include "SearcherModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/BHMath.h"
#include <limits>

SearcherModel::Cell::Cell(const int coverage, unsigned timestamp,
                                const float positionOnFieldX, const float positionOnFieldY,
                                const float cellLengthX, const float cellLengthY, 
                                const float searchWeight)
  : coverage(coverage), timestamp(timestamp), positionOnField(positionOnFieldX, positionOnFieldY), searchWeight(searchWeight)
{
  const float xMin = positionOnFieldX - cellLengthX / 2.f;
  const float xMax = positionOnFieldX + cellLengthX / 2.f;
  const float yMin = positionOnFieldY - cellLengthY / 2.f;
  const float yMax = positionOnFieldY + cellLengthY / 2.f;

  const_cast<Vector2f&>(polygon[0]) = Vector2f(xMin + 25.0f, yMin + 25.0f);
  const_cast<Vector2f&>(polygon[1]) = Vector2f(xMin + 25.0f, yMax - 25.0f);
  const_cast<Vector2f&>(polygon[2]) = Vector2f(xMax - 25.0f, yMax - 25.0f);
  const_cast<Vector2f&>(polygon[3]) = Vector2f(xMax - 25.0f, yMin + 25.0f);
}

int SearcherModel::getCellIndexFromPosition(const Vector2f position, const FieldDimensions& theFieldDimensions) const{
  float clippedPositionX = clip(position.x(), theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.xPosOpponentGroundLine);
  float clippedPositionY = clip(position.y(), theFieldDimensions.yPosRightSideline, theFieldDimensions.yPosLeftSideline);

  const int x = std::min(static_cast<int>((clippedPositionX - theFieldDimensions.xPosOwnGroundLine) / cellLengthX), numOfCellsX - 1);
  const int y = std::min(static_cast<int>((clippedPositionY - theFieldDimensions.yPosRightSideline) / cellLengthY), numOfCellsY - 1);

  return coordinatesFlattening(x,y);
}

int SearcherModel::coordinatesFlattening(int x, int y) const{
  return y * numOfCellsX + x;
}

Vector2i SearcherModel::flatToCoordinates(int n) const{
  Vector2i out(n%numOfCellsX, static_cast<int>(floor(n/numOfCellsX)));
  return out;
}

unsigned SearcherModel::timeWhenLastSeen(const Vector2f& positionOnField, const FieldDimensions& theFieldDimensions) const
{
  float clippedPositionX = clip(positionOnField.x(), theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.xPosOpponentGroundLine);
  float clippedPositionY = clip(positionOnField.y(), theFieldDimensions.yPosRightSideline, theFieldDimensions.yPosLeftSideline);

  const int x = std::min(static_cast<int>((clippedPositionX - theFieldDimensions.xPosOwnGroundLine) / cellLengthX), numOfCellsX - 1);
  const int y = std::min(static_cast<int>((clippedPositionY - theFieldDimensions.yPosRightSideline) / cellLengthY), numOfCellsY - 1);

  return grid[y * numOfCellsY + x].timestamp;
}

ColorRGBA heatMapToRGB(float maximum, float value){
  float ratio = 2*value/maximum;
  int r,g,b;
  b = std::max(0, (int)(255*(1-ratio)));
  r = std::max(0, (int)(255*(ratio - 1)));
  g = 255 - b - r;
  return ColorRGBA((unsigned char)r,(unsigned char)g,(unsigned char)b);
}


void SearcherModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:SearcherModel:searchWeight", "drawingOnField");
  DEBUG_DRAWING("representation:SearcherModel:coverage", "drawingOnField")
  {
    float max = 0;
    for(size_t i = 0; i < grid.size(); ++i)
    {
      if(grid[i].searchWeight > max)
        max = grid[i].searchWeight;
    }

    for(size_t i = 0; i < grid.size(); ++i)
    {
      const Cell& c = grid[i];

      ColorRGBA color;

      color = heatMapToRGB(max, c.searchWeight);

      FILLED_RECTANGLE("representation:SearcherModel:coverage",
                 c.polygon[1].x(), c.polygon[1].y(),
                 c.polygon[3].x(), c.polygon[3].y(),
                20, Drawings::solidPen, color, Drawings::solidBrush, color);

      DRAW_TEXT("representation:SearcherModel:searchWeight", c.polygon[0].x(), c.polygon[0].y(), 50, ColorRGBA(255, 255, 255, 255), c.searchWeight);
    }
  }
}
