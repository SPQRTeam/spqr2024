/**
 * @file SearcherModel.h (fork of GlobalFieldCoverage.h)
 * @author Daniele Affinita
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include <vector>

STREAMABLE(SearcherModel,
{
  void draw() const;

  /**
   * Return the timestamp when the position was last seen.
   *
   * @return timestamp When the position was last seen
   */
  unsigned timeWhenLastSeen(const Vector2f& positionOnField, const FieldDimensions& theFieldDimensions) const;
  int getCellIndexFromPosition(const Vector2f position, const FieldDimensions& theFieldDimensions) const;
  int coordinatesFlattening(int x, int y) const;
  Vector2i flatToCoordinates(int n)const;

  STREAMABLE(Cell,
  {
    Cell() = default;
    Cell(const int coverage, unsigned timestamp,
         const float positionOnFieldX, const float positionOnFieldY,
         const float cellLengthX, const float cellLengthY, 
         const float searchWeight);

    ENUM(CellType,
    {,
      standard,
      ballComponentSeenWeak,
      ballComponentSeenStrong,
      searchedBySomeone,
    });

    const Vector2f polygon[4], /* Used for drawing */
    (int)(0) coverage, /* Rating for the cell */
    (unsigned)(0) timestamp, /* Timestamp when the cell was last seen */
    (Vector2f) positionOnField, /* Position of the cell in field coordinates */
    (float)(0.f) searchWeight, /* A weight used for compute the searching point */
    (float) (1) ballComponentValue, /* for optimization purposes */
    (CellType) (standard) type,
    (bool) (false) isShadow,
  }),

  (int)(0) meanCoverage,
  (std::vector<Cell>) grid,

  // The cell rate should be 3/2 (x/y)
  (int)(18) numOfCellsX, 
  (int)(12) numOfCellsY,
  (float) cellLengthX, /* Length of a cell in x direction */
  (float) cellLengthY, /* Length of a cell in y direction */
});
