#include "Representations/spqr_representations/SPQRPatches.h"
#include "Tools/Debugging/DebugDrawings.h"


void SPQRPatches::draw() const
{
  DEBUG_DRAWING("representation:ObstaclesImagePercept:image", "drawingOnImage")
  {
    //for(const auto& obstacle : patches)

     // RECTANGLE("representation:ObstaclesImagePercept:image", obstacle.left, obstacle.top, obstacle.right, obstacle.bottom, 4, Drawings::solidPen, ColorRGBA::white);
  }
}

SPQRPatch::SPQRPatch(Vector2s offset, unsigned short width, unsigned short height, unsigned short minNeighbors, unsigned short minBallSize, unsigned short maxBallSize, bool isHighResolution, PatchType type){
    offset = offset;
    width =  width;
    height = height;
    minNeighbors = minNeighbors;
    minBallSize = minBallSize;
    maxBallSize = maxBallSize;
    isHighResolution = isHighResolution;
    type = type;
}
SPQRPatch::SPQRPatch(const CameraImage& image, Vector2s offset, unsigned short width, unsigned short height, PatchType type, unsigned short minNeighbors){
    ASSERT(offset.x() >= 0 && offset.x() + width <= image.CV_image.cols);
    ASSERT(offset.y() >= 0 && offset.y() + height <= image.CV_image.rows);
    this->offset = offset;
    this->width =  width;
    this->height = height;
    this->minNeighbors = minNeighbors;
    this->type = type;

}