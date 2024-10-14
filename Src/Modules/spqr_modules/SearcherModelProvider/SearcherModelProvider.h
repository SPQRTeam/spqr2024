/**
 * @file SearcherModelProvider.h (fork of GlobalFieldCoverageProvider.h)
 * @author Daniele Affinita
 */

#pragma once

#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/spqr_representations/SearcherModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Projection.h"

MODULE(SearcherModelProvider,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamData),
  REQUIRES(LibMisc),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(PlayerRole),
  REQUIRES(ObstacleModel),
  PROVIDES(SearcherModel),
  LOADS_PARAMETERS(
  {,
    (float) LocalPositionWeight,
    (float) LastTimeSeenWeight,
    (float) updateTime,
    (float) sigmaNormalizationRate,
    (float) gaussianMax,
    (float) fieldOfViewFarTh,
    (int) guardCoveredX_Th,
    (int) activeSearcherZone,
  }),
});

class SearcherModelProvider : public SearcherModelProviderBase
{
  private:
    unsigned int lastTimeUpdate;
    float angleBetweenVectors(Vector2f v1, Vector2f v2);
    float compute2dGaussianKernel(SearcherModel::Cell cell);
    bool isInsideFieldOfView(SearcherModel::Cell cell);
    bool isInsideShadowCone(SearcherModel::Cell cell);
  public:
    void update(SearcherModel& SearcherModel) override;

    bool initDone = false,
         justSwitched = false;
    void init(SearcherModel& SearcherModel);
};

