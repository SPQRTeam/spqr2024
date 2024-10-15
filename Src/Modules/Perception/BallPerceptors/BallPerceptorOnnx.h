/**
 * @file BallPerceptorOnnx.h
 *
 * This file declares a module that detects balls in images with a neural network.
 *
 * @author Bernd Poppinga
 * @author Felix Thielke
 * @author Gerrit Felsch
 */

#pragma once

#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Tools/ImageProcessing/PatchUtilities.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>

#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h> 
#include <sys/mman.h>
#include <onnxruntime_cxx_api.h>

MODULE(BallPerceptorOnnx,
{,
  REQUIRES(BallSpots),
  REQUIRES(BallSpecification),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(MotionInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBehaviorStatus),
  PROVIDES(BallPercept),
  LOADS_PARAMETERS(
  {,
    (std::string) encoderName, /**< The file name (relative to "NeuralNetworks/BallPerceptor") from which to load the model.  */
    (std::string) classifierName,
    (std::string) correctorName,
    (float) guessedThreshold, /**< Limit from which a ball is guessed. */
    (float) acceptThreshold, /**< Limit from which a ball is accepted. */
    (float) ensureThreshold, /**< Limit from which a ball is detected for sure. */
    (bool) useContrastNormalization,
    (float) ballAreaFactor,
    (float) contrastNormalizationPercent,
    (bool) useFloat,
    (PatchUtilities::ExtractionMode) extractionMode,
  }),
});

class BallPerceptorOnnx : public BallPerceptorOnnxBase
{
public:
  BallPerceptorOnnx();

private:

  Ort::Env env;
  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

  std::unique_ptr<Ort::Session> feature_extractor;
  std::unique_ptr<Ort::Session> classifier;
  std::unique_ptr<Ort::Session> detector;

  std::array<int64_t, 4> feature_extractor_input_shape{1,1,32,32};
  std::array<int64_t, 2> feature_extractor_output_shape{1,512};
  std::array<int64_t, 2> detector_output_shape{1,3};
  std::array<int64_t, 2> classifier_output_shape{1,1};

  void* shm_ptr;

  std::size_t patchSize = 0;

  void update(BallPercept& theBallPercept) override;
  float apply(const Vector2i& ballSpot, Vector2f& ballPosition, float& predRadius);
  void compile();
};