/**
 * @file BallPerceptorOnnx.cpp
 *
 * This file implements a module that detects balls in images with a neural network.
 *
 * @author Daniele Affinita
 */

#include "BallPerceptorOnnx.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/Global.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(BallPerceptorOnnx, perception);

BallPerceptorOnnx::BallPerceptorOnnx() {
  setup();
  shm_unlink(SHM_NAME);
  
  int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR | O_TRUNC, 0666);
  ASSERT(shm_fd != -1);
  ftruncate(shm_fd, 1);
  shm_ptr = mmap(nullptr, 1, PROT_WRITE, MAP_SHARED, shm_fd, 0);
  ASSERT(shm_ptr != MAP_FAILED);
}

void BallPerceptorOnnx::update(BallPercept& theBallPercept) {
  DECLARE_DEBUG_DRAWING("module:BallPerceptorOnnx:spots", "drawingOnImage");

  theBallPercept.status = BallPercept::notSeen;

  const auto& ballSpots = theBallSpots.ballSpots;
  if(ballSpots.empty()) return;

  float prob, bestProb = guessedThreshold;
  Vector2f ballPosition, bestBallPosition;
  float radius, bestRadius;
  for(std::size_t i = 0; i < ballSpots.size(); ++i) {
    prob = apply(ballSpots[i], ballPosition, radius);
    COMPLEX_DRAWING("module:BallPerceptorOnnx:spots") {
      std::stringstream ss;
      ss << i << ": " << static_cast<int>(prob * 100);
      DRAW_TEXT("module:BallPerceptorOnnx:spots", ballSpots[i].x(), ballSpots[i].y(), 15, ColorRGBA::red, ss.str());
    }
    if(prob > bestProb) {
      bestProb = prob;
      bestBallPosition = ballPosition;
      bestRadius = radius;
      if(SystemCall::getMode() == SystemCall::physicalRobot && prob >= ensureThreshold) break;
    }
  }

  if(bestProb > guessedThreshold) {
    theBallPercept.positionInImage = bestBallPosition;
    theBallPercept.radiusInImage = bestRadius;
    if(Transformation::imageToRobotHorizontalPlane(theImageCoordinateSystem.toCorrected(bestBallPosition), theBallSpecification.radius, theCameraMatrix, theCameraInfo, theBallPercept.positionOnField)) {
      theBallPercept.status = bestProb >= acceptThreshold ? BallPercept::seen : BallPercept::guessed;
      *((uint8_t*)shm_ptr) = theBallPercept.status == BallPercept::seen ? 1 : 0;
      return;
    }
  } else {
    *((uint8_t*)shm_ptr) = 0;
  }
}

float BallPerceptorOnnx::apply(const Vector2i& ballSpot, Vector2f& ballPosition, float& predRadius) {
  Vector2f relativePoint;
  Geometry::Circle ball;
  if(!(Transformation::imageToRobotHorizontalPlane(ballSpot.cast<float>(), theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePoint) &&
       Projection::calculateBallInImage(relativePoint, theCameraMatrix, theCameraInfo, theBallSpecification.radius, ball))) {
    return -1.f;
  }

  int ballArea = (static_cast<int>(ball.radius * ballAreaFactor) + 4) & ~3;
  RECTANGLE("module:BallPerceptorOnnx:spots", ballSpot.x() - ballArea / 2, ballSpot.y() - ballArea / 2, ballSpot.x() + ballArea / 2, ballSpot.y() + ballArea / 2, 2, Drawings::PenStyle::solidPen, ColorRGBA::black);

  const int inputSize = patchSize * patchSize;
  std::array<float, inputSize> encoder_input_data = {};
  std::array<float, 512> encoder_output_data = {};
  
  STOPWATCH("module:BallPerceptorOnnx:getImageSection")
  PatchUtilities::extractPatch(ballSpot, Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), theECImage.grayscaled, encoder_input_data.data(), extractionMode);

  const float stepSize = static_cast<float>(ballArea) / patchSize;

  feature_extractor->infer(encoder_input_data.data(), encoder_output_data.data());

  std::array<float, 1> classification_output_data = {};
  classifier->infer(encoder_output_data.data(), classification_output_data.data());

  float pred = classification_output_data[0];
  if(pred > guessedThreshold) {
    std::array<float, 3> detector_output_data = {};
    detector->infer(encoder_output_data.data(), detector_output_data.data());

    ballPosition.x() = (detector_output_data[0] - patchSize / 2) * stepSize + ballSpot.x();
    ballPosition.y() = (detector_output_data[1] - patchSize / 2) * stepSize + ballSpot.y();
    predRadius = detector_output_data[2] * stepSize;
  }

  return pred;
}

void BallPerceptorOnnx::setup() {
  const std::string baseDir = std::string(File::getBHDir()) + "/Config/NeuralNets/BallPerceptor/";
  env = Ort::Env{ORT_LOGGING_LEVEL_ERROR, "Default"};

  feature_extractor = new OnnxHelper<float, float>(baseDir + encoderName);
  classifier = new OnnxHelper<float, float>(baseDir + classifierName);
  detector = new OnnxHelper<float, float>(baseDir + correctorName);
}