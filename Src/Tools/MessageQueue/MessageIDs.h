/**
 * @file MessageIDs.h
 *
 * Declaration of ids for debug messages.
 *
 * @author Martin Lötzsch
 */

#pragma once

#include "Tools/Streams/Enum.h"

/**
 * IDs for debug messages
 *
 * To distinguish debug messages, they all have an id.
 */
ENUM(MessageID,
{,
  undefined,
  idFrameBegin,
  idFrameFinished,

  idActivationGraph,
  idAlternativeRobotPoseHypothesis,
  idAnnotation,
  idArmMotionRequest,
  idAudioData,
  idBallModel,
  idBallPercept,
  idBallSpots,
  idBehaviorStatus,
  idBodyContour,
  idCameraCalibration,
  idCameraImage,
  idCameraInfo,
  idCameraMatrix,
  idCirclePercept,
  idFallDownState,
  idFieldBoundary,
  idFootOffset,
  idFootSoleRotationCalibration,
  idFootSupport,
  idFrameInfo,
  idFsrSensorData,
  idGameInfo,
  idGlobalOptions,
  idGroundContactState,
  idGroundTruthOdometryData,
  idGroundTruthWorldState,
  idIMUCalibration,
  idImageCoordinateSystem,
  idInertialData,
  idInertialSensorData,
  idJointAngles,
  idJointCalibration,
  idJointLimits,
  idJointRequest,
  idJointSensorData,
  idJPEGImage,
  idKeyStates,
  idLabelImage,
  idLinesPercept,
  idMotionInfo,
  idMotionRequest,
  idObstacleModel,
  idCompressedObstacleModel,
  idObstaclesFieldPercept,
  idObstaclesImagePercept,
  idOdometer,
  idOdometryData,
  idOpponentTeamInfo,
  idOwnTeamInfo,
  idPenaltyMarkPercept,
  idRawGameInfo,
  idRefereeEstimator,
  idRobotDimensions,
  idRobotHealth,
  idRobotInfo,
  idRobotPose,
  idSelfLocalizationHypotheses,
  idSideInformation,
  idStopwatch,
  idSystemSensorData,
  idTeamActivationGraph,
  idTeamBallModel,
  idTeamBehaviorStatus,
  idTeamData,
  idTeamPlayersModel,
  idTeamTalk,
  idThumbnail,
  idWalkGenerator,
  idWalkStepData,
  idWalkingEngineOutput,
  idWalkLearner,
  idWhistle,
  numOfDataMessageIDs, /**< everything below this does not belong into log files */

  // infrastructure
  idRobot = numOfDataMessageIDs,
  idConsole,
  idDebugDataChangeRequest,
  idDebugDataResponse,
  idDebugDrawing,
  idDebugDrawing3D,
  idDebugImage,
  idDebugRequest,
  idDebugResponse,
  idDrawingManager,
  idDrawingManager3D,
  idLogResponse,
  idModuleRequest,
  idModuleTable,
  idPlot,
  idRobotname,
  idText,
  idTypeInfo,
  idTypeInfoRequest,

  // spqr
  idPlayerRole,
});
