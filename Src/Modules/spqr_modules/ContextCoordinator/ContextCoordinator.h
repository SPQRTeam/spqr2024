#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/MessageManagement.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibDefender.h"
#include "Representations/BehaviorControl/Libraries/LibSupporter.h"
#include "Representations/BehaviorControl/Libraries/LibJolly.h"
#include "Representations/BehaviorControl/Libraries/LibStriker.h"
#include "Representations/BehaviorControl/Libraries/LibLibero.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Tools/Math/Geometry.h"

#define FIXED_COORD_MAX_PLAYER_NUMBER 7 //SIM


#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <mutex>

MODULE(ContextCoordinator, //TODO need to change Forget and Change as streamable and sand to all also the role toghether with the timestamp.. byebye
{,
 REQUIRES(GameInfo),
 REQUIRES(OpponentTeamInfo),
 REQUIRES(OwnTeamInfo),
 REQUIRES(RobotInfo),
 REQUIRES(RobotPose),
 REQUIRES(BallModel),
 REQUIRES(FrameInfo),
 REQUIRES(FallDownState),
 REQUIRES(TeamData),
 REQUIRES(TeamBallModel),
 REQUIRES(FieldDimensions),
 REQUIRES(ObstaclesFieldPercept),
 REQUIRES(LibMisc),
 REQUIRES(LibStriker),
 REQUIRES(LibDefender),
 REQUIRES(LibJolly),
 REQUIRES(LibLibero),
 REQUIRES(MessageManagement),
 REQUIRES(FieldBall),
 USES(LibSupporter),
 USES(PlayerRole),

 PROVIDES(PlayerRole),

// DANIELE CHECK
 LOADS_PARAMETERS(
{,
  (int) time_hysteresis,
  (int) fall_down_penalty,
  (float) target_orientation_weight,
  (float) opp_goal_orientation_weight,
  (float) translation_weight,
  (float) history_weight,
  (float) bias_weight,
  (float) goalie_pose_x,
  (float) goalie_pose_y,
  (float) goalie_pose_t,
  (float) defenderone_pose_x,
  (float) defenderone_pose_y,
  (float) defenderone_pose_t,
  (float) defendertwo_pose_x,
  (float) defendertwo_pose_y,
  (float) defendertwo_pose_t,
  (float) supporter_pose_x,
  (float) supporter_pose_y,
  (float) supporter_pose_t,
  (float) jolly_pose_x,
  (float) jolly_pose_y,
  (float) jolly_pose_t,
  (float) libero_pose_x,
  (float) libero_pose_y,
  (float) libero_pose_t,
  (float) striker_pose_x,
  (float) striker_nokickoff_pose_x,
  (float) striker_pose_y,
  (float) striker_pose_t,
  (float) threshold_striker,
  (unsigned int) time_when_last_seen,
  (unsigned) recentSeenTime,
 }),
       });

class ContextCoordinator: public ContextCoordinatorBase
{

private:

    /** Utilities */
    float norm(float x, float y){ return (float)sqrt(x*x + y*y); }
    float sign(float x){ if (x >= 0) return 1.f; else return -1.f; }
    Vector2f rel2Glob(float x, float y) const;
    unsigned last_utility_computation;
    bool ballSeen;
    bool teamBall;

public:
    unsigned stamp, role_time, context_time;
    bool flag = true;
    std::vector<Vector2f> explored_clusters_centroids;    // aggiunti per togliere spqrdwk
    std::vector<Vector2f> unexplored_clusters_centroids;

    Pose2f goalie_pose;
    Pose2f defenderone_pose;
    Pose2f defendertwo_pose;
    Pose2f supporter_pose;
    Pose2f jolly_pose;
    Pose2f libero_pose;
    Pose2f striker_pose;
    Pose2f striker_nokickoff_pose;

    std::vector<std::vector<float>> utility_matrix;
    PlayerRole::RoleType tmp_role, myRole;

    PlayerRole::Context prev_status = PlayerRole::no_context;
    PlayerRole::Context current_status = PlayerRole::no_context;


    int role_hysteresis_cycle;
    bool startHysteresis;
    std::vector<bool> mapped_robots;
    std::vector<int> penalized_robots;

    void configure();

    bool isInCurrentContext(PlayerRole::RoleType currentRole);
    float getUtilityOrientationValue(Pose2f target, Pose2f robot) const;
    float getUtilityRoleHistory(int j) const;
    bool isMaxRoleScore(int c);

    bool isRobotPenalized(int r);
    bool isRobotActive(int r, PlayerRole pr);
    void computePlayingRoleAssignment();
    void computeSearchRoleAssignment();

    void computeUtilityMatrix(const std::vector<Pose2f>& targets);

    void updateRobotRole(PlayerRole& role);
    void updateRobotPoses(PlayerRole& role);

    void updatePlayingRoleSpace(PlayerRole& role);

    ContextCoordinator();
    void update(PlayerRole& role);

    // FOR THE FIXED COORDINATION IN CASE OF OUT-OF-PACKETS
    bool fixed_coord_initialized = false;
    std::vector<PlayerRole::RoleType> fixed_roles;
    void updatePlayingFixedCoord();
    void assignPlayingFixedCoord(PlayerRole& role);
    int find_lowest_number_with_a_given_role(PlayerRole::RoleType r);
};
