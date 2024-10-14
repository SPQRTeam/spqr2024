#pragma once

#include <string.h>
#include "Representations/SPQR-Libraries/ConfigFile/ConfigFile.h"
#include <iostream>
#include <unistd.h>
#include <Platform/SystemCall.h>


// #define PARAM_WARN(x) std::cerr << "\033[0;33;1m" << x << "\033[0m" << std::endl;
#define PARAM_ERR(x) ::std::cerr << "\033[22;31;1m" << x << "\033[0m" << ::std::endl;

namespace SPQR
{

    static ::std::string IP_GOALIE;

    static float FIELD_DIMENSION_X,FIELD_DIMENSION_Y, PENALTY_AREA_X, PENALTY_AREA_Y, COORDINATION_PORT_NUMBER,
    PREDICTOR_PORT_NUMBER, MAXIMUM_DISTANCE_BALL_VIEWED, MAXIMUM_DISTANCE_ON_THE_FIELD, MINIMUM_PASSING_DISTANCE,
    GOALIE_LEARNING_STATE;

    static float  TURN_VALID_THS, TURN_EXCESS, WALKING_VELOCITY_X, DEFENDER_DEFAULT_POSITION_X,  DEFENDER_DEFAULT_POSITION_Y,
    SUPPORTER_DEFAULT_POSITION_X, SUPPORTER_DEFAULT_POSITION_Y, JOLLY_DEFAULT_POSITION_X, JOLLY_DEFAULT_POSITION_Y,
    STRIKER_KICKOFF_POSITION_X, STRIKER_KICKOFF_POSITION_Y, STRIKER_NO_KICKOFF_POSITION_X, STRIKER_NO_KICKOFF_POSITION_Y,
    GUARD_X_POSE, GUARD_Y_POSE, GOALIE_BASE_POSITION_X, GOALIE_BASE_POSITION_Y, GOALIE_POSE_X_TOLLERANCE, GOALIE_POSE_Y_TOLLERANCE,
    GOALIE_POSE_ANGLE_TOLLERANCE, GOALIE_POSE_X_TOLLERANCE_AFTER_DIVE, GOALIE_POSE_Y_TOLLERANCE_AFTER_DIVE,
    GOALIE_KICK_AWAY_RANGE, GOALIE_DIVE_RECOVER_TIME, GOALIE_DIVE_REPOSITION_TIME, GOALIE_CLOSE_DIVE_TIME,
    GOALIE_CLOSE_DIVE_RECOVER_TIME, GOALIE_STOP_BALL_TIME, GOALIE_STOP_BALL_RECOVER_TIME, GOALIE_DIVE_TIME_TOLERANCE,
    GOALIE_DIVE_TIME, GOALIE_MOVING_BALL_MIN_VELOCITY, GOALIE_EPSILON_COLLINEAR, GOALIE_FAR_LIMIT_Y, GOALIE_CLOSE_LIMIT_Y, GOALIE_MIN_BALL_DIST_FROM_POST, GOALIE_MAX_DIST_BALL_IN_RANGE_ABS,
    ALIGN_TO_GOAL_X, ALIGN_TO_GOAL_Y, ALIGN_BEHIND_BALL_X, ALIGN_BEHIND_BALL_Y, FORWARD_KICK_APPROACH_Y, SIDEKICK_APPROACH_X,
    SIDEKICK_APPROACH_Y, BACKKICK_ANGLE, BACKKICK_APPROACH_X, BACKKICK_APPROACH_Y, MIN_VELOCITY_THRESHOLD, GOALIE_KICK_AWAY_ANGLE;

    static unsigned int COORDINATION_INFORMATION_NETWORK_FREQUENCY, FALL_DOWN_PENALTY, TIME_TO_GET_UP, MOVING_BALL_MIN_VELOCITY,
    GOALIE_MIN_TIME_WHEN_LAST_SEEN;


    static void ConfigurationParameters()   //NB: this function must be called by every module that uses the SPQR namespace
    {
        ::std::string configDirectory;
        char currentWorkingDirectory[1024];

        configDirectory = "";

        if (SystemCall::getMode() == SystemCall::simulatedRobot)
        {
            if (getcwd(currentWorkingDirectory,1024)) {;}
            configDirectory = currentWorkingDirectory;
            configDirectory = configDirectory.substr(0,configDirectory.rfind("Config/")) + "Config/";
        }
        else configDirectory = "Config/";
        
        ::GMapping::ConfigFile fCfg;
        ::std::string key, section;

        section = "";

        //PARAM_WARN("Reading: '" << configDirectory + "configurationParameters.cfg'");
        if (!fCfg.read(configDirectory + ::std::string("configurationParameters.cfg")))
        {
            PARAM_ERR("Error reading file '" << configDirectory + "configurationParameters.cfg'. Exiting..."<< ::std::endl);
            //exit(-1);
        }

        try
        {
            section = "parameters";

            key = "IP_GOALIE";
            IP_GOALIE = (::std::string) fCfg.value(section,key);

            key = "FIELD_DIMENSION_X";
            FIELD_DIMENSION_X = fCfg.value(section,key);
            key = "FIELD_DIMENSION_Y";
            FIELD_DIMENSION_Y = fCfg.value(section,key);
            key = "PENALTY_AREA_X";
            PENALTY_AREA_X = (float) fCfg.value(section,key) * (float) FIELD_DIMENSION_X;
            key = "PENALTY_AREA_Y";
            PENALTY_AREA_Y = (float) fCfg.value(section,key) * (float) FIELD_DIMENSION_Y;

            key = "TURN_VALID_THS";
            TURN_VALID_THS = fCfg.value(section,key);
            key = "TURN_EXCESS";
            TURN_EXCESS = fCfg.value(section,key);
            key = "MAXIMUM_DISTANCE_BALL_VIEWED";
            MAXIMUM_DISTANCE_BALL_VIEWED = fCfg.value(section,key);
            key = "MAXIMUM_DISTANCE_ON_THE_FIELD";
            MAXIMUM_DISTANCE_ON_THE_FIELD = fCfg.value(section,key);
            key = "COORDINATION_INFORMATION_NETWORK_FREQUENCY";
            COORDINATION_INFORMATION_NETWORK_FREQUENCY = fCfg.value(section, key);
            key = "MOVING_BALL_MIN_VELOCITY";
            MOVING_BALL_MIN_VELOCITY = fCfg.value(section,key);
            key = "MINIMUM_PASSING_DISTANCE";
            MINIMUM_PASSING_DISTANCE = fCfg.value(section,key);

            key = "WALKING_VELOCITY_X";
            WALKING_VELOCITY_X = fCfg.value(section,key);

            key = "DEFENDER_DEFAULT_POSITION_X";
            DEFENDER_DEFAULT_POSITION_X = (float) fCfg.value(section,key) * (float) FIELD_DIMENSION_X;
            key = "DEFENDER_DEFAULT_POSITION_Y";
            DEFENDER_DEFAULT_POSITION_Y = (float) fCfg.value(section,key) * (float) FIELD_DIMENSION_Y;
            key = "SUPPORTER_DEFAULT_POSITION_X";
            SUPPORTER_DEFAULT_POSITION_X = (float) fCfg.value(section,key) * (float) FIELD_DIMENSION_X;
            key = "SUPPORTER_DEFAULT_POSITION_Y";
            SUPPORTER_DEFAULT_POSITION_Y = (float) fCfg.value(section,key) * (float) FIELD_DIMENSION_Y;
            key = "JOLLY_DEFAULT_POSITION_X";
            JOLLY_DEFAULT_POSITION_X = (float) fCfg.value(section,key) * (float) FIELD_DIMENSION_X;
            key = "JOLLY_DEFAULT_POSITION_Y";
            JOLLY_DEFAULT_POSITION_Y = (float) fCfg.value(section,key) * (float) FIELD_DIMENSION_Y;
            key = "STRIKER_KICKOFF_POSITION_X";
            STRIKER_KICKOFF_POSITION_X = fCfg.value(section,key);
            key = "STRIKER_KICKOFF_POSITION_Y";
            STRIKER_KICKOFF_POSITION_Y = fCfg.value(section,key);
            key = "STRIKER_NO_KICKOFF_POSITION_X";
            STRIKER_NO_KICKOFF_POSITION_X = fCfg.value(section,key);
            key = "STRIKER_NO_KICKOFF_POSITION_Y";
            STRIKER_NO_KICKOFF_POSITION_Y = fCfg.value(section,key);
            key = "GUARD_X_POSE";
            GUARD_X_POSE = fCfg.value(section,key);
            key = "GUARD_Y_POSE";
            GUARD_Y_POSE = fCfg.value(section,key);


            //goalie parameters
            key = "GOALIE_BASE_POSITION_X";
            GOALIE_BASE_POSITION_X = (int) fCfg.value(section,key) - (int) FIELD_DIMENSION_X;
            key = "GOALIE_BASE_POSITION_Y";
            GOALIE_BASE_POSITION_Y = fCfg.value(section,key);
            key = "GOALIE_POSE_X_TOLLERANCE";
            GOALIE_POSE_X_TOLLERANCE = fCfg.value(section,key);
            key = "GOALIE_POSE_Y_TOLLERANCE";
            GOALIE_POSE_Y_TOLLERANCE = fCfg.value(section,key);
            key = "GOALIE_POSE_ANGLE_TOLLERANCE";
            GOALIE_POSE_ANGLE_TOLLERANCE = fCfg.value(section,key);
            key = "GOALIE_POSE_X_TOLLERANCE_AFTER_DIVE";
            GOALIE_POSE_X_TOLLERANCE_AFTER_DIVE = fCfg.value(section,key);
            key = "GOALIE_POSE_Y_TOLLERANCE_AFTER_DIVE";
            GOALIE_POSE_Y_TOLLERANCE_AFTER_DIVE = fCfg.value(section,key);
            key = "GOALIE_KICK_AWAY_RANGE";
            GOALIE_KICK_AWAY_RANGE = fCfg.value(section,key);
            key = "GOALIE_LEARNING_STATE";
            GOALIE_LEARNING_STATE = fCfg.value(section,key);
            key = "GOALIE_DIVE_RECOVER_TIME";
            GOALIE_DIVE_RECOVER_TIME = fCfg.value(section,key);
            key = "GOALIE_DIVE_REPOSITION_TIME";
            GOALIE_DIVE_REPOSITION_TIME = fCfg.value(section,key);
            key = "GOALIE_CLOSE_DIVE_TIME";
            GOALIE_CLOSE_DIVE_TIME = fCfg.value(section,key);
            key = "GOALIE_CLOSE_DIVE_RECOVER_TIME";
            GOALIE_CLOSE_DIVE_RECOVER_TIME = fCfg.value(section,key);
            key = "GOALIE_STOP_BALL_TIME";
            GOALIE_STOP_BALL_TIME = fCfg.value(section,key);
            key = "GOALIE_STOP_BALL_RECOVER_TIME";
            GOALIE_STOP_BALL_RECOVER_TIME = fCfg.value(section,key);
            key = "GOALIE_DIVE_TIME_TOLERANCE";
            GOALIE_DIVE_TIME_TOLERANCE = fCfg.value(section,key);
            key = "GOALIE_DIVE_TIME";
            GOALIE_DIVE_TIME = fCfg.value(section,key);
            key = "GOALIE_MOVING_BALL_MIN_VELOCITY";
            GOALIE_MOVING_BALL_MIN_VELOCITY = fCfg.value(section,key);
            key = "GOALIE_EPSILON_COLLINEAR";
            GOALIE_EPSILON_COLLINEAR = fCfg.value(section,key);
            key = "GOALIE_FAR_LIMIT_Y";
            GOALIE_FAR_LIMIT_Y = fCfg.value(section,key);
            key = "GOALIE_CLOSE_LIMIT_Y";
            GOALIE_CLOSE_LIMIT_Y = fCfg.value(section,key);
            key = "GOALIE_MIN_TIME_WHEN_LAST_SEEN";
            GOALIE_MIN_TIME_WHEN_LAST_SEEN = fCfg.value(section,key);
            key = "GOALIE_MIN_BALL_DIST_FROM_POST";
            GOALIE_MIN_BALL_DIST_FROM_POST = fCfg.value(section,key);
            key = "GOALIE_MAX_DIST_BALL_IN_RANGE_ABS";
            GOALIE_MAX_DIST_BALL_IN_RANGE_ABS = fCfg.value(section,key);
            key = "MIN_VELOCITY_THRESHOLD";
            MIN_VELOCITY_THRESHOLD = fCfg.value(section,key);
            key = "GOALIE_KICK_AWAY_ANGLE";
            GOALIE_KICK_AWAY_ANGLE = fCfg.value(section,key);

            //striker parameters
            if (SystemCall::getMode() == SystemCall::simulatedRobot)  section = "Default";
            else
            {
                if (strcmp( SystemCall::getHostName(), "Spartaco") == 0 ) section = "Spartaco";
                else if (strcmp( SystemCall::getHostName(), "Pancrazio") == 0 ) section = "Pancrazio";
                else if (strcmp( SystemCall::getHostName(), "Romolo") == 0 ) section = "Romolo";
                else if (strcmp( SystemCall::getHostName(), "Quintiliano") == 0 ) section = "Quintiliano";
                else if (strcmp( SystemCall::getHostName(), "Decebalo") == 0 ) section = "Decebalo";
                else if (strcmp( SystemCall::getHostName(), "Bruto") == 0 ) section = "Bruto";
                else section = "Default";
            }

            key = "ALIGN_TO_GOAL_X";
            ALIGN_TO_GOAL_X = fCfg.value(section,key);
            key = "ALIGN_TO_GOAL_Y";
            ALIGN_TO_GOAL_Y = fCfg.value(section,key);
            key = "ALIGN_BEHIND_BALL_X";
            ALIGN_BEHIND_BALL_X = fCfg.value(section,key);
            key = "ALIGN_BEHIND_BALL_Y";
            ALIGN_BEHIND_BALL_Y = fCfg.value(section,key);
            key = "SIDEKICK_APPROACH_X";
            SIDEKICK_APPROACH_X = fCfg.value(section,key);
            key = "SIDEKICK_APPROACH_Y";
            SIDEKICK_APPROACH_Y = fCfg.value(section,key);
            key = "BACKKICK_ANGLE";
            BACKKICK_ANGLE = fCfg.value(section,key);
            key = "BACKKICK_APPROACH_X";
            BACKKICK_APPROACH_X = fCfg.value(section,key);
            key = "BACKKICK_APPROACH_Y";
            BACKKICK_APPROACH_Y = fCfg.value(section,key);

        }
        catch (...)
        {
            PARAM_ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << ::std::endl);
            //exit(-1);
        }
    }
}
