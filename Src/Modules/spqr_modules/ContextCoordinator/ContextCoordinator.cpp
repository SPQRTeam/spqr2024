/**
* @file ContextCoordinator.cpp
*   This file implements the team coordination module
* @author Francesco Riccio, Emanuele Borzi
*/

#include "ContextCoordinator.h"

#include <unistd.h>
#include <iostream>
#include "Representations/SPQR-Libraries/ConfigFile/ConfigFile.h"
#include "Platform/BHAssert.h"
#include <fstream> // ofstream
#include <string>

#define NORM(x, y) sqrt(x*x + y*y)
//#define UM_VIEW

MAKE_MODULE(ContextCoordinator, behaviorControl);

ContextCoordinator::ContextCoordinator():
    goalie_pose(Pose2f()), defenderone_pose(Pose2f()), defendertwo_pose(Pose2f()), supporter_pose(Pose2f()),
    jolly_pose(Pose2f()), libero_pose(Pose2f()), striker_pose(Pose2f()), striker_nokickoff_pose(Pose2f()),
    tmp_role(PlayerRole::undefined), myRole(PlayerRole::undefined), role_hysteresis_cycle(0),
    startHysteresis(true)
{
    SPQR::ConfigurationParameters();
    //update config file
    configure();
    for(unsigned i = 0; i< thePlayerRole.robots_poses.size(); i++) // 7 robots
    {
        std::vector<float> init_uv(thePlayerRole.robots_poses.size(),0); // 7 roles
        utility_matrix.push_back(init_uv);

        mapped_robots.push_back(false);
        penalized_robots.push_back(fall_down_penalty);
    }

    // role_time.setToNow();
    // stamp.setToNow();
    role_time = Time::getRealSystemTime();
    stamp = Time::getRealSystemTime();

}

void ContextCoordinator::configure()                    // configura la posizione inziale dei giocatori
{
    goalie_pose = Pose2f(goalie_pose_t, goalie_pose_x, goalie_pose_y);
    goalie_pose.translation.x() = goalie_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
    goalie_pose.translation.y() = goalie_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
    
    defenderone_pose = theLibDefender.getDefenderonePosition();
    defendertwo_pose = theLibDefender.getDefendertwoPosition();
    jolly_pose = theLibJolly.getJollyPosition();
    supporter_pose = theLibSupporter.getSupporterPosition();
    libero_pose = theLibLibero.getLiberoPosition();

    striker_pose = theLibStriker.getStrikerPosition(true);
}

bool ContextCoordinator::isInCurrentContext(PlayerRole::RoleType currentRole)  // in base al numero del ruolo del giocatore torna true o false
{
    if (currentRole == PlayerRole::RoleType::none || currentRole == PlayerRole::RoleType::undefined)
        return false;
    //playing context
    if((thePlayerRole.current_context == PlayerRole::playing) && (((int) currentRole) <= PlayerRole::planJolly) && (((int) currentRole) >= PlayerRole::goalie ))
        return true;
    //search_for_ball context
    if(thePlayerRole.current_context == PlayerRole::search_for_ball &&
            ( ((int) currentRole >= PlayerRole::activeSearcher && (int) currentRole <= PlayerRole::passiveSearcher)))
        return true;

    return false;
}

float ContextCoordinator::getUtilityOrientationValue(Pose2f target, Pose2f robot) const   // ritorna l'angolo con cui sono orientato; il target è in locale e il robot in globale
{    
    Vector2f target_glob = theLibMisc.rel2Glob(target.translation.x(), target.translation.y()).translation;

    // Vector2f end1_local = Vector2f(1000.f, 0.f);
    // Vector2f end1_global = theLibMisc.rel2Glob(end1_local.x(), end1_local.y()).translation;
    // float angle = theLibMisc.angleBetweenGlobalVectors(robot.translation, end1_global, target_glob.translation);

    float angle = Geometry::angleTo(robot, target_glob);
    return (float)((M_PI - std::abs(angle))/M_PI);
}

float ContextCoordinator::getUtilityRoleHistory(int j) const
{
    //from the time since this particular role has been mapped   --- per quanto tempo ho fatto quel ruolo
    uint time;
    // time = (int)(PTracking::Timestamp() - role_time).getSeconds();    ------modificato
    time = (Time::getTimeSince(role_time) ); //theFrameInfo.time tempo attuale in millisecondi
    time = time / 1000;  //cosi torna in secondi
    if(tmp_role == j)
    {
        if(time*50 <= 500)
        {
            return time/10;
        }
        else
        {
            return 1;
        }
    }

    return 0;

}

 /**
 * - Controlla la matrice di utilità e decide se il giocatore è il migliore per quel ruolo c
 * - Controlla il caso in cui si ha gia quel ruolo
 */
bool ContextCoordinator::isMaxRoleScore(int c)     
{                                                
    unsigned int max_r = 0;
    for(unsigned int r=0; r<utility_matrix.size(); ++r)
    {
        if(mapped_robots.at(r))
            continue;
        if(utility_matrix.at(max_r).at(c-1) < utility_matrix.at(r).at(c-1) )
            max_r  = r;
    }


    if(max_r != 0)
    {
        mapped_robots.at(max_r) = true;
    }
    if( (int) max_r+1 == theRobotInfo.number)
    {
        return true;
    }
    return false;
}

bool ContextCoordinator::isRobotPenalized(int r)
{
    for(unsigned int t=0; t<theTeamData.teammates.size(); ++t)
    {
        if(theTeamData.teammates.at(t).number == r){
            return theTeamData.teammates.at(t).status != Teammate::PLAYING;
        }
    }

    return false;
}

inline bool ContextCoordinator::isRobotActive(int r, PlayerRole pr)
{
    Vector2f v = pr.robots_poses.at(r).translation;
    return v.x() != magic_value && v.y() != magic_value;
}

Vector2f ContextCoordinator::rel2Glob(float x, float y) const
{
    Vector2f result;
    float rho = (float)sqrt((x * x) + (y * y));

    result.x() = (float)(theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x))));
    result.y() = (float)(theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x))));

    return Vector2f(result.x(),result.y());
}

void ContextCoordinator::computePlayingRoleAssignment()     // assegno il ruolo durante il playing
{
    for(unsigned int r=0; r<utility_matrix.size(); ++r){
        mapped_robots.at(r) = false;
    }

    if( isMaxRoleScore(PlayerRole::getUtilityMatrixRowIndex(PlayerRole::striker)))
        tmp_role = PlayerRole::striker;
    else if( isMaxRoleScore(PlayerRole::getUtilityMatrixRowIndex(PlayerRole::defenderone))) 
        tmp_role = PlayerRole::defenderone;
    else if( isMaxRoleScore(PlayerRole::getUtilityMatrixRowIndex(PlayerRole::defendertwo))) 
        tmp_role = PlayerRole::defendertwo;
    else if( isMaxRoleScore(PlayerRole::getUtilityMatrixRowIndex(PlayerRole::supporter)))
        tmp_role = PlayerRole::supporter;
    else if( isMaxRoleScore(PlayerRole::getUtilityMatrixRowIndex(PlayerRole::jolly)))
        tmp_role = PlayerRole::jolly;
    else if( isMaxRoleScore(PlayerRole::getUtilityMatrixRowIndex(PlayerRole::libero)))
        tmp_role = PlayerRole::libero;
    else
        tmp_role = PlayerRole::undefined;
    
    
#ifdef DEEP_DEBUG_CONTEXT_COORD
    SPQR_INFO("tmp: "<<tmp_role); /// when no role has been selected the tmp role assigns the undefined enum.
    for(unsigned int r=0;r<mapped_robots.size(); ++r)
        SPQR_INFO("mapped robot "<<mapped_robots.at(r));
#endif
}

void ContextCoordinator::computeUtilityMatrix(const std::vector<Pose2f>& targets)
{
float teammate_on_the_ball = 0.2f;

//cycle on the rows (robots)
for(unsigned int r=1; r < thePlayerRole.robots_poses.size(); ++r) 
{
    //cycle on the columns (roles aka targets)
    for(unsigned int j=1; j<targets.size(); ++j)
    {
        
        if(j == 1)
            teammate_on_the_ball = 0.2f;
        else
            teammate_on_the_ball = 0.f;

        if(thePlayerRole.robots_poses.at(r).translation.x() == .0f) continue; //check
        int penalty = 0;
        if(isRobotPenalized(r+1)) penalty = 1;  //modificato r+1


        Pose2f robot_pose = theLibMisc.glob2Rel(thePlayerRole.robots_poses.at(r).translation.x(), thePlayerRole.robots_poses.at(r).translation.y());
        
        utility_matrix.at(r).at(j) = 
        ((1-penalty)*
        (
        + translation_weight * ((SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD - theLibMisc.distanceVec(targets.at(j).translation, robot_pose.translation))/SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD)
        + history_weight * getUtilityRoleHistory(j)  //default 1
        + target_orientation_weight * getUtilityOrientationValue(targets.at(j),thePlayerRole.robots_poses.at(r))  //orient respect the target
        + opp_goal_orientation_weight * getUtilityOrientationValue(Pose2f(theFieldDimensions.xPosOpponentGroundLine, 0.f),thePlayerRole.robots_poses.at(r)) //orient respect opponent goal
        ))/(translation_weight+history_weight+target_orientation_weight+opp_goal_orientation_weight);

    } 
}

}

// DANIELE CHECK
void ContextCoordinator::updateRobotRole(PlayerRole& role)
{
    /// WARNING: handle cases like: role.role == 0
    if( (role.role == PlayerRole::undefined && tmp_role != PlayerRole::undefined) || !isInCurrentContext(role.role) )
    {
        role.lastRole = role.role;
        role.role = tmp_role;
        myRole = tmp_role;
    }
    else
    {
        if(role.role != tmp_role && startHysteresis == true)
        {
            role_hysteresis_cycle = theFrameInfo.time;
            startHysteresis = false;
        }

        if ( role_hysteresis_cycle != 0 && theFrameInfo.getTimeSince(role_hysteresis_cycle) > time_hysteresis)
        {
            role.lastRole = role.role;
            role.role = tmp_role;
            myRole = tmp_role;
            role_hysteresis_cycle = 0;
            role_time = Time::getRealSystemTime();
            role_hysteresis_cycle = 0;
            startHysteresis =true;
        }
    }
}

void ContextCoordinator::updatePlayingRoleSpace(PlayerRole& role)
{
    if(theRobotInfo.number == 1)
    {
        role.role = PlayerRole::goalie;
        myRole = PlayerRole::goalie;
    }
    else
    {
        /// define targets: the order is crazy important
        // IMPORTANT: the targets are in LOCAL coordinates
        std::vector<Pose2f> targets;

        //1. Goalie
        targets.push_back(goalie_pose);

        //2. Striker
        // if(ballSeen){ //FLAVIO
        //     targets.push_back(Pose2f(theBallModel.estimate.position.x(),theBallModel.estimate.position.y()));
        // } else {
        //     targets.push_back(theLibMisc.glob2Rel(theTeamBallModel.position.x(), theTeamBallModel.position.y()));
        // }

        //2. Striker
        striker_pose = theLibMisc.glob2Rel(theLibStriker.getStrikerPosition(ballSeen).x(), theLibStriker.getStrikerPosition(ballSeen).y());
        targets.push_back(striker_pose);

        //3. Defender One
        defenderone_pose = theLibMisc.glob2Rel(theLibDefender.getDefenderonePosition().x(), theLibDefender.getDefenderonePosition().y());
        targets.push_back(defenderone_pose);

        //6. Defender Two
        defendertwo_pose = theLibMisc.glob2Rel(theLibDefender.getDefendertwoPosition().x(), theLibDefender.getDefendertwoPosition().y());
        targets.push_back(defendertwo_pose);

        //5. Supporter
        supporter_pose = theLibMisc.glob2Rel(theLibSupporter.getSupporterPosition().x(), theLibSupporter.getSupporterPosition().y()); //VINCENZO
        targets.push_back(supporter_pose);       //VINCENZO

        //4. Receiver
        jolly_pose = theLibMisc.glob2Rel(theLibJolly.getJollyPosition().x(), theLibJolly.getJollyPosition().y()); // TODO VINCENZO
        targets.push_back(jolly_pose);  

        //7. Libero
        libero_pose = theLibMisc.glob2Rel(theLibLibero.getLiberoPosition().x(), theLibLibero.getLiberoPosition().y());
        targets.push_back(libero_pose);



    #if defined(DEBUG_AND_WRITE_ROLE_PERSISTENCE)
        Pose2f posa_palla = Pose2f(rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
    #endif

        //compute the utility matrix
        computeUtilityMatrix(targets);

        //map the current configuration in the 'role space'
        computePlayingRoleAssignment();

        //update spqr robot role
        updateRobotRole(role);
    
        /*
        * PLOTTING
        */
    #if defined(DEBUG_AND_WRITE_ROLE_PERSISTENCE)
        Vector2f my_pos = Vector2f(
                    //theRole.robots_poses.at( theRobotInfo.number-1 ).translation.x(),
                    //theRole.robots_poses.at( theRobotInfo.number-1 ).translation.y()
                    theTeamData.teammates.at(theRobotInfo.number-1).theRobotPose.translation.x(),  // modificato sbagliato5
                    theTeamData.teammates.at(theRobotInfo.number-1).theRobotPose.translation.y()
                    );
        Vector2f my_role_pos;
        std::stringstream to_plot;
        switch(role.role)
        {
        case Role::goalie:
            my_role_pos = Vector2f( goalie_pose.translation.x(), goalie_pose.translation.y() );
            to_plot << "Goalie";
            break;
        case Role::defender:
            my_role_pos = Vector2f( defender_pose.translation.x(), defender_pose.translation.y() );
            to_plot << "Defender";
            break;
        case Role::jolly:
            my_role_pos = Vector2f( jolly_pose.translation.x(), jolly_pose.translation.y() );
            to_plot << "Jolly";
            break;
        case Role::supporter:
            my_role_pos = Vector2f( supporter_pose.translation.x(), supporter_pose.translation.y() );
            to_plot << "Supporter";
            break;
        case Role::striker:
            my_role_pos = Vector2f( posa_palla.translation.x(), posa_palla.translation.y() );
            to_plot << "Striker";
            break;
        case Role::undefined:
            my_role_pos = Vector2f( 0,0 );
            to_plot << "undefined";
            break;
        }

        if( role.role == Role::undefined )
            to_plot << " <> MY POSITION: " << my_pos.x << "," << my_role_pos.y <<
                       " <> MY ROLE POSITION: " << 0 << "," << 0 <<
                       " <> DISTANCE: " << 9000.0;
        else
            to_plot << " <> MY POSITION: " << my_pos.x << "," << my_pos.y <<
                       " <> MY ROLE POSITION: " << my_role_pos.x << "," << my_role_pos.y <<
                       " <> DISTANCE: " << (my_pos - my_role_pos).absFloat();

        std::ofstream myfile;
        std::stringstream ss;
        ss << "log_roles/log_roles_" << theRobotInfo.number << (theOwnTeamInfo.fieldPlayerColour == TEAM_BLUE ? "_(BLUE)" : "_(RED)" ) << ".txt";
        myfile.open(ss.str(),std::ofstream::ate | std::ofstream::app);
        myfile << to_plot.str() << "\n";
        myfile.close();
#endif

    }
}

void ContextCoordinator::updateRobotPoses(PlayerRole& role)
{
    role.robots_poses.at(theRobotInfo.number-1) = Pose2f(theRobotPose.rotation,theRobotPose.translation.x(),theRobotPose.translation.y()); // global
    if(!theTeamData.teammates.size()) return;

    for(unsigned int i=0; i<theTeamData.teammates.size(); ++i) // all global
    {
        Pose2f tmp_pose = Pose2f(theTeamData.teammates.at(i).theRobotPose.rotation,
                                 theTeamData.teammates.at(i).theRobotPose.translation.x(),
                                 theTeamData.teammates.at(i).theRobotPose.translation.y());

        role.robots_poses.at(theTeamData.teammates.at(i).number-1) = tmp_pose;
    }
}

void ContextCoordinator::update(PlayerRole& role)
{

    if(theGameInfo.state == STATE_PLAYING)
    {
        updateRobotPoses(role);

        unsigned timeWhenBallLastSeenByTeammate = 0;
        for (Teammate mate : theTeamData.teammates) {
            unsigned twls = mate.theBallModel.timeWhenLastSeen;
            if (twls > timeWhenBallLastSeenByTeammate) {
                timeWhenBallLastSeenByTeammate = twls;
            }
        }
        if ( (unsigned) theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < time_when_last_seen)
        {
            ballSeen=true;
        }
        else if (
            theTeamBallModel.isValid
            || (unsigned) theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastSeen) < recentSeenTime      // <-- [Francesco 2022] WORKAROUND in order to allow one searcher to notify ball seen
            || ((unsigned) theFrameInfo.getTimeSince(timeWhenBallLastSeenByTeammate) < recentSeenTime)   // <-- [Francesco 2023] idem
        )
        {
            teamBall=true;
            ballSeen=false;
        }else
        {
            ballSeen=false;
            teamBall=false;
        }

        if(!ballSeen && !teamBall)
        {
            prev_status = current_status;
            current_status = PlayerRole::search_for_ball;
        }
        else
        {
            prev_status = current_status;
            current_status = PlayerRole::playing;
        }
        role.current_context = current_status;

        if (theMessageManagement.outOfPackets) {
            updatePlayingFixedCoord();
        }
        else {
            fixed_coord_initialized = false;
        }

        /// playing context
        if(current_status == PlayerRole::playing){
            if (theMessageManagement.outOfPackets){
                assignPlayingFixedCoord(role);
            } 
            else {
                updatePlayingRoleSpace(role);
            }
        }

        /// search for ball context
        else if(current_status == PlayerRole::search_for_ball && (prev_status==PlayerRole::no_context || prev_status==PlayerRole::playing)){
            int fartherToOwnGoal = 0;

            for(unsigned int r=0; r < role.robots_poses.size(); ++r){
                if(!isRobotActive(r, role) || r == theRobotInfo.number-1)
                    continue;
                if(role.robots_poses.at(r).translation.x() > role.robots_poses.at(theRobotInfo.number-1).translation.x())
                    ++fartherToOwnGoal;
            }
            
            if(fartherToOwnGoal >= 2){
                role.role = PlayerRole::passiveSearcher;
                myRole = PlayerRole::passiveSearcher;
            }
            else{
                role.role = PlayerRole::activeSearcher;
                myRole = PlayerRole::activeSearcher;
            }
        }

        // assign keeper role   ----------------- Goal Keeper
        if(theRobotInfo.number == 1)
        {
            role.role = PlayerRole::goalie;
            myRole = PlayerRole::goalie;
        }
    }

#ifdef DEBUG_CONTEXT_COORD
    if(theGameInfo.state == STATE_PLAYING && theRobotInfo.number != 1)
        //        if(theGameInfo.state == STATE_PLAYING && (theRobotInfo.number == 2 || theRobotInfo.number == 5) )
    {
        // if ((PTracking::Timestamp() - stamp).getMs() > 3000)
        if ((Time::getTimeSince(stamp)) > 3000)    // modificato è gia in millisecondi
        {
            //for(unsigned int i=0; i<unexplored_clusters_centroids.size(); ++i)
            //    SPQR_INFO("Centroid "<<i<<": "<<unexplored_clusters_centroids.at(i).x<<", "<<
            //              unexplored_clusters_centroids.at(i).y);
            for(unsigned int i=0; i<unexplored_clusters_centroids.size(); ++i)  // modificato
                SPQR_INFO("Centroid "<<i<<": "<<unexplored_clusters_centroids.at(i).x<<", "<<
                          unexplored_clusters_centroids.at(i).y);
        }
    }
#endif

#ifdef UM_VIEW
    if(theGameInfo.state == STATE_PLAYING)
    {
        if( theRobotInfo.number != 1 )
        {
            // if ((PTracking::Timestamp() - stamp).getMs() > 3000)
            if ((Time::getTimeSince(stamp)) > 3000) // modificato
            {
                //std::cout<<"Context: "; CONTEXT(theRole.current_context);
                std::cerr<<"Spqr Role mapped to robot ["<<theRobotInfo.number<<"] =>  "<< (int) role.role<<std::endl;
                //SPQR_ROLE((int) role.role);
                for(unsigned int i=0; i<utility_matrix.size(); ++i)
                {
                    for(unsigned int j=0; j<utility_matrix.at(i).size(); ++j)
                    {
                        if(utility_matrix.at(i).at(j) >= 850) std::cout<<"\033[0;32;1m"<<"["<<utility_matrix.at(i).at(j)<<"]--- \033[0m";
                        else if(utility_matrix.at(i).at(j) < 100) std::cout<<"\033[22;31;1m"<<"["<<utility_matrix.at(i).at(j)<<"]--- \033[0m";
                        else std::cout<<"\033[22;34;1m"<<"["<<utility_matrix.at(i).at(j)<<"]--- \033[0m";
                    }

                    std::cout<<std::endl;
                }
                std::cout<<std::endl;

                // stamp.setToNow();
                //stamp.getCurrentSystemTime();    // modificato
                stamp = Time::getRealSystemTime();

            }
        }
    }
#endif
}




// FOR THE FIXED COORDINATION
// aux
int ContextCoordinator::find_lowest_number_with_a_given_role(PlayerRole::RoleType r) {
    for (int i=0; i<FIXED_COORD_MAX_PLAYER_NUMBER; i++) {
        if (fixed_roles[i] == r) {
            return i + 1;
        }
    }
    return -1;
}

/// aux
bool is_playing_role(PlayerRole::RoleType r) {
    return 
    r==PlayerRole::goalie || 
    r==PlayerRole::defenderone ||  
    r==PlayerRole::striker || 
    r==PlayerRole::jolly || 
    r==PlayerRole::supporter ||
    r==PlayerRole::defendertwo ||
    r==PlayerRole::libero;
}

/// aux
PlayerRole::RoleType get_default_role_by_number(int number) {
    // DANIELE CHECK
    switch (number) {
        case 1:  return PlayerRole::goalie;     break;
        case 2:  return PlayerRole::defenderone;   break;
        case 3:  return PlayerRole::defendertwo;    break;
        case 4:  return PlayerRole::jolly;      break;
        case 5:  return PlayerRole::supporter;  break;
        case 6:  return PlayerRole::striker; break;
        case 7:  return PlayerRole::libero; break;
        default: return PlayerRole::striker;    break;
    }
}

/// Defines simple deterministic rules for everyone to follow in the event of out-of-packets.
/// DOESN'T ASSIGN (YET).
/// This is b/c this should happen every timestep for everyone, even while in search mode,
/// so that all players keep their copies of fixed_roles in sync w/o communication.
void ContextCoordinator::updatePlayingFixedCoord() {
    ASSERT(theRobotInfo.number <= FIXED_COORD_MAX_PLAYER_NUMBER);
    
    // initialize based on the latest TeamData
    if (!fixed_coord_initialized) {
        fixed_coord_initialized = true;
        fixed_roles.clear();

        for (int i=0; i<FIXED_COORD_MAX_PLAYER_NUMBER; i++) {
            fixed_roles.push_back(PlayerRole::undefined);
        }

        // save only playing roles b/c this is only for the playing context
        if (is_playing_role(thePlayerRole.role)) {
            fixed_roles[theRobotInfo.number - 1] = thePlayerRole.role;
        }

        for (Teammate mate : theTeamData.teammates) {
            if (is_playing_role(mate.role)) {
                fixed_roles[mate.number - 1] = mate.role;
            }
        }
    }

    for (int i=0; i<FIXED_COORD_MAX_PLAYER_NUMBER; i++) {
        int number = i + 1;
        bool penalized = theOwnTeamInfo.players[i].penalty != PENALTY_NONE;

        // a penalized robot loses its role
        if (penalized) {
            fixed_roles[i] = PlayerRole::undefined;
        }

        // a non-penalized, undefined-role robot gets a static role
        if (!penalized && fixed_roles[i] == PlayerRole::undefined) {
            fixed_roles[i] = get_default_role_by_number(number);
        }
    }

    // if no striker...
    if (find_lowest_number_with_a_given_role(PlayerRole::striker) == -1) {
        int found_number;
        // first jolly becomes striker
        if ((found_number = find_lowest_number_with_a_given_role(PlayerRole::jolly)) != -1) {
            fixed_roles[found_number - 1] = PlayerRole::striker;
        }
        // else first supporter becomes striker
        else if ((found_number = find_lowest_number_with_a_given_role(PlayerRole::supporter)) != -1) {
            fixed_roles[found_number - 1] = PlayerRole::striker;
        }
        // else first defender becomes striker
        else if ((found_number = find_lowest_number_with_a_given_role(PlayerRole::defenderone)) != -1) {
            fixed_roles[found_number - 1] = PlayerRole::striker;
        }
        else if ((found_number = find_lowest_number_with_a_given_role(PlayerRole::defendertwo)) != -1) {
            fixed_roles[found_number - 1] = PlayerRole::striker;
        }
        else {
            // ummm
            // this is bad
            // whatever, first (non-goalie) available robot becomes striker
            for (int i=1; i<FIXED_COORD_MAX_PLAYER_NUMBER; i++) {
                if (theOwnTeamInfo.players[i].penalty != PENALTY_NONE) {
                    fixed_roles[i] = PlayerRole::striker;
                    break;
                }
            }
        }
        // if THIS fails, then we have no robot on the field to strike with!
    }
}

///Assigns the fixed role computed earlier.
void ContextCoordinator::assignPlayingFixedCoord(PlayerRole& role) {
    ASSERT(theRobotInfo.number <= FIXED_COORD_MAX_PLAYER_NUMBER);

    // fixed goalie
    if(theRobotInfo.number == 1)
    {
        role.role = PlayerRole::goalie;
        myRole = PlayerRole::goalie;
        return;
    }

    // assign the decided role, but never assign undefined (shouldn't happen, but better be safe)
    myRole = fixed_roles[theRobotInfo.number - 1];
    if (!is_playing_role(myRole)) {
        myRole = get_default_role_by_number(theRobotInfo.number);
    }
    role.role = myRole;
}
