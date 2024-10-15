#pragma once

#include "Tools/Module/Module.h"
#include "Representations/spqr_representations/RoleAndContext.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Modeling/TeamBallModel.h"


MODULE(RoleAndContextProvider, 
{,
  REQUIRES(PlayerRole),
  REQUIRES(BallModel),
  REQUIRES(ObstacleModel),
  REQUIRES(LibMisc),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(OwnTeamInfo),
  USES(TeamPlayersModel),
  USES(TeamBallModel),
  USES(TeamData),
  PROVIDES(RoleAndContext),
});

class RoleAndContextProvider : public RoleAndContextProviderBase
{
private:

public:
    std::tuple<float,int> minOpponent_BallDistance();
    std::tuple<float,int, int> minTeammate_BallDistance();
    std::tuple<int,int,int,int> score_andActivePlayers();

    void update(RoleAndContext& rac);
    RoleAndContextProvider();
};
