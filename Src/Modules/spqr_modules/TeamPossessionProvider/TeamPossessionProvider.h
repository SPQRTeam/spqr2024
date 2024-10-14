/**
 * @file TeamPossessionProvider.h 
 * @author Daniele Affinita
 */

#pragma once

#include "Representations/spqr_representations/TeamPossession.h"
#include "Representations/BehaviorControl/Libraries/LibObstacles.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Tools/Module/Module.h"


MODULE(TeamPossessionProvider,
{,
  REQUIRES(FieldBall),
  REQUIRES(LibObstacles),
  PROVIDES(TeamPossession),
  LOADS_PARAMETERS(
  {,
    (float) decaySpeed,
  }),
});

class TeamPossessionProvider : public TeamPossessionProviderBase
{
  private:
  float prevLaggyBalance = 0;
  float decay = 0;

  public:
    TeamPossessionProvider();
    void update(TeamPossession& teamPossession) override;
};
