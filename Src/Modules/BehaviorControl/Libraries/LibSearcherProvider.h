/**
 * @file LibSearcherProvider.h
 * 
 * See LibSearcher
 *
 * @author Daniele Affinita
 */


#pragma once

#include "Tools/Module/Module.h"
#include "Representations/spqr_representations/SearcherModel.h"
#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/Libraries/LibSearcher.h"

MODULE(LibSearcherProvider,
{,
  REQUIRES(PlayerRole),
  REQUIRES(SearcherModel),  
  PROVIDES(LibSearcher),
  // LOADS_PARAMETERS(
  // {,
  //   // nothing for now!
  // }),
});

class LibSearcherProvider : public LibSearcherProviderBase
{
private:
  
  /**
   * Updates LibSearcher
   * @param libSearcher The representation provided
   */
  void update(LibSearcher& LibSearcher) override;


  /** 
   * Provides the defender reference position based on the SearcherModel matrix
   */
  Vector2f getActiveSearcherPosition() const;
};
