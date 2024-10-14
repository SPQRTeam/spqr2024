/**
 * @file LibSearcherProvider.cpp
 * 
 * See LibSearcher
 *
 * @author Daniele Affinita
 */
 

#include "LibSearcherProvider.h"

MAKE_MODULE(LibSearcherProvider, behaviorControl);

void LibSearcherProvider::update(LibSearcher& libSearcher)
{
  libSearcher.getActiveSearcherPosition = [this]()->Vector2f{
    return getActiveSearcherPosition();
  };

}

Vector2f LibSearcherProvider::getActiveSearcherPosition() const{
  if(thePlayerRole.role == PlayerRole::goalie)
    return Vector2f();
  
  float max = 0;
  Vector2f maxPositionTmp;
  for(SearcherModel::Cell c : theSearcherModel.grid){
      if(c.searchWeight > max){
          max = c.searchWeight;
          maxPositionTmp = c.positionOnField;
      }
  }
  return maxPositionTmp;
};