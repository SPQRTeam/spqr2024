/**
 * @file SearcherModelProvider.cpp (fork of GlobalFieldCoverageProvider.cpp)
 * @author Daniele Affinita
 */


#include "SearcherModelProvider.h"
using namespace std;

Vector2f fieldOfViewVector1, fieldOfViewVector2;

static void assignWeight(SearcherModel::Cell& c, float timestampComponent, float distanceComponent){
  if(c.type == SearcherModel::Cell::CellType::standard)
    c.searchWeight = timestampComponent*(distanceComponent+c.ballComponentValue);
      
  else if(c.type == SearcherModel::Cell::CellType::ballComponentSeenWeak ||
          c.type == SearcherModel::Cell::CellType::ballComponentSeenStrong)
    c.searchWeight = timestampComponent*distanceComponent;
      
  else
    c.searchWeight = 0;
}

float SearcherModelProvider::compute2dGaussianKernel(SearcherModel::Cell cell){ 

  auto getBestBall = [&](){
    if(theFieldBall.ballWasSeen(10000))
      return theFieldBall.endPositionOnField;
    else
      return theFieldBall.teamEndPositionOnField;
  };

  Vector2f maxPoint = getBestBall();
  double sigma = sigmaNormalizationRate * (theFrameInfo.time - theTeamBallModel.timeWhenLastSeen);
  sigma = std::max(0.0001, sigma); //avoid division by zero -> Nan
  double sigma2 = sigma*sigma;

  Vector2f translation = cell.positionOnField - maxPoint;
  double res = 1.f / (2 * M_PI * sigma2) * exp(-(translation.x() * translation.x() + translation.y() * translation.y()) / (2 * sigma2));

  double normalizedWeight = 2*gaussianMax * M_PI * sigma2;
  return res*normalizedWeight;
}

bool SearcherModelProvider::isInsideFieldOfView(SearcherModel::Cell cell){
  LocalPose2f cellPositionRelative = theLibMisc.glob2Rel(cell.positionOnField.x(), cell.positionOnField.y()).translation;

  if(cellPositionRelative.translation.x() < 0)
    return false;

  //the cell is too far
  if((theRobotPose.translation-cell.positionOnField).norm() > fieldOfViewFarTh)
    return false;
      
  int pointsInsideFov = 0;
  Vector2f cellPointVector;
  float fieldOfViewAngle1, fieldOfViewAngle2;

  for(int i = 0; i < 4 && pointsInsideFov < 3; ++i){
    cellPointVector = Vector2f(cell.polygon[i].x() - theRobotPose.translation.x(), cell.polygon[i].y() - theRobotPose.translation.y());
    fieldOfViewAngle1 = theLibMisc.angleBetweenVectors(fieldOfViewVector1, cellPointVector);
    fieldOfViewAngle2 = theLibMisc.angleBetweenVectors(fieldOfViewVector2, cellPointVector);
    pointsInsideFov += (fieldOfViewAngle1 * fieldOfViewAngle2) > 0 ? 0 : 1 ;
  }

  if(pointsInsideFov < 3){
    cellPointVector = Vector2f(cell.positionOnField.x() - theRobotPose.translation.x(), cell.positionOnField.y() - theRobotPose.translation.y());
    fieldOfViewAngle1 = theLibMisc.angleBetweenVectors(fieldOfViewVector1, cellPointVector);
    fieldOfViewAngle2 = theLibMisc.angleBetweenVectors(fieldOfViewVector2, cellPointVector);
    pointsInsideFov += fieldOfViewAngle1 * fieldOfViewAngle2 > 0 ? 0 : 1 ;
  }
  return pointsInsideFov >= 3;
}

void SearcherModelProvider::update(SearcherModel& SearcherModel){
  if(!initDone)
    init(SearcherModel);

  if(thePlayerRole.role == PlayerRole::goalie)
    return;

  if(thePlayerRole.current_context != thePlayerRole.search_for_ball){
    if(justSwitched){

      //Restore grid type since the current context is no more searching
      int gridSize = SearcherModel.grid.size();
      for(int j = 0; j < gridSize; ++j)
        SearcherModel.grid[j].type = SearcherModel::Cell::CellType::standard;
      justSwitched = false;
    }
    return;
  }

  justSwitched = true;
  
  if(theFrameInfo.getTimeSince(lastTimeUpdate) < updateTime)
    return;
  
  lastTimeUpdate = theFrameInfo.time;

  function<void(vector<SearcherModel::Cell>&)> updateWeightMatrix = [&](vector<SearcherModel::Cell>& matrix){
    int              cellIdx, 
                     r,
                     c,
                     zoneSize,
                     penalizedTeammates;
    float            maxDistance,
                     cellDistance,
                     timestampComponent,
                     distanceComponent;

    bool             cameraValid = false;

    Vector2f         cellPosition,
                     robotPoseVec;
    Vector2i         cellCoordinates;

    vector<Vector2f> fieldOfView;

    const RobotPose&       robotPose = static_cast<const RobotPose&>(theRobotPose);
    const CameraInfo&      cameraInfo = static_cast<const CameraInfo&>(theCameraInfo);
    const FieldDimensions& fieldDimensions = static_cast<const FieldDimensions&>(theFieldDimensions);
    const CameraMatrix&    cameraMatrix = static_cast<const CameraMatrix&>(theCameraMatrix);

    maxDistance = Vector2f(theFieldDimensions.xPosOpponentGroundLine - theFieldDimensions.xPosOwnGroundLine,
                           theFieldDimensions.yPosLeftSideline - theFieldDimensions.yPosRightSideline).norm();
    if(cameraMatrix.isValid)
      Projection::computeFieldOfViewInFieldCoordinates(robotPose, cameraMatrix, cameraInfo, fieldDimensions, fieldOfView);
    
    if(cameraMatrix.isValid){
      fieldOfViewVector1 = Vector2f(fieldOfView[2].x() - theRobotPose.translation.x(), fieldOfView[2].y() - theRobotPose.translation.y());
      fieldOfViewVector2 = Vector2f(fieldOfView[3].x() - theRobotPose.translation.x(), fieldOfView[3].y() - theRobotPose.translation.y());
      cameraValid = true;
    }

    // STAGE 0: other teammates zone
    penalizedTeammates = 0;
    for(Teammate teammate : theTeamData.teammates){
      if(teammate.isPenalized)
        ++penalizedTeammates;
      if(teammate.theRobotPose.translation.x() < theFieldDimensions.xPosOwnPenaltyArea+300) // TODO: OWN BOX becames OWN PENALTY is it correct????
        continue;

      robotPoseVec = Vector2f(teammate.theRobotPose.translation.x(), teammate.theRobotPose.translation.y());
      cellIdx = SearcherModel.getCellIndexFromPosition(robotPoseVec, theFieldDimensions);
      cellCoordinates = SearcherModel.flatToCoordinates(cellIdx);
      
      for(int row = cellCoordinates.x() - activeSearcherZone/2; row <= cellCoordinates.x() + activeSearcherZone/2; row++){
        for(int col = cellCoordinates.y() - activeSearcherZone/2; col <= cellCoordinates.y() + activeSearcherZone/2; col++){
          r = min(max(0, row), SearcherModel.numOfCellsX-1);
          c = min(max(0, col), SearcherModel.numOfCellsY-1);
          cellIdx = SearcherModel.coordinatesFlattening(r, c);
          matrix[cellIdx].searchWeight = 1; // 1 instead of 0 is a workaround in order to fix a shadow cones bug
          matrix[cellIdx].type = SearcherModel::Cell::CellType::searchedBySomeone;
          matrix[cellIdx].timestamp = theFrameInfo.time;
        }
      }
    }

    for(size_t i = 0; i < matrix.size(); ++i){
      if(matrix[i].type == SearcherModel::Cell::CellType::searchedBySomeone){
        matrix[i].type = SearcherModel::Cell::CellType::standard;
        continue;
      }

      else if(matrix[i].type == SearcherModel::Cell::CellType::ballComponentSeenWeak)
        matrix[i].type = SearcherModel::Cell::CellType::ballComponentSeenStrong;
      
      matrix[i].isShadow = false;

      // STAGE 0: guard covered zone

      if(penalizedTeammates < 2 &&
        SearcherModel.flatToCoordinates(i).x()*SearcherModel.cellLengthX < guardCoveredX_Th){
        matrix[i].searchWeight = 1;
        continue;
      }

      // STAGE 1: position based weights
      
      cellPosition = matrix[i].positionOnField;
      cellDistance = Vector2f(abs(theRobotPose.translation.x() - cellPosition.x()), abs(theRobotPose.translation.y() - cellPosition.y())).norm();

      distanceComponent = LocalPositionWeight*(1-cellDistance/maxDistance);
      
      // STAGE 2: last time seen based weight
      
      if(cameraValid){
        if(isInsideFieldOfView(matrix[i])){
          timestampComponent = 0;

          if(matrix[i].type == SearcherModel::Cell::CellType::standard)
            matrix[i].type = SearcherModel::Cell::CellType::ballComponentSeenWeak;
          
          matrix[i].timestamp = theFrameInfo.time;
        }
        else
          timestampComponent = min(1.f, LastTimeSeenWeight*(theFrameInfo.time - matrix[i].timestamp));
    
      }
      else
        timestampComponent = 1;
      
      // STAGE 3: gaussian distribution representing the ball position
      
      if(matrix[i].type == SearcherModel::Cell::CellType::standard){
        matrix[i].ballComponentValue = compute2dGaussianKernel(SearcherModel.grid[i]);
      }
      
      assignWeight(matrix[i],timestampComponent,distanceComponent);
    }

    // Underlying cell workaround to fix searcher stuck 
    int selfIdx = SearcherModel.getCellIndexFromPosition(theRobotPose.translation, theFieldDimensions);
    matrix[selfIdx].searchWeight = 1;
    matrix[selfIdx].type = SearcherModel::Cell::CellType::ballComponentSeenStrong;
    matrix[selfIdx].timestamp = theFrameInfo.time;

    // STAGE 4: shadow cones

    int idx,
        myIdx = SearcherModel.getCellIndexFromPosition(theRobotPose.translation, theFieldDimensions),
        current_x,
        current_y,
        current_flat,
        max_flat = matrix.size();
    Vector2i shadowDirection, 
             obsVec,
             myVec = SearcherModel.flatToCoordinates(myIdx);
    float rate;

    for(Obstacle obs : theObstacleModel.obstacles){

      idx = SearcherModel.getCellIndexFromPosition(obs.center, theFieldDimensions);

      if(matrix[idx].searchWeight == 0){
        obsVec = SearcherModel.flatToCoordinates(idx);
        
        shadowDirection = Vector2i(obsVec.x() - myVec.x(), obsVec.y() - myVec.y());
        current_x = obsVec.x();
        current_y = obsVec.y();
        current_flat = idx;
        if(shadowDirection.x() == 0)
          rate = 1e5;
        else 
          rate = (float)(shadowDirection.y())/(float)(shadowDirection.x());

        while(current_flat < max_flat && current_flat >= 0 &&
              matrix[current_flat].type != SearcherModel::Cell::CellType::searchedBySomeone &&
              matrix[current_flat].searchWeight == 0){

          //restore cell weight
          
          if(matrix[current_flat].type == SearcherModel::Cell::CellType::ballComponentSeenWeak)
            matrix[current_flat].type = SearcherModel::Cell::CellType::standard;
          
          cellPosition = matrix[current_flat].positionOnField;
          cellDistance = Vector2f(abs(theRobotPose.translation.x() - cellPosition.x()), abs(theRobotPose.translation.y() - cellPosition.y())).norm();

          distanceComponent = LocalPositionWeight*(1-cellDistance/maxDistance);

          assignWeight(matrix[current_flat], 1, distanceComponent);
          matrix[current_flat].timestamp = 0;
          matrix[current_flat].isShadow = true;

          if(abs(rate) <= 1){
            current_x = shadowDirection.x() > 0 ? current_x+1 : current_x-1;
            current_y = obsVec.y() + (int) ((current_x - obsVec.x())*rate);
            current_flat = SearcherModel.coordinatesFlattening(current_x, current_y);
          }
          else{
            current_y = shadowDirection.y() > 0 ? current_y+1 : current_y-1;
            current_x = obsVec.x() + (int) ((current_y - obsVec.y())/rate);
            current_flat = SearcherModel.coordinatesFlattening(current_x, current_y);
          }

        }
         
      } 
    } 
  };

  updateWeightMatrix(SearcherModel.grid);
}

void SearcherModelProvider::init(SearcherModel& SearcherModel)
{
  initDone = true;

  SearcherModel.cellLengthX = theFieldDimensions.xPosOpponentGroundLine * 2 / SearcherModel.numOfCellsX;
  SearcherModel.cellLengthY = theFieldDimensions.yPosLeftSideline * 2 / SearcherModel.numOfCellsY;

  float positionOnFieldX = theFieldDimensions.xPosOwnGroundLine + SearcherModel.cellLengthX / 2.f;
  float positionOnFieldY = theFieldDimensions.yPosRightSideline + SearcherModel.cellLengthY / 2.f;
  const unsigned time = max(10000u, theFrameInfo.time);
  lastTimeUpdate = time;
  SearcherModel.grid.reserve(SearcherModel.numOfCellsY * SearcherModel.numOfCellsX);
  for(int y = 0; y < SearcherModel.numOfCellsY; ++y)
  {
    for(int x = 0; x < SearcherModel.numOfCellsX; ++x)
    {
      SearcherModel.grid.emplace_back(time, time, 
                                            positionOnFieldX, positionOnFieldY,
                                            SearcherModel.cellLengthX,
                                            SearcherModel.cellLengthY, INFINITY);
      positionOnFieldX += SearcherModel.cellLengthX;
    }
    positionOnFieldX = theFieldDimensions.xPosOwnGroundLine + SearcherModel.cellLengthX / 2.f;
    positionOnFieldY += SearcherModel.cellLengthY;
  }

}

MAKE_MODULE(SearcherModelProvider, modeling);
