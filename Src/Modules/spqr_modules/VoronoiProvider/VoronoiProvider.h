/**
 * @file VoronoiProvider.h
 * @author Flavio Volpi, Valerio Spagnoli
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/spqr_representations/Voronoi.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibMisc.h"
#include "Representations/BehaviorControl/Libraries/LibSpec.h"
#include "Tools/Math/Geometry.h"
#include <vector>
#include <list>
#include <tuple>
#include <map>
#include <set>
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"

using Line2 = Eigen::Hyperplane<float,2>;


MODULE(VoronoiProvider,
{,
    REQUIRES(TeamPlayersModel),
    REQUIRES(FieldDimensions),
    REQUIRES(RobotInfo),
    REQUIRES(LibMisc),
    REQUIRES(LibSpec),
    PROVIDES(Voronoi),

});

class VoronoiProvider : public VoronoiProviderBase
{
    private:
        std::vector<std::tuple<Vector2f, Vector2f, Vector2f>> BowyerWatson(std::vector<Vector2f> pointList);
        void dualGraph(std::vector<Voronoi::Node>& graph, std::vector<std::tuple<Vector2f, Vector2f, Vector2f>> triangulation);
        bool isCorridorFree(Vector2f start, Vector2f end, float width);

        
    public:
        void update(Voronoi& voronoi);
        //void printGraph();
};



