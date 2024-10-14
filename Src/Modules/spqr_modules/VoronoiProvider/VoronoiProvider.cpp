/**
 * @file VoronoiProvider.cpp
 * @author Flavio Volpi, Valerio Spagnoli
 */

#include "VoronoiProvider.h"
#include <iostream>
#include <iterator>
#include <unordered_set>


// above this distance (between two opponents) the voronoi edge between the two oopponents is added to the graph
#ifndef MIN_DISTANCE_OPP
#define MIN_DISTANCE_OPP 750
#endif

#ifndef MAX_DISTANCE_CLUSTER
#define MAX_DISTANCE_CLUSTER 750
#endif

// Comparator for the graph ordering
bool NodeComparator(Voronoi::Node opp1, Voronoi::Node opp2){
    if(opp1.position.y() != opp2.position.y())
        return opp1.position.y() < opp2.position.y();
    else
        return opp1.position.x() < opp2.position.x();
}

// Comparator for edges ordering
struct VectorComparator
{
    bool operator() (Vector2f lhs, Vector2f rhs) const
    {
        return lhs.x() + lhs.y() < rhs.x() + rhs.y();
    }
};

/**
 * Return true if exists a free corridor between two points
 * @param start Start point
 * @param end End point
 * @param width Width of the corridor
 * @return True if the corridor is free
 */
bool VoronoiProvider::isCorridorFree(Vector2f start, Vector2f end, float width){
    for(auto robot: theTeamPlayersModel.obstacles){
        if( (robot.type==Obstacle::opponent || robot.type==Obstacle::fallenSomeRobot || robot.type==Obstacle::unknown) &&
          theLibSpec.isCorridorCovered(start, end, width, robot.center) ){
            return false;
          }
    }
    return true;
}

/**
 * Return true if v is in vector (linear search)
 * @param vector std::vector of Vector2f
 * @param v Element to check
 * @return True if v is in vector
 */
bool isInVector(std::vector<Vector2f> vector, Vector2f v){
    for(Vector2f e: vector){
        if(e == v)
            return true;
    }
    return false;
}

/**
 * Compute the Delaunay triangulation through Bowyer-Watson algorithm
 * Variations: 1) two super-triangles (instead of one)
 *             2) keep vertices of the super-triangles in final triangulation
 * @param pointList List of obstacles (opponent + some useful points)
 * @return List of Delaunay triangles
 */
std::vector<std::tuple<Vector2f, Vector2f, Vector2f>> VoronoiProvider::BowyerWatson(std::vector<Vector2f> pointList){

    std::vector<std::tuple<Vector2f, Vector2f, Vector2f>> triangulation;

    // Define two super triangles which contains the all field
    std::vector<Vector2f> super_triangle_vertex;
    super_triangle_vertex.push_back(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline));
    super_triangle_vertex.push_back(Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline));
    super_triangle_vertex.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightSideline));
    super_triangle_vertex.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline));

    std::tuple<Vector2f, Vector2f, Vector2f> super_triangle1 = std::make_tuple(super_triangle_vertex.at(0), super_triangle_vertex.at(2), super_triangle_vertex.at(3));
    std::tuple<Vector2f, Vector2f, Vector2f> super_triangle2 = std::make_tuple(super_triangle_vertex.at(0), super_triangle_vertex.at(1), super_triangle_vertex.at(3));
    
    triangulation.push_back(super_triangle1);
    triangulation.push_back(super_triangle2);
    
    for(auto opp: pointList){
        
        std::vector<std::tuple<Vector2f, Vector2f, Vector2f>> bad_triangles;
        for(auto triangle: triangulation){
            Geometry::Circle circumcenter = Geometry::getCircle(std::get<0>(triangle), std::get<1>(triangle), std::get<2>(triangle));
            if((opp-circumcenter.center).norm() < circumcenter.radius){
                bad_triangles.push_back(triangle);
            }
        }

    
        std::vector<std::tuple<Vector2f, Vector2f>> polygon;

        for(auto triangle: bad_triangles){
            Vector2f vertex0 = std::get<0>(triangle);
            Vector2f vertex1 = std::get<1>(triangle);
            Vector2f vertex2 = std::get<2>(triangle);

            std::vector<std::tuple<Vector2f, Vector2f>> edges;
            edges.push_back(std::make_tuple(vertex0, vertex1));
            edges.push_back(std::make_tuple(vertex0, vertex2));
            edges.push_back(std::make_tuple(vertex1, vertex2));

            for(auto edge:edges){
                bool shared = false;

                for(auto t: bad_triangles){
                    if(t!=triangle){    //t!=triangle
                        Vector2f v0 = std::get<0>(t);
                        Vector2f v1 = std::get<1>(t);
                        Vector2f v2 = std::get<2>(t);

                        std::vector<std::tuple<Vector2f, Vector2f>> es;
                        es.push_back(std::make_tuple(v0, v1));
                        es.push_back(std::make_tuple(v0, v2));
                        es.push_back(std::make_tuple(v1, v2));

                        for(auto e:es){                            
                            if(((std::get<0>(edge) == std::get<0>(e)) && (std::get<1>(edge) == std::get<1>(e))) || 
                            ((std::get<0>(edge) == std::get<1>(e)) && (std::get<1>(edge) == std::get<0>(e)))){
                                shared = true;
                                break;
                            }
                        }
                        if(shared)
                            break;
                    }
                }
            
                if(!shared){
                    polygon.push_back(edge);
                }
            }
        }
        
        for(auto triangle: bad_triangles){
            bool equal;

            int j = 0;
            while(j < triangulation.size()){
                std::tuple<Vector2f, Vector2f, Vector2f> t = triangulation.at(j);
                equal = ((std::get<0>(triangle) == std::get<0>(t)) || (std::get<0>(triangle) == std::get<1>(t)) || (std::get<0>(triangle) == std::get<2>(t))) &&
                        ((std::get<1>(triangle) == std::get<0>(t)) || (std::get<1>(triangle) == std::get<1>(t)) || (std::get<1>(triangle) == std::get<2>(t))) && 
                        ((std::get<2>(triangle) == std::get<0>(t)) || (std::get<2>(triangle) == std::get<1>(t)) || (std::get<2>(triangle) == std::get<2>(t)));
                if(equal){ 
                    triangulation.erase(triangulation.begin()+j); 
                    break;
                } 
                else{ ++j; }
            }
        }

        for(auto edge: polygon){
            std::tuple<Vector2f, Vector2f, Vector2f> new_triangle = std::make_tuple(opp, std::get<0>(edge), std::get<1>(edge));
            triangulation.push_back(new_triangle);        
        }

    }

    for(auto opp: pointList){
        int j = 0;
        while(j < triangulation.size()){
            std::tuple<Vector2f, Vector2f, Vector2f> triangle = triangulation.at(j);

            if(opp != std::get<0>(triangle) && opp != std::get<1>(triangle) && opp != std::get<2>(triangle)){
                Geometry::Circle circumcenter = Geometry::getCircle(std::get<0>(triangle), std::get<1>(triangle), std::get<2>(triangle));
                if((opp-circumcenter.center).norm() < circumcenter.radius){
                    triangulation.erase(triangulation.begin()+j); 
                }
                else{ 
                    ++j;
                }
            }
            else{
                ++j; 
            }
        }
    }

    return triangulation;
}

/**
 * Compute the Voronoi Diagram as dual graph of Delaunay triangulation
 * Variations: 1) two points closer than MAX_DISTANCE_CLUSTER are merged
 *             2) if an opponent is in the corridor (of width = MIN_DISTANCE_OPP) 
 *                of an edge, the edge is discarded
 * @param graph List of nodes of the graph
 * @param triangulation List of Delaunay triangles
 */
void VoronoiProvider::dualGraph(std::vector<Voronoi::Node>& graph, std::vector<std::tuple<Vector2f, Vector2f, Vector2f>> triangulation){
    
    // voronoi triangle = < center(vornoi point), vertex, vertex, vertex >
    std::vector<std::tuple<Vector2f, Vector2f, Vector2f, Vector2f>> voronoi_triangles; 
    for(auto triangle: triangulation){
        Vector2f vertex0 = std::get<0>(triangle);
        Vector2f vertex1 = std::get<1>(triangle);
        Vector2f vertex2 = std::get<2>(triangle);

        Geometry::Circle circumcenter = Geometry::getCircle(vertex0, vertex1, vertex2);
        Vector2f voronoi_vertex = circumcenter.center;

        // if a voronoi node is outside the field, is clipped inside it;
        if(voronoi_vertex.x() >= 4350 && voronoi_vertex.x() <= 5500)
            voronoi_vertex.x() = 4350;
        else if(voronoi_vertex.x() <= -4350 && voronoi_vertex.x() >= -5500)
            voronoi_vertex.x() = -4350;
        if(voronoi_vertex.y() >= 2850 && voronoi_vertex.y() <= 4000)
            voronoi_vertex.y() = 2850;
        else if(voronoi_vertex.y() <= -2850 && voronoi_vertex.y() >= -4000)
            voronoi_vertex.y() = -2850;

        // if a voronoi node is outside the bounds, is discarded
        if(voronoi_vertex.x() > 5500 || voronoi_vertex.x() < -5500 || voronoi_vertex.y() > 4000 || voronoi_vertex.y() < -4000) 
            continue;

        // necessary due to the super triangles
        if(voronoi_vertex == Vector2f(.0f, .0f)) continue;

        // if the new node is in conflict with one of the previous, they are merged
        bool conflict = false;
        for(auto vt : voronoi_triangles){
            if(Geometry::distance(std::get<0>(vt), voronoi_vertex)<MAX_DISTANCE_CLUSTER){
                conflict = true;
                voronoi_triangles.push_back(std::make_tuple(std::get<0>(vt), vertex0, vertex1, vertex2));
                break;
            }
        }
        if(!conflict)
            voronoi_triangles.push_back(std::make_tuple(voronoi_vertex, vertex0, vertex1, vertex2));


        LINE("module:VoronoiProvider:BowyerWatson", vertex0.x(), vertex0.y(), vertex1.x(), vertex1.y(), 50.0f, Drawings::PenStyle::solidPen, ColorRGBA::black);
        LINE("module:VoronoiProvider:BowyerWatson", vertex0.x(), vertex0.y(), vertex2.x(), vertex2.y(), 50.0f, Drawings::PenStyle::solidPen, ColorRGBA::black);
        LINE("module:VoronoiProvider:BowyerWatson", vertex1.x(), vertex1.y(), vertex2.x(), vertex2.y(), 50.0f, Drawings::PenStyle::solidPen, ColorRGBA::black);

        // CIRCLE("module:VoronoiProvider:BowyerWatson", voronoi_vertex.x(), voronoi_vertex.y(), circumcenter.radius, 25.0f, Drawings::PenStyle::solidPen, ColorRGBA::yellow, Drawings::BrushStyle::noBrush, ColorRGBA::yellow);
        
    }
    
    // for each triangle, it is linked with its adjacent triangles;
    // two triangles are adjacent if they have two vertices in common
    for(auto t1: voronoi_triangles){
        Vector2f c0 = std::get<0>(t1); 
        Vector2f v0 = std::get<1>(t1); 
        Vector2f v1 = std::get<2>(t1); 
        Vector2f v2 = std::get<3>(t1); 
    
        std::vector<Vector2f> edges;

        for(auto t2: voronoi_triangles){
            Vector2f c1 = std::get<0>(t2);
            Vector2f u0 = std::get<1>(t2);
            Vector2f u1 = std::get<2>(t2);
            Vector2f u2 = std::get<3>(t2);

            // trianlges are different if they have at least one different vertex
            if(v0!=u0 || v1!=u1 || v2!=u2){
                if(v0 == u0){
                    if(v1 == u1 || v1 == u2 || v2 == u1 || v2 == u2){
                        if(isCorridorFree(c0, c1, MIN_DISTANCE_OPP)){
                            edges.push_back(c1);
                        }
                    }
                }
                else if(v0 == u1){
                    if(v1 == u0 || v1 == u2 || v2 == u0 || v2 == u2){
                        if(isCorridorFree(c0, c1, MIN_DISTANCE_OPP)){
                            edges.push_back(c1);
                        }
                    }
                }
                else if(v0 == u2){
                    if(v1 == u0 || v1 == u1 || v2 == u0 || v2 == u1){
                        if(isCorridorFree(c0, c1, MIN_DISTANCE_OPP)){
                            edges.push_back(c1);
                        }
                    }
                }
                else if(v1 == u0){
                    if(v2 == u1 || v2 == u2){
                        if(isCorridorFree(c0, c1, MIN_DISTANCE_OPP)){
                            edges.push_back(c1);
                        }    
                    }
                }
                else if(v1 == u1){
                    if(v2 == u0 || v2 == u2){
                        if(isCorridorFree(c0, c1, MIN_DISTANCE_OPP)){
                            edges.push_back(c1);
                        }
                    }
                }
                else if(v1 == u2){
                    if(v2 == u0 || v2 == u1){
                        if(isCorridorFree(c0, c1, MIN_DISTANCE_OPP)){
                            edges.push_back(c1);
                        }
                    }
                }

            }
        }

        

        // if edges of a node is empty, or contains only the node itself, discard the node; 
        // otherwise push in graph
        if(edges.size()!=0 && (edges.size()!=1 || edges.at(0) != c0)){
            graph.push_back(Voronoi::Node(c0, edges));
        }
    }
    // merge the edges lists of duplicated nodes, and delete duplicated nodes
    std::sort(graph.begin(), graph.end(), NodeComparator);

    int k=0;
    while(k<graph.size()){
        int j=k;
        while(j<graph.size()){
            if(k!=j && graph[k].position == graph[j].position){
                for(Vector2f edge2: graph[j].edges){
                    if(!isInVector(graph[k].edges, edge2))
                        graph[k].edges.push_back(edge2);
                }
                graph.erase(graph.begin()+j);
            }
            else
                ++j;
        }
        
        // delete self link
        j=0;
        while(j<graph[k].edges.size()){
            if(graph[k].position == graph[k].edges[j]){
                graph[k].edges.erase(graph[k].edges.begin()+j);
            }
            else
                ++j;
        }
        if(graph[k].edges.size()==0){
            graph.erase(graph.begin()+k);
        }
        else{
            ++k;
        }
    }

    // alternative approach for duplicated nodes

    // int j = 0;
    // while(j+1<graph.size()){
    //     if(graph[j].position == graph[j+1].position){

    //         std::set<Vector2f, VectorComparator> s(std::make_move_iterator(graph[j].edges.begin()), std::make_move_iterator(graph[j].edges.end()));
    //         std::copy(graph[j+1].edges.begin(), graph[j+1].edges.end(), std::inserter(s, s.end()));
    //         graph[j].edges.clear();
    //         graph[j].edges.insert(graph[j].edges.end(), std::begin(s), std::end(s));
    //         graph.erase(graph.begin()+j);
    //     }
    //     else 
    //         ++j;
    // }
}



void VoronoiProvider::update(Voronoi& voronoi) {
    
    DECLARE_DEBUG_DRAWING("module:VoronoiProvider:BowyerWatson", "drawingOnField");

    std::vector<Voronoi::Node> graph;

    // create opponents list (only Vector2f)
    std::vector<Vector2f> opponents;
    for(auto obstacle: theTeamPlayersModel.obstacles){
        if((obstacle.type == Obstacle::opponent || obstacle.type == Obstacle::fallenOpponent)){    
            if(obstacle.center.x() <= 4500 && obstacle.center.x() >= -4500 && obstacle.center.y() <= 3000 && obstacle.center.y() >= -3000){
                opponents.push_back(obstacle.center);
                CIRCLE("module:VoronoiProvider:BowyerWatson", obstacle.center.x(), obstacle.center.y(), 80, 50, Drawings::PenStyle::solidPen, ColorRGBA::blue, Drawings::BrushStyle::solidBrush, ColorRGBA::blue);

            }
        }
    }   


    // side points of the middle line
    opponents.push_back(Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosLeftSideline));
    opponents.push_back(Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosRightSideline));
    // left and right goal
    opponents.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftGoal));
    opponents.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoal));

    /** Compute triangulation between obstacles through Bowyer-Watson algorithm;
     *  Voronoi graph is computed as the dual graph of Bowyer-Watson graph.
     *  Triangles are considered as tuple of Vector2f (which are the vertices)
    **/

    std::vector<std::tuple<Vector2f, Vector2f, Vector2f>> triangulation;
    triangulation = BowyerWatson(opponents);
    dualGraph(graph, triangulation);

    voronoi.graph = graph;
}

MAKE_MODULE(VoronoiProvider, modeling);

