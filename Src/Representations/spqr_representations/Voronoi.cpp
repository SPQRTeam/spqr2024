/**
 * @file Voronoi.cpp
 * @author Flavio Volpi, Valerio Spagnoli
 */

#include "Voronoi.h"
#include "Tools/Debugging/DebugDrawings.h"


Voronoi::Node::Node(Vector2f position, std::vector<Vector2f> edges) : position(position), edges(edges)
{

}

void Voronoi::draw() const 
{
    //DECLARE_DEBUG_DRAWING("representation:Voronoi:graph", "drawingOnField");
    DEBUG_DRAWING("representation:Voronoi:graph", "drawingOnField")
    {
        for(auto node: graph){
            CIRCLE("representation:Voronoi:graph", node.position.x(), node.position.y(), 80, 50, Drawings::PenStyle::solidPen, ColorRGBA::orange, Drawings::BrushStyle::solidBrush, ColorRGBA::orange);
            DRAW_TEXT("representation:Voronoi:graph", node.position.x(), node.position.y(), 100, ColorRGBA::black, "x: "<<node.position.x()<<"  y: "<<node.position.y());

            for(Vector2f edge: node.edges){
                LINE("representation:Voronoi:graph", node.position.x(), node.position.y(), edge.x(), edge.y(), 50.0f, Drawings::PenStyle::solidPen, ColorRGBA::red);

            }
        }
    }
  
}

void Voronoi::printGraph() const
{
    for(int i=0; i< graph.size(); ++i){
        std::cout << "position:     x = "<< graph[i].position.x() << "    y: " << graph[i].position.y() <<std::endl;
        for(auto edge: graph[i].edges){
            std::cout << "--------------- edge:     x = "<< edge.x() << "    y: " << edge.y() <<std::endl;
        
        }
    }    
}