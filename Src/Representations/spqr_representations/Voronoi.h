/**
 * @file Voronoi.h
 * @author Flavio Volpi, Valerio Spagnoli
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include <vector>
#include <iostream>




STREAMABLE(Voronoi,
{
    void draw() const;
    void printGraph() const;

    STREAMABLE(Node,
    {
        Node() = default;
        Node(Vector2f position, std::vector<Vector2f> edges),

        (Vector2f) position,
        (std::vector<Vector2f>) edges,
    }),

    (std::vector<Node>) graph,
});
