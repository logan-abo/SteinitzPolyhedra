#ifndef VERTEX_H
#define VERTEX_H

#include <array>

class HalfEdge;

class Vertex {

    public:

        Vertex();

        Vertex(const std::array<double, 3>& loc);
        Vertex(double radius, 
               double angle);

        ~Vertex();

        std::array<double, 3> position;
        HalfEdge* leaving = 0;
        double radius = 0.5;

        void relocate(const std::array<double, 3>& loc);
        void scale(double scalingFactor);

};

#endif