#ifndef VERTEX_H
#define VERTEX_H

#include <array>

class HalfEdge;

class Vertex {

    public:

        Vertex(const std::array<double, 3>& loc);
        virtual ~Vertex();

        std::array<double, 3> position;

        HalfEdge* leaving = nullptr;

        void relocate(const std::array<double, 3>& loc);
        virtual void scale(double scalingFactor);



        // DEPRECATED METHOD
        Vertex(double radius, 
               double angle);
};

#endif