#ifndef CIRCLEVERTEX_H
#define CIRCLEVERTEX_H

#include "../../DCEL/Interfaces/Vertex.h"

#include <vector>
#include <array>

using std::vector;
using std::array;

class CircleVertex : Vertex {

    private:


    public: 

        CircleVertex(array<double, 3> center);
        CircleVertex(array<double, 3> center, 
                     double radius);

        ~CircleVertex();


        double radius = 0.5;
        double getRadius() const;
        
        void scale(double scalar) override;
};

#endif