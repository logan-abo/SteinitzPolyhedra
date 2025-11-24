#include "../Interfaces/CircleVertex.h"

#include <vector>
#include <cmath>
#include <array>

using std::vector;
using std::array;

CircleVertex::CircleVertex(array<double, 3> center,
                           double init_radius) : 
    Vertex(center),
    radius(init_radius) {}

CircleVertex::CircleVertex(array<double, 3> center) 
    : CircleVertex(center, 0.5) {}

CircleVertex::~CircleVertex() {}


double CircleVertex::getRadius() const {
    return radius;
}


void CircleVertex::scale(double scalar) {

    Vertex::scale(scalar);

    radius *= scalar;

}

