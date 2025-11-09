#include "../Interfaces/HalfEdge.h"
#include "../Interfaces/Vertex.h"
#include "../Interfaces/Face.h"

#include <array>
#include <cmath>

using std::array;
using std::sqrt;
using std::pow;

HalfEdge::HalfEdge() :
    origin(nullptr),
    twin(nullptr), 
    next(nullptr), 
    face(nullptr) {}

double HalfEdge::length() {

    array<double, 3> v1 = this->origin->position;
    array<double, 3> v2 = this->twin->origin->position;

    return sqrt( pow(v1[0]-v2[0], 2) + pow(v1[1]-v2[1], 2) );

}

double slope(Vertex* v1, Vertex* v2) {

    return ((v1->position[1]-v2->position[1])/(v1->position[0]-v2->position[0]));

}

double HalfEdge::angleWith(HalfEdge* edge) {

    std::array<double, 2> v1 = {
        edge->twin->origin->position[0] - edge->origin->position[0],
        edge->twin->origin->position[1] - edge->origin->position[1]
    };
    std::array<double, 2> v2 = {
        twin->origin->position[0] - origin->position[0],
        twin->origin->position[1] - origin->position[1]
    };

    double dot = v1[0]*v2[0] + v1[1]*v2[1];
    double magnitude = std::abs(v1[0]*v2[1] - v2[0]*v1[1]);

    return std::atan2(magnitude, dot);

}

