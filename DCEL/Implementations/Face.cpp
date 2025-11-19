#include "../Interfaces/Face.h"
#include "../Interfaces/HalfEdge.h"
#include "../Interfaces/Vertex.h"

#include <array>
#include <vector>
#include <cmath>

using std::vector;
using std::array;


Face::Face(bool exterior) {

    isExterior = exterior;

    if (isExterior) {

        inradius = 1;
    }

}

vector<Vertex*> Face::vertices() const {

    vector<Vertex*> vertexList;

    HalfEdge* start = this->edge;
    HalfEdge* current = start;

    do {
        vertexList.push_back(current->origin);
        current = current->next;
    }
    while (current != start);

    return vertexList;

}

vector<HalfEdge*> Face::edges() const {

    vector<HalfEdge*> edgeList;

    HalfEdge* start = this->edge;
    HalfEdge* current = start;

    do {
        edgeList.push_back(current);
        current = current->next;
    }
    while (current != start);

    return edgeList;

}

int Face::numSides() {

    HalfEdge* start = edge;
    HalfEdge* current = start;

    int sides = 0;

    do {
        sides++;
        current = current->next;
    } 
    while (current != start);

    return sides;

}

std::array<double, 3> Face::centroid() const {

        int numSides = 0;

        double xSum = 0;
        double ySum = 0;
        double zSum = 0;

        HalfEdge* current = edge;

        do {

            numSides++;

            xSum += current->origin->position[0];
            ySum += current->origin->position[1];
            zSum += current->origin->position[2];

            current = current->next;

        } while (current != edge);

        return {xSum/numSides, 
                ySum/numSides, 
                zSum/numSides};

}

array<double, 3> Face::normal() const {
    array<double, 3> p1 = edge->origin->position;
    array<double, 3> p2 = edge->next->origin->position;
    array<double, 3> p3 = edge->next->next->origin->position;

    // Vector 1
    p2[0] -= p1[0];
    p2[1] -= p1[1];
    p2[2] -= p1[2];

    // Vector 2
    p3[0] -= p1[0];
    p3[1] -= p1[1];
    p3[2] -= p1[2];

    // Cross Product
    array<double, 3> normal = {
        (p3[1] * p2[2]) - (p3[2] * p2[1]),
        (p3[2] * p2[0]) - (p3[0] * p2[2]),
        (p3[0] * p2[1]) - (p3[1] * p2[0])
    };

    // Normalize to unit length
    double length = std::sqrt(normal[0]*normal[0] + 
                              normal[1]*normal[1] + 
                              normal[2]*normal[2]);
                              
    return {normal[0]/length, normal[1]/length, normal[2]/length};
}
