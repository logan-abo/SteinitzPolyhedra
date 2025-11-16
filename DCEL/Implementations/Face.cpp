#include "../Interfaces/Face.h"
#include "../Interfaces/HalfEdge.h"
#include "../Interfaces/Vertex.h"

#include <array>
#include <vector>

using std::vector;
using std::array;


Face::Face(bool exterior) {

    isExterior = exterior;

    if (isExterior) {

        inradius = 1;
    }

}

vector<Vertex*> Face::vertices() {

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

vector<HalfEdge*> Face::edges() {

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