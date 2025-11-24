#include "../Interfaces/SteinitzPolyhedron.h"
#include "../Interfaces/CirclePacking.h"
#include "../Interfaces/CircleVertex.h"
#include "../../DCEL/Interfaces/DCEL.h"
#include "../../DCEL/Interfaces/Face.h"
#include "../../DCEL/Interfaces/HalfEdge.h"
#include "../../DCEL/Interfaces/Vertex.h"

#include <vector>
#include <array>
#include <memory>

using std::vector;
using std::array;
using std::unique_ptr;
using std::make_unique;


void semiStereographic(CircleVertex* vertex) {

    double scale = 7;

    double r = scale*vertex->radius;
    double x = scale*vertex->position[0];
    double y = scale*vertex->position[1];

    double P = x*x + y*y - r*r;

    array<double, 3> coneApex = {
        (2*x)/(P+1), 
        (2*y)/(P+1), 
        (P-1)/(P+1)
    };

    vertex->position = coneApex;

}


SteinitzPolyhedron::SteinitzPolyhedron(CirclePacking& pack) {

    packed = &pack;

    projectToSphere();

}

void SteinitzPolyhedron::projectToSphere() {

    for (Vertex* vertex : packed->vertices) {

        semiStereographic(packed->cast(vertex));
    }

}

// unordered_map<HalfEdge*, array<double, 3>> SteinitzPolyhedron::tangencies() {

//     unordered_map<HalfEdge*, array<double, 3>> circleTangencies;

//     for (HalfEdge* edge : packed->edges) {

//         circleTangencies[edge] = packed->tangencyPoint(edge);
//     }

//     return circleTangencies;
// }

