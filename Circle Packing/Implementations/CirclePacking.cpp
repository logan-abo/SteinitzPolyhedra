#include "../../Graphs/Interfaces/PlanarEmbedding.h"
#include "../Interfaces/CirclePacking.h"
#include "../Interfaces/Circle.h"
#include "../../DCEL/Interfaces/DCEL.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <unordered_map>
#include <array>
#include <unordered_set>
#include <numeric>
#include <algorithm>

using std::vector;
using std::unordered_map;
using std::array;
using std::sqrt;
using std::pow;

CirclePacking::CirclePacking(DCEL& dcel) {

    object = &dcel;

    dcel.triangulate();

    // embedding = new PlanarEmbedding(dcel);

    //Add interior vertices first
    for (Vertex* vertex : dcel.vertices) {
        if (!dcel.exteriorVertices.count(vertex)) {

            centers.push_back(vertex);
        }
    }
    interiorVertexCount = centers.size();
    //Add correct exterior indices last (iterating through vertices() to add counterclockwise)
    for (Vertex* vertex : dcel.exteriorFace->vertices()) {
        centers.push_back(vertex);
    }
    
    computeEffectiveRadii();

    //PART A
    placeExteriorCircles();

    //These should happen in a constructor for a planar embedding object
    //  organize vertices: interior first, exterior last (organized counterclockwise)
    //  --for every vertex, create adjacency list, organized counterclockwise--

    //STEP C:
    //  Calculate angles of sectors for every vertex
    //      Calculate aim ofevery vertex (2PI if interior, sum of sector angles if exterior)
    //      Use this to calculate effective radius of every vertex
    //STEP A:
    //  Estimate solution to MONOTONIC angle sum condition (start above zero, increase)
    //      place first exterior vertex circle on x-axis
    //      use estimated external radius to calculate location of next exterior circle
    //      place all exterior circles succeessively
    //  Rescale all circles to be within unit circle 
    //STEP B: the magic
    //  Calculate radius of incircles for each face
    //  Calculate edge conductance using inradii
    //  Calculate node conductance (sum of incident edge conductance)
    //  Create Transition probability matrix
    //The crucial step: solve matrix system for centers of interior vertices
    
}

//STEP C:
    //  Calculate angles of sectors for every vertex
    //      Calculate aim ofevery vertex (2PI if interior, sum of sector angles if exterior)
    //      Use this to calculate effective radius of every vertex
void CirclePacking::computeEffectiveRadii() {

    radii.clear();

    //Effective radii for interior vertices
    // Aim is 2pi
    // same number of sectors as degree
    for (int i=0 ; i<interiorVertexCount ; i++) {

        HalfEdge* start = centers[i]->leaving;
        HalfEdge* current = start;

        double areaSum = 0;

        do {

            double angle = current->angleWith(current->twin->next);

            //Sector radius calculation depends on face, 
            //Current edge is exterior to the sector angle we just calculated so:
            //  next edge (counterclockwise along the vertex) must be used
            double radius = sectorRadius(current->twin->next);

            areaSum += angle*radius*radius;

            current = current->twin->next;
        }
        while (current != start);

        radii.push_back( sqrt(areaSum / (2*M_PI)) );

    }

    //Effective radii for exterior vertices:
    //  Aim has to be calculated
    //  there is degree-1 sectors
    for (int i=interiorVertexCount ; i<centers.size() ; i++) {

        double areaSum = 0;
        double aim = 0;

        HalfEdge* current = centers[i]->leaving;

        int degree = object->degree(centers[i]);
        for (int j=0 ; j<degree-1 ; j++) {

            double angle = current->angleWith(current->twin->next);
            double radius = sectorRadius(current->twin->next);

            areaSum += angle*radius*radius;

            aim += angle;

        }

        radii.push_back( sqrt(areaSum / aim) );
    }

}
//
double CirclePacking::sectorRadius(HalfEdge* edge) const {

    // much nicer than index-modulo, takes advantage of DCEL pointers
    return ( edge->length() - edge->next->length() + edge->next->next->length() ) / 2;

}


//STEP A:
//  Estimate solution to MONOTONIC angle sum condition (start above zero, increase)
//      place first exterior vertex circle on x-axis
//      use estimated external radius to calculate location of next exterior circle
//      place all exterior circles succeessively
//  Rescale all circles to be within unit circle 
double CirclePacking::estimateBoundingRadius() const {

    double upper = std::accumulate(radii.begin()+interiorVertexCount, 
                                   radii.end(), 0.0);
    double lower = *std::max_element(radii.begin() + interiorVertexCount,
                                     radii.end());

    double rho;
    double approx;

    for (int i=0 ; i<20 ; i++) {

        rho = (upper+lower)/2;
        approx = sumExteriorOverRho(rho) - 2*M_PI;

        if (approx < 0) {

            upper = rho;

        } else {

            lower = rho;

        }
    }

    return rho;
}
//
double CirclePacking::sumExteriorOverRho(double rho) const {

    double sum = 0;

    for (int i=interiorVertexCount ; i<radii.size()-1 ; i++) {

        sum += std::acos(1 - 
            (2 * radii[i] * radii[i+1]) / 
            ((rho - radii[i]) * (rho - radii[i+1])));

    }

    return sum + std::acos(1 - 
        (2 * radii[radii.size()-1] * radii[interiorVertexCount]) / 
        ((rho - radii[radii.size()-1]) * (rho - radii[interiorVertexCount])));

}
//
void CirclePacking::placeExteriorCircles() {

    double rho = estimateBoundingRadius();

    //Place the first exterior circle
    centers[interiorVertexCount]->relocate({
        rho-radii[interiorVertexCount],
        0,
        centers[interiorVertexCount]->position[2]
    });

    double cummulativeAngle = 0;

    for (int i=interiorVertexCount+1 ; i<centers.size() ; i++) {

        double radius = rho - radii[i];
        cummulativeAngle += std::acos(1 - (2*radii[i-1]*radii[i]) / (radius*(rho - radii[i-1])));

        centers[i]->relocate({
            radius*std::cos(cummulativeAngle),
            radius*std::sin(cummulativeAngle),
            centers[i]->position[2]
        });
    }

}


//STEP B: the magic
//  Calculate radius of incircles for each face
//  Calculate edge conductance using inradii
//  Calculate node conductance (sum of incident edge conductance)
//  Create Transition probability matrix
//The crucial step: solve matrix system for centers of interior vertices
void CirclePacking::computeInradii() const {

    // unordered_map<Face*, double> inradii;

    // for (Face* face : object->faces) {

    //     double productOfRadii = 1;
    //     double sumOfRadii = 0;

    //     inradii[face] = sqrt(productOfRadii / sumOfRadii);

    // }
}


//Calculate desired sum of sector angles around a vertex
// No longer needed since this would require the calculation of the sector angle twice;
// double CirclePacking::aim(int index) {
// 
//     if (index<interiorVertexCount) {
//         return 2*M_PI;
//     }
// 
//     double angleSum = 0;
//     HalfEdge* start = object->vertices[index]->leaving;
// 
//     for (int i=0 ; i<object->degree(start->origin)-1 ; i++) {
//         angleSum += start->angleWith(start->twin->next);
//         start = start->twin->next;
//     }
// 
//     return angleSum;
// 
// }

//Original Visualizer for multi-circles around vertex (uncut sectors)
// void CirclePacking::createInitialSectors() {
// 
//     for (Face* face : object->faces) {
//         if (! face->isExterior) {
// 
//             auto vertices = face->vertices();
//             auto faceEdges = face->edges();
// 
//             array<double, 3> lengths;
// 
//             for (int i=0 ; i<faceEdges.size() ; i++) {
//                 lengths[i] = faceEdges[i]->length();
//             }
//             for (int i=0 ; i<vertices.size() ; i++) {
// 
//                 double radius = (lengths[i] + lengths[(i+2)%3] - lengths[(i+1)%3]) / 2;
// 
//                 Circle* circle = new Circle({vertices[i]->position[0], 
//                                              vertices[i]->position[1]}, 
//                                             radius);
//                 circles.push_back(circle);
// 
//             }
// 
//         }
//     }
// 
// }