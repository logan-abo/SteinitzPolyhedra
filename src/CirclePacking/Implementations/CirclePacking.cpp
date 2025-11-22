#define _USE_MATH_DEFINES

#include "../../Graphs/Interfaces/PlanarEmbedding.h"
#include "../Interfaces/CirclePacking.h"
#include "../Interfaces/CircleVertex.h"
#include "../Interfaces/ConductanceEdge.h"
#include "../../DCEL/Interfaces/Vertex.h"
#include "../../DCEL/Interfaces/HalfEdge.h"
#include "../../DCEL/Interfaces/Face.h"
#include "../../DCEL/Interfaces/DCEL.h"

#include <Eigen/Dense>

#include <cmath>
#include <vector>
#include <unordered_map>
#include <array>
#include <unordered_set>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <chrono>

using std::vector;
using std::unordered_map;
using std::array;
using std::sqrt;
using std::pow;


// These could eventually be replaced with templating
CircleVertex* CirclePacking::cast(Vertex* vertex) const {
    return static_cast<CircleVertex*>(vertex);
}

ConductanceEdge* CirclePacking::cast(HalfEdge* edge) const {
    return static_cast<ConductanceEdge*>(edge);
}


Vertex* CirclePacking::allocateVertex(array<double, 3> coords) {

    return new CircleVertex(coords, 0.5);
}

HalfEdge* CirclePacking::allocateHalfEdge(Vertex* vertex) {

    return new ConductanceEdge(vertex);
}


CirclePacking::CirclePacking(const PlanarEmbedding& planeGraph) {

    // Effectively call the parent DCEL constructor
    buildFromEmbedding(planeGraph);

    // Packing algorithm requires a somewhat (ignore exterior face) maximal planar graph
    triangulate();

    sortVertices();

    std::cout << "Starting Packing..." << std::endl;
    std::cout << std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    // Full Approximation
    for (int i=0 ; i<75 ; i++) {
        approximationStep();
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << std::endl;
    std::cout << "Total Elapsed time: ";
    std::cout << std::chrono::duration_cast<std::chrono::minutes>(end - begin).count();
    std::cout << std::endl;

    //THE ALGORITHM
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

// Circle Packing alogirthm requires that the exterior vertices are last
// and in counter-clockwise order.
// Exterior Vertices should be sorted counter clockwise but otherwise the:
// updateExterior()/setExterior() method will handle this.
void CirclePacking::sortVertices() {

    vector<Vertex*> interiorVertices;

    // Find interior  vertices
    for (Vertex* vertex : vertices) {

        if (!exteriorVertices.count(vertex)) {
            interiorVertices.push_back(vertex);
        }
    }

    vertices.clear();
    interiorVertexCount = interiorVertices.size();

    for (Vertex* vertex : interiorVertices) {
        vertices.push_back(vertex);
    }
    for (Vertex* vertex : exteriorFace->vertices()) {
        vertices.push_back(vertex);
    }

}


//STEP C:
//  Calculate angles of sectors for every vertex
//      Calculate aim ofevery vertex (2PI if interior, sum of sector angles if exterior)
//      Use this to calculate effective radius of every vertex
//
void CirclePacking::computeEffectiveRadii() {

    //Effective radii for interior vertices
    // Aim is 2pi
    // same number of sectors as degree
    for (int i=0 ; i<interiorVertexCount ; i++) {

        HalfEdge* start = vertices[i]->leaving;
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

        cast(vertices[i])->radius = sqrt(areaSum / (2*M_PI));

    }

    //Effective radii for exterior vertices:
    //  Aim has to be calculated
    //  there is degree-1 sectors
    for (int i=interiorVertexCount ; i<vertices.size() ; i++) {

        double areaSum = 0;
        double aim = 0;

        HalfEdge* current = vertices[i]->leaving;

        int deg = degree(vertices[i]);
        for (int j=0 ; j<deg-1 ; j++) {

            double angle = current->angleWith(current->twin->next);
            double radius = sectorRadius(current->twin->next);

            areaSum += angle*radius*radius;

            aim += angle;

            current = current->twin->next;

        }

        cast(vertices[i])->radius = sqrt(areaSum / aim);
    }

}
//
double CirclePacking::sectorRadius(HalfEdge* edge) const {

    // much nicer than index-modulo, takes advantage of DCEL pointers
    return ( edge->length() - edge->next->length() + edge->next->next->length() ) / 2;

}


//STEP A:
//  Estimate solution to MONOTONIC angle sum condition
//      place first exterior vertex circle on x-axis
//      use estimated external radius to calculate location of next exterior circle
//      place all exterior circles succeessively
//  Rescale all circles to be within unit circle 
//
// Using the radius of the vounding circle, the exterior circles can be placed 1 by 1
void CirclePacking::placeExteriorCircles() {

    double rho = estimateBoundingRadius();

    //Place the first exterior circle
    vertices[interiorVertexCount]->relocate({
        rho - cast(vertices[interiorVertexCount])->radius,
        0,
        vertices[interiorVertexCount]->position[2]
    });

    double cummulativeAngle = 0;

    for (int i=interiorVertexCount+1 ; i<vertices.size() ; i++) {

        double radius = rho - cast(vertices[i])->radius;
        cummulativeAngle += std::acos(1 - 
            (2 * cast(vertices[i-1])->radius * cast(vertices[i])->radius) / (radius * (rho - cast(vertices[i-1])->radius)));

        vertices[i]->relocate({
            radius*std::cos(cummulativeAngle),
            radius*std::sin(cummulativeAngle),
            vertices[i]->position[2]
        });
    }

    scaleToUnitDisc(rho);

}
//
// This is the radius of the circle which bounds the entire packing.
// All vertices of the exterior face are internally tangent to this circle.
// The function for the radius is monotic so a simple estimating technique suffices.
double CirclePacking::estimateBoundingRadius() const { 

    double upper = std::accumulate(vertices.begin()+interiorVertexCount, vertices.end(), 0.0,
        [this](double s, Vertex* v) {
            return s + cast(v)->radius;
        });
    double lower = 0;
    // double lower = *std::max_element(radii.begin() + interiorVertexCount,
    //                                  radii.end());

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

    for (int i=interiorVertexCount ; i<vertices.size()-1 ; i++) {

        sum += std::acos(1 - 
            (2 * cast(vertices[i])->radius * cast(vertices[i+1])->radius) / 
            ((rho - cast(vertices[i])->radius) * (rho - cast(vertices[i+1])->radius)));

    }

    return sum + std::acos(1 - 
        (2 * cast(vertices[vertices.size()-1])->radius * cast(vertices[interiorVertexCount])->radius) / 
        ((rho - cast(vertices[vertices.size()-1])->radius) * (rho - cast(vertices[interiorVertexCount])->radius)));

}
//
// This scales the entire circle to be packed in the unit disc.
// Not a necessary step but it keeps the packing a consistent size.
void CirclePacking::scaleToUnitDisc(double boundingRadius) {

    for (Vertex* vertex : vertices) {

        vertex->scale(1/boundingRadius);

    }

}





//STEP B: the magic
//  Calculate radius of incircles for each face
//  Calculate edge conductance using inradii
//  Calculate node conductance (sum of incident edge conductance)
//  Create Transition probability matrix
//The crucial step: solve matrix system for centers of interior vertices

// Only deals with effective radius to compute an inradius of the given circle triples
void CirclePacking::computeInradii() {

    for (Face* face : faces) {
        if (! face->isExterior) {

            double ra = cast(face->edge->origin)->radius;
            double rb = cast(face->edge->next->origin)->radius;
            double rc = cast(face->edge->next->next->origin)->radius;

            face->inradius = sqrt( (ra*rb*rc) / (ra+rb+rc) );

        }
    }
}
//
// Edge conductance is zero for exterior edges
void CirclePacking::computeEdgeConductance() {

    for (HalfEdge* edge : edges) {

        // 1. Check Edge
        if (!edge) { std::cerr << "Null Edge!" << std::endl; continue; }

        // 2. Check Twins
        if (!edge->twin) { std::cerr << "Null Twin!" << std::endl; continue; }

        // 3. Check Faces (Likely Culprit)
        if (!edge->face) { 
            std::cerr << "Edge has no Face!" << std::endl; 
            continue; 
        }
        if (!edge->twin->face) { 
            std::cerr << "Twin has no Face!" << std::endl; 
            continue; 
        }

        // 4. Check Origins
        if (!edge->origin) { std::cerr << "Edge has no Origin!" << std::endl; continue; }
        if (!edge->twin->origin) { std::cerr << "Twin has no Origin!" << std::endl; continue; }

        double conductance = (edge->face->inradius + edge->twin->face->inradius) /
                             (cast(edge->origin)->radius + cast(edge->twin->origin)->radius);

        cast(edge)->conductance = conductance;

    }

    // Edges along the exterior have zero conductance (absorbing boundary)
    for (Vertex* exteriorVertex : exteriorVertices) {

        cast(exteriorVertex->leaving)->conductance = 0;
    }
}
//
// Compute all the edge conductance together. Necessary to normalize for Markov Probabilities
Eigen::MatrixXd CirclePacking::transitionProbabilities() const {

    // Vertex Lookup for creating matrices
    // This shouldn't get recreated every time, it does not change.
    // See comment in header file for why it exists 
    unordered_map<Vertex*, int> lookup;
    for (int i=0 ; i<vertices.size() ; i++) {

        lookup[vertices[i]] = i;
    }

    Eigen::MatrixXd transitionProbabilities = Eigen::MatrixXd::Zero(vertices.size(), vertices.size());

    // Load the edge conductances into the matrix
    for (int i=0 ; i<edges.size() ; i++) {

        ConductanceEdge* edge = cast(edges[i]);
        transitionProbabilities(lookup[edge->origin], lookup[edge->twin->origin]) = edge->conductance;
    }

    // Divide every element by the corresponding row sums to get Markov probabilities
    transitionProbabilities.array().colwise() /= transitionProbabilities.rowwise().sum().array();

    return transitionProbabilities;
}
//
void CirclePacking::placeInteriorCircles() {

    computeInradii();
    computeEdgeConductance();

    Eigen::MatrixXd transitions = transitionProbabilities();

    // Transition probabilities within interior vertices
    Eigen::MatrixXd A0 = transitions.block(0, 0, interiorVertexCount, interiorVertexCount);
    // Subtract the identity
    Eigen::MatrixXd B0 = A0 - Eigen::MatrixXd::Identity(interiorVertexCount, interiorVertexCount);

    // Transition probabilities between interior and exterior vertices
    Eigen::MatrixXd Ad = transitions.block(0, interiorVertexCount, interiorVertexCount, vertices.size()-interiorVertexCount);

    // Positions of exterior vertices
    Eigen::MatrixXd Zd = Eigen::MatrixXd::Zero(vertices.size()-interiorVertexCount, 2);
    for (int i=interiorVertexCount ;  i<vertices.size() ; i++) {

        Zd(i-interiorVertexCount, 0) = vertices[i]->position[0];
        Zd(i-interiorVertexCount, 1) = vertices[i]->position[1];

    }

    // Sparse matrix solving algorithm
    Eigen::MatrixXd Z0 = B0.lu().solve(-Ad * Zd);

    //update innerCenter->position based on result
    for (int i=0 ; i<interiorVertexCount ; i++) {
        vertices[i]->position[0] = Z0(i, 0);
        vertices[i]->position[1] = Z0(i, 1);
    }
}


double CirclePacking::getRadius(Vertex* vertex) const {
    return cast(vertex)->radius;
}



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