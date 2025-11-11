#define _USE_MATH_DEFINES

#include "../../Graphs/Interfaces/PlanarEmbedding.h"
#include "../Interfaces/CirclePacking.h"
#include "../Interfaces/Circle.h"
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

using std::vector;
using std::unordered_map;
using std::array;
using std::sqrt;
using std::pow;

CirclePacking::CirclePacking(DCEL& dcel) {

    object = &dcel;

    dcel.triangulate();

    //Add interior vertices first
    for (Vertex* vertex : dcel.vertices) {
        if (!dcel.exteriorVertices.count(vertex)) {

            centers.push_back(vertex);
        }
    }
    interiorVertexCount = centers.size();
    //Then add exterior vertices (iterating through vertices() to add counterclockwise)
    for (Vertex* vertex : dcel.exteriorFace->vertices()) {
        centers.push_back(vertex);
    }
    
    //createLookup to quickly connect DCEL to adjacencyMatrix
    for (int i=0 ; i<centers.size() ; i++) {
        vertexLookup[centers[i]] = i;
    }

    createAdjacencyMatrix();
    computeEffectiveRadii();

    placeExteriorCircles();
    // placeInteriorCircles();

    computeInradii();

    //Full Approximation
    // for (int i=0 ; i<3 ; i++) {
    //     approximationStep();
    // }

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


//Initializes edge conductance matrix to standard adjacency matrix values
void CirclePacking::createAdjacencyMatrix() {

    //Initialize Empty matrix
    edgeConductance = vector<vector<int>>(centers.size(), vector<int>(centers.size(), 0));

    for (HalfEdge* edge : object->edges) {

        edgeConductance[vertexLookup[edge->origin]][vertexLookup[edge->twin->origin]] = 1;
    }

}


//STEP C:
    //  Calculate angles of sectors for every vertex
    //      Calculate aim ofevery vertex (2PI if interior, sum of sector angles if exterior)
    //      Use this to calculate effective radius of every vertex

// Needs to be checked
void CirclePacking::computeEffectiveRadii() {

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

        centers[i]->radius = sqrt(areaSum / (2*M_PI));

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

        centers[i]->radius = sqrt(areaSum / aim);
    }

}
//
// Needs to be checked
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

// Correctly places exteriors tangent to each other
// may not center them in the correct place
void CirclePacking::placeExteriorCircles() {

    double rho = estimateBoundingRadius();

    //Place the first exterior circle
    centers[interiorVertexCount]->relocate({
        rho-centers[interiorVertexCount]->radius,
        0,
        centers[interiorVertexCount]->position[2]
    });

    double cummulativeAngle = 0;

    for (int i=interiorVertexCount+1 ; i<centers.size() ; i++) {

        double radius = rho - centers[i]->radius;
        cummulativeAngle += std::acos(1 - 
            (2 * centers[i-1]->radius * centers[i]->radius) / (radius * (rho - centers[i-1]->radius)));

        centers[i]->relocate({
            radius*std::cos(cummulativeAngle),
            radius*std::sin(cummulativeAngle),
            centers[i]->position[2]
        });
    }

}
//
// Definitely Functional
double CirclePacking::estimateBoundingRadius() const { 

    double upper = std::accumulate(centers.begin()+interiorVertexCount, centers.end(), 0.0,
        [](double s, const Vertex* v) {
            return s + v->radius;
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
// Must be working since used in placeExteriorCircles()
double CirclePacking::sumExteriorOverRho(double rho) const {

    double sum = 0;

    for (int i=interiorVertexCount ; i<centers.size()-1 ; i++) {

        sum += std::acos(1 - 
            (2 * centers[i]->radius * centers[i+1]->radius) / 
            ((rho - centers[i]->radius) * (rho - centers[i+1]->radius)));

    }

    return sum + std::acos(1 - 
        (2 * centers[centers.size()-1]->radius * centers[interiorVertexCount]->radius) / 
        ((rho - centers[centers.size()-1]->radius) * (rho - centers[interiorVertexCount]->radius)));

}


//STEP B: the magic
//  Calculate radius of incircles for each face
//  Calculate edge conductance using inradii
//  Calculate node conductance (sum of incident edge conductance)
//  Create Transition probability matrix
//The crucial step: solve matrix system for centers of interior vertices

// Definitely Functional (second display window)
void CirclePacking::computeInradii() {

    unordered_map<HalfEdge*, double> length;

    for (HalfEdge* edge : object->edges) {
        length[edge] = edge->length();
    }

    for (Face* face : object->faces) {

        if (! face->isExterior) {

            double a = length[face->edge];
            double b = length[face->edge->next];
            double c = length[face->edge->next->next];

            //Half Perimeter
            double s = (a + b + c)/2;

            face->inradius = sqrt(((s-a)*(s-b)*(s-c)) / s);

        }
    }
}
//
// Need to check that the correct radii are being used
void CirclePacking::computeEdgeConductance() {

    for (HalfEdge* edge : object->edges) {

        // if ( object->exteriorVertices.count(edge->origin) && 
        //      object->exteriorVertices.count(edge->twin->origin)) 
        //     continue;

        // double denominator = (edge->length()*edge->length())/2;

        // double conductance = (edge->face->inradius * edge->twin->face->inradius) /
        //                      denominator;


        double conductance = (edge->face->inradius * edge->twin->face->inradius) /
                             (edge->origin->radius + edge->twin->origin->radius);

        edgeConductance[vertexLookup[edge->origin]][vertexLookup[edge->twin->origin]] = conductance;

    }
}
//
// Needs to be checked
// Check that zeros match with adjacency matrix (if u and v arent adjacent and along the diagonal)
// Is node conductance always the sum of entries in the edge conductance matrix?
Eigen::MatrixXd CirclePacking::interiorTransitionProbabilities() const {

    Eigen::MatrixXd transitionProbabilities = Eigen::MatrixXd::Zero(interiorVertexCount, interiorVertexCount);

    for (int i=0 ; i<interiorVertexCount ; i++) {

        double conductanceSum = std::accumulate(edgeConductance[i].begin(), 
                                                edgeConductance[i].end(), 0.0);

        for (int j=0 ; j<interiorVertexCount ; j++) {
            if (edgeConductance[i][j]) {

                transitionProbabilities(i, j) = edgeConductance[i][j] / conductanceSum;
            }
        }
    }
    return transitionProbabilities;
}
//
// Needs to be checked
// Same as interiorTransitionProbabilities
Eigen::MatrixXd CirclePacking::exteriorTransitionProbabilities() const {

    Eigen::MatrixXd transitionProbabilities = Eigen::MatrixXd::Zero(interiorVertexCount, centers.size()-interiorVertexCount);

    for (int i=0 ; i<interiorVertexCount ; i++) {

        double conductanceSum = std::accumulate(edgeConductance[i].begin(), 
                                                edgeConductance[i].end(), 0.0);

        for (int j=interiorVertexCount ; j<centers.size() ; j++) {
            if (edgeConductance[i][j]) {

                transitionProbabilities(i, j) = edgeConductance[i][j] / conductanceSum;
            }
        }
    }
    return transitionProbabilities;
}
//
// Needs to be checked
// Must better understand how to use Eigen solver
void CirclePacking::placeInteriorCircles() {

    computeInradii();
    computeEdgeConductance();

    Eigen::MatrixXd B0 = interiorTransitionProbabilities() - Eigen::MatrixXd::Identity(interiorVertexCount, interiorVertexCount);
    Eigen::MatrixXd Ad = exteriorTransitionProbabilities();

    Eigen::MatrixXd Zd = Eigen::MatrixXd::Zero(centers.size()-interiorVertexCount, 2);
    for (int i=interiorVertexCount ;  i<centers.size() ; i++) {

        Zd(i-interiorVertexCount, 0) = centers[i]->position[0];
        Zd(i-interiorVertexCount, 1) = centers[i]->position[1];

    }

    Eigen::MatrixXd Z0 = B0.llt().solve(-Ad * Zd);

    //solve:
    //  (interiorTransitionProbabilities - IDENTITY(interiorVertexCount)) * innerCenters 
    //                  = 
    //  -exteriorTransitionProbabilities * outerCenters )
    //for:
    //  innerCenters

    //update innerCenter->position based on result
    for (int i=0 ; i<interiorVertexCount ; i++) {
        centers[i]->position[0] = Z0(i, 0);
        centers[i]->position[0] = Z0(i, 1);
        std::cout << centers[i]->position[0] << std::endl;
        std::cout << centers[i]->position[1] << std::endl;
    }

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