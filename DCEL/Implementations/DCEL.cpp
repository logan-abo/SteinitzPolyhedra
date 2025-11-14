#define _USE_MATH_DEFINES

#include "../Interfaces/DCEL.h"
#include "../Interfaces/HalfEdge.h"
#include "../../Graphs/Interfaces/PlanarEmbedding.h"

#include <vector>
#include <utility>
#include <map>
#include <array>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <unordered_set>

using std::map;


//Create n-gon on the unit circle
DCEL::DCEL(int n) {

    //Create Faces
    faces.push_back(new Face);
    faces.push_back(new Face);

    double rad = (2*M_PI)/n;

    //Create Vertices and Inner Half Edges
    for (int i=0 ; i<n ; i++) {

        vertices.push_back(new Vertex(3, rad*i));

        edges.push_back(new HalfEdge);

        vertices[i]->leaving = edges[i];

        edges[i]->origin = vertices[i];
        edges[i]->face = faces[1];

    }

    //Create Half Edge pairs
    for (int i=0 ; i<n ; i++) {

        edges.push_back(new HalfEdge);

        edges[i]->twin = edges[i+n];
        edges[i+n]->twin = edges[i];

        edges[i+n]->face = faces[0];

    }

    //Give outside edges origins
    for (int i=0 ; i<n-1 ; i++) {
        edges[i+n]->origin = vertices[i+1];
    }
    edges[2*n-1]->origin = vertices[0];

    //Create Inner Half Edge chain
    for (int i=0 ; i<n-1 ; i++) {

        edges[i]->next = edges[i+1];

    }
    edges[n-1]->next = edges[0];

    //Create Outer Half Edge chain
    for (int i=2*n-1 ; i>n ; i--) {
        edges[i]->next = edges[i-1];
    }
    edges[n]->next = edges[2*n-1];

    faces[1]->edge = edges[0];
    faces[0]->edge = edges[n];

}

// Create DCEL from planar embedding
DCEL::DCEL(const PlanarEmbedding& g) {

    std::pair<int, double> leftMostVertex = {-1, std::numeric_limits<double>::max()};

    //create all vertices, keep track of leftMost
    for (int u=0 ; u<g.order() ; u++) {
        vertices.push_back(new Vertex(g.vertex(u)));
        
        if (vertices[u]->position[0] < leftMostVertex.second) {
            leftMostVertex.first = u; 
            leftMostVertex.second = vertices[u]->position[0];
        }
    }

    map<std::pair<int, int>, HalfEdge*> edgeMap;
    std::pair<int, double> steepestLeftMostEdge = {-1, std::numeric_limits<double>::min()};

    //links vertices with a leaving edge and create half edges (with origins)
    for (int u=0 ; u<vertices.size() ; u++) {

        for (int v=0 ; v<g.degree(u) ; v++) {

            HalfEdge* edge = new HalfEdge;
            edge->origin = vertices[u];

            edges.push_back(edge);

            edgeMap[std::make_pair(u, g.neighbors(u)[v])] = edge;

            if (u==leftMostVertex.first && 
                slope(vertices[u], vertices[v])>steepestLeftMostEdge.second) {

                steepestLeftMostEdge.first = edges.size()-1;
            }

        }

        vertices[u]->leaving = edges[edges.size()-1];

    }

    //pair twin half edges and link nexts
    for (auto const& [ends, edge] : edgeMap) {
        edgeMap[{ends.first, ends.second}]->twin = 
            edgeMap[{ends.second, ends.first}];

        edgeMap[{ends.first, ends.second}]->next = 
            edgeMap[std::make_pair(ends.second, g.clockwiseNextFrom(ends.second, ends.first))];
    }

    HalfEdge* exteriorEdge = edges[steepestLeftMostEdge.first];

    //create faces and match with half edges
    for (HalfEdge* edge : edges) {

        // std::cout << "In HalfEdge loop" <<std::endl;

        if (!(edge->face)) {

            // std::cout << "Creating new face" <<std::endl;

            Face* newFace = new Face;
            newFace->edge = edge;
            faces.push_back(newFace);

            HalfEdge* current = edge;
            do {
                if (current == exteriorEdge) {
                    newFace->isExterior = true;
                    exteriorFace = newFace;
                }
                current->face = newFace;
                current = current->next;

            } while (current != edge);

        }

    }

    //normalize exterior points so that exteriorVertex->leaving is an exterior edge
    auto exterior = exteriorFace->vertices();
    exteriorVertices = std::unordered_set(exterior.begin(), 
                                          exterior.end());
    for (Vertex* vertex : exterior) {

        HalfEdge* currentLeaving = vertex->leaving;

        while (exteriorVertices.find(currentLeaving->twin->origin) == exteriorVertices.end()) {

            vertex->leaving = currentLeaving->twin->next;
            currentLeaving = vertex->leaving;
        }
    }

    std::cout << "Num Exterior Vertices: ";
    std::cout << exteriorVertices.size() << std::endl;

}

// Possible different algorithm for DCEL constructor (see picture of whiteboard)
// DCEL::DCEL(const PlanarEmbedding& g) {
//
//     vector<vector<int>> adjCopy = g.adjacencyCopy();
// 
//     int halfEdgeCounter = 0;
// 
//     while (halfEdgeCounter < g.size()*2) {}
// 
// }

DCEL::~DCEL() {

    std::cout << "DCEL DESTRUCTOR" << std::endl;

    for (Vertex* vertex : vertices) delete vertex;
    for (HalfEdge* edge : edges) delete edge;
    for (Face* face : faces) delete face;

}

void DCEL::triangulate() {

    int i=0;
    while (i<faces.size()) {
        
        if (!faces[i]->isExterior && 
            faces[i]->numSides() != 3) {

            triangulate(i);
        }
        else i++;
    }

}

void DCEL::triangulate(int faceIndex) {

    Face* oldFace = faces[faceIndex];

    Vertex* newVertex = new Vertex(oldFace->centroid());
    vertices.push_back(newVertex);

    HalfEdge* start = oldFace->edge;
    HalfEdge* current = start;

    delete faces[faceIndex];
    faces.erase(faces.begin()+faceIndex);

    int numNewFaces = 0;
    int oldNumEdges = edges.size();

    do {
        HalfEdge* edge = current;
        current = current->next;

        Face* newFace = new Face;
        newFace->edge = edge;

        HalfEdge* newEdge = new HalfEdge;
        newEdge->origin = edge->twin->origin;

        HalfEdge* nextEdge = new HalfEdge;
        nextEdge->origin = newVertex;

        newEdge->next = nextEdge;
        nextEdge->next = edge;
        edge->next = newEdge;
        
        newEdge->face = newFace;
        nextEdge->face = newFace;
        edge->face = newFace;

        faces.push_back(newFace);
        edges.push_back(nextEdge);
        edges.push_back(newEdge);

        numNewFaces++;

    } while (current != start);

    newVertex->leaving = edges[edges.size()-2];

    for (int i=0 ; i<numNewFaces-1 ; i++) {

        edges[oldNumEdges + 2*i + 1]->twin = edges[oldNumEdges + 2*i + 2];
        edges[oldNumEdges + 2*i + 2]->twin = edges[oldNumEdges + 2*i + 1];

    }
    edges[oldNumEdges]->twin = edges[edges.size()-1];
    edges[edges.size()-1]->twin = edges[oldNumEdges];
    
}


int DCEL::degree(Vertex* vertex) {

    int deg = 0;

    HalfEdge* start = vertex->leaving;
    HalfEdge* current = start;

    do {
        deg++;
        current = current->twin->next;
    }
    while (current != start);

    return deg;

}

