#define _USE_MATH_DEFINES

#include "../Interfaces/DCEL.h"
#include "../Interfaces/Vertex.h"
#include "../Interfaces/HalfEdge.h"
#include "../Interfaces/Face.h"
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
using std::array;


Vertex* DCEL::new_vertex(array<double, 3> coords) {

    // Call Vertex Factory
    Vertex* newVertex = allocateVertex(coords);

    // Add to memory tracking
    vertices.push_back(newVertex);

    return newVertex;
}

HalfEdge* DCEL::new_halfEdge(Vertex* vertex) {

    // Call HalfEdge Factory
    HalfEdge* newHalfEdge = allocateHalfEdge(vertex);

    // Add to memory tracking
    edges.push_back(newHalfEdge);

    return newHalfEdge;

}


Vertex* DCEL::allocateVertex(array<double, 3> coords) {

    return new Vertex(coords);
}

HalfEdge* DCEL::allocateHalfEdge(Vertex* vertex) {

    return new HalfEdge(vertex);
}


double distanceBetween(const Vertex* v, const Vertex* u) {

    double dx = v->position[0] - u->position[0];
    double dy = v->position[1] - u->position[1];
    double dz = v->position[2] - u->position[2];

    return sqrt(dx*dx + dy*dy + dy*dz);

}


// DEPRECATED METHOD. NO LONGER SERVES A PURPOSE
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


//
DCEL::DCEL() {}

// Create DCEL from planar embedding
DCEL::DCEL(const PlanarEmbedding& g) {

    buildFromEmbedding(g);

}

void DCEL::buildFromEmbedding(const PlanarEmbedding& g) {

    std::pair<int, double> leftMostVertex = {-1, std::numeric_limits<double>::max()};

    //create all vertices, keep track of leftMost
    for (int u=0 ; u<g.order() ; u++) {

        new_vertex(g.vertex(u));
        
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

            HalfEdge* edge = new_halfEdge(vertices[u]);

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

        if (!(edge->face)) {

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

        while (vertex->leaving->face != exteriorFace) {

            vertex->leaving = currentLeaving->twin->next;
            currentLeaving = vertex->leaving;
        }
    }

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

    for (Vertex* vertex : vertices) delete vertex;
    for (HalfEdge* edge : edges) delete edge;
    for (Face* face : faces) delete face;

}


vector<Vertex*> DCEL::triangulate() {

    vector<Vertex*> newVertices;

    int i=0;
    while (i<faces.size()) {
        
        if (!faces[i]->isExterior && 
            faces[i]->numSides() != 3) {

            newVertices.push_back(triangulate(i));
        }
        else i++;
    }

    return newVertices;
}

// The order of the creation of edges matters a lot. That is a weakness
// of this triangulate function. Should be refactored in the future.
Vertex* DCEL::triangulate(int faceIndex) {

    Face* oldFace = faces[faceIndex];

    Vertex* newVertex = new_vertex(oldFace->centroid());

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

        HalfEdge* nextEdge = new_halfEdge(newVertex);

        HalfEdge* newEdge = new_halfEdge(edge->twin->origin);

        newEdge->next = nextEdge;
        nextEdge->next = edge;
        edge->next = newEdge;
        
        newEdge->face = newFace;
        nextEdge->face = newFace;
        edge->face = newFace;

        faces.push_back(newFace);

        numNewFaces++;

    } while (current != start);

    newVertex->leaving = edges[edges.size()-2];

    for (int i=0 ; i<numNewFaces-1 ; i++) {

        edges[oldNumEdges + 2*i + 1]->twin = edges[oldNumEdges + 2*i + 2];
        edges[oldNumEdges + 2*i + 2]->twin = edges[oldNumEdges + 2*i + 1];

    }
    edges[oldNumEdges]->twin = edges[edges.size()-1];
    edges[edges.size()-1]->twin = edges[oldNumEdges];

    return newVertex;
    
}

void DCEL::triangulate(Face* oldFace) {

    int faceIndex = std::distance(faces.begin(), 
                                  std::find(faces.begin(), 
                                            faces.end(), 
                                            oldFace));
    triangulate(faceIndex);

}


void DCEL::addVertex(array<double, 3> coords) {

    // Create new Vertex (u)
    Vertex* newVertex = new_vertex(coords);


    // Find closest interior vertex (v)
    Vertex* closestExterior;
    double closestDistance = std::numeric_limits<double>::max();
    
    for (Vertex* vertex : exteriorFace->vertices()) {

        double distance = distanceBetween(newVertex, vertex);
        if (distance < closestDistance) {

            closestExterior = vertex;
            closestDistance = distance;

        }
    }

    // rotate around v to find previous edge in exteriorFace
    HalfEdge* v_vm = closestExterior->leaving;
    do {
        v_vm = v_vm->twin->next;
    }
    while (exteriorVertices.count(v_vm->twin->origin) == 0);

    HalfEdge* vm_previous = v_vm->twin;
    do {
        vm_previous = vm_previous->twin->next;
    }
    while (exteriorVertices.count(vm_previous->twin->origin) == 0);

    // Track 4 useful existing edges and create the 6 new edges
    HalfEdge* vm_v = v_vm->twin;
    HalfEdge* v_vp = closestExterior->leaving;
    HalfEdge* v_vp_next = v_vp->next;
    //
    vm_previous = vm_previous->twin;
    //
    HalfEdge* v_u = new_halfEdge(closestExterior);
    HalfEdge* u_v = new_halfEdge(newVertex);
    HalfEdge* u_vp = new_halfEdge(newVertex);
    HalfEdge* vp_u = new_halfEdge(closestExterior->leaving->twin->origin);
    //
    HalfEdge* vm_u = new_halfEdge(vm_v->origin);
    HalfEdge* u_vm = new_halfEdge(newVertex);

    // Track previous vertex in exterior (vm)
    Vertex* previousExterior = vm_v->origin;

    // Set nexts
    vm_v->next = v_u;
    v_u->next = u_vp;
    u_vp->next = v_vp_next;
    //
    u_v->next = v_vp;
    v_vp->next = vp_u;
    vp_u->next = u_v;
    //
    //
    vm_previous->next = vm_u;
    vm_u->next = u_vp;
    //
    v_u->next = u_vm;
    u_vm->next = vm_v;

    // Pair twins
    u_v->twin = v_u;
    v_u->twin = u_v;
    //
    u_vp->twin = vp_u;
    vp_u->twin = u_vp;
    //
    //
    vm_u->twin = u_vm;
    u_vm->twin = vm_u;

    // Create the new face and set edge
    Face* leftFace = new Face();
    Face* rightFace = new Face();
    //
    faces.push_back(leftFace);
    faces.push_back(rightFace);
    //
    leftFace->edge = u_v;
    rightFace->edge = v_u;
    exteriorFace->edge = u_vp;

    // Match edges to faces
    u_v->face = leftFace;
    v_vp->face = leftFace;
    vp_u->face = leftFace;
    //
    v_u->face = exteriorFace;
    u_vp->face = exteriorFace;
    //
    //
    vm_v->face = rightFace;
    v_u->face = rightFace;
    u_vm->face = rightFace;
    //
    vm_u->face = exteriorFace;

    // Update vertex leaving edges
    closestExterior->leaving = v_u;
    newVertex->leaving = u_vp;
    previousExterior->leaving = vm_u;

    // New vertex is in the exterior
    exteriorVertices.insert(newVertex);
    exteriorVertices.erase(closestExterior);

}


void DCEL::deleteVertex(Vertex* vertex) {

    vector<Face*> oldFaces;
    vector<HalfEdge*> spokes;
    vector<HalfEdge*> rimEdges;

    // Rotate around vertex being deleted
    HalfEdge* start = vertex->leaving;
    HalfEdge* current = start;
    do {

        // Collect face to be deleted
        oldFaces.push_back(current->face);

        // Collect spoke pairs to be deleted
        spokes.push_back(current);
        spokes.push_back(current->twin);

        // Collect edges on rim to splice into new face
        rimEdges.push_back(current->next);

        current = current->twin->next;
    } 
    while (current != start);

    // Delete old faces and check if any are the exterior
    bool deleteExterior = false;
    for (Face* face : oldFaces) {

        if (face->isExterior) {
            deleteExterior = true;
        }
        delete face;
        faces.erase(std::remove(faces.begin(), faces.end(), face), faces.end());
    }

    // Delete old spokes
    for (HalfEdge* edge : spokes) {
        delete edge;
        edges.erase(std::remove(edges.begin(), edges.end(), edge), edges.end());
    }

    // Create new face
    Face* newFace = new Face(deleteExterior);
    newFace->edge = rimEdges[0];

    // Splice rim edges into new face
    for (int i=1 ; i<rimEdges.size() ; i++) {
        rimEdges[i]->next = rimEdges[i-1];
        rimEdges[i]->face = newFace;
    }
    rimEdges[0]->next = rimEdges[rimEdges.size()-1];
    rimEdges[0]->face = newFace;

    if (deleteExterior) {
        updateExteriorFace(newFace);
    } 
    else {
        // Change newFace vertices to assure "leaving" wasnt deleted
        for (HalfEdge* edge : rimEdges) {

            edge->origin->leaving = edge;
        }
    }

    // Delete old vertex
    delete vertex;
    vertices.erase(std::remove(vertices.begin(), vertices.end(), vertex), vertices.end());

    // Add newFace to DCEL memory tracking
    faces.push_back(newFace);

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


void DCEL::updateExteriorFace(Face* newFace) {

    if (exteriorFace) {
        exteriorFace->isExterior = false;
    }

    newFace->isExterior = true;
    exteriorFace = newFace;

    exteriorVertices.clear();

    for (Vertex* vertex : newFace->vertices()) {
        exteriorVertices.insert(vertex);
    }

    for (HalfEdge* edge : newFace->edges()) {
        edge->origin->leaving = edge;
    }

}

