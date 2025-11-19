#ifndef DCEL_H
#define DCEL_H

#include <vector>
#include <unordered_set>

using std::vector;
using std::unordered_set;

#include "Vertex.h"
#include "HalfEdge.h"
#include "Face.h"

class PlanarEmbedding;

class DCEL {

    private:


    public:

        DCEL(int numFaces);
        DCEL(const PlanarEmbedding& graph);

        ~DCEL();

        unordered_set<Vertex*> exteriorVertices;

        vector<Vertex*> vertices;
        vector<HalfEdge*> edges;

        Face* exteriorFace;
        vector<Face*> faces;

        int degree(Vertex* vertex);
        
        vector<Vertex*> triangulate();
        Vertex* triangulate(int faceIndex);
        void triangulate(Face* face);

        void deleteVertex(Vertex*);
        void addVertex(array<double, 3> coords);

        void updateExteriorFace(Face* newFace);

};

#endif