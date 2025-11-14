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
        
        void triangulate();
        void triangulate(int faceIndex);
        void triangulate(Face* face);

};

#endif