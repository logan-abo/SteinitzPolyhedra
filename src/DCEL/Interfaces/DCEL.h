#ifndef DCEL_H
#define DCEL_H

#include <vector>
#include <array>
#include <unordered_set>

using std::vector;
using std::array;
using std::unordered_set;

class Vertex;
class HalfEdge;
class Face;
class PlanarEmbedding;

class DCEL {

    private:

        // Calls Vertex factory and adds new factory to memory tracking
        Vertex* new_vertex(array<double, 3> coords);
        HalfEdge* new_halfEdge(Vertex* vertex);

        virtual Vertex* allocateVertex(array<double, 3> coords);
        virtual HalfEdge* allocateHalfEdge(Vertex* vertex);


    protected:

        DCEL();
        

    public:

        DCEL(int numFaces);
        DCEL(const PlanarEmbedding& graph);

        void buildFromEmbedding(const PlanarEmbedding& graph);

        virtual ~DCEL();

        unordered_set<Vertex*> exteriorVertices;

        vector<Vertex*> vertices;
        vector<HalfEdge*> edges;

        Face* exteriorFace;
        vector<Face*> faces;

        int degree(Vertex* vertex);
        
        void triangulate();
        void triangulate(int faceIndex);
        void triangulate(Face* face);

        void addVertex(array<double, 3> coords);

};

#endif