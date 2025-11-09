#ifndef FACE_H
#define FACE_H

#include <array>
#include <vector>

using std::vector;
using std::array;

class Vertex;
class HalfEdge;

class Face {

    public:
        Face(bool exterior=false);

        bool isExterior;

        HalfEdge* edge = 0;

        int numSides();
        
        vector<Vertex*> vertices();
        vector<HalfEdge*> edges();
        
        array<double, 3> centroid();

};

#endif