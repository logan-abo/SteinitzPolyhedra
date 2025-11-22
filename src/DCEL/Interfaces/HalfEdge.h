#ifndef HALFEDGE_H
#define HALFEDGE_H

class Vertex;
class Face;

class HalfEdge {

    public:

        HalfEdge(Vertex* org);

        Vertex* origin = nullptr;

        HalfEdge* twin = nullptr;
        HalfEdge* next = nullptr;

        Face* face = nullptr;

        double length();
        double angleWith(HalfEdge* edge);



        // DEPRECATED CONSTRUCTOR
        HalfEdge();
};

double slope(Vertex* v1, Vertex* v2);

#endif