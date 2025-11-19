#ifndef HALFEDGE_H
#define HALFEDGE_H

class Vertex;
class Face;

class HalfEdge {

    public:

        HalfEdge();
        HalfEdge(Vertex* org);

        Vertex* origin = 0;

        HalfEdge* twin = 0;
        HalfEdge* next = 0;

        Face* face = 0;

        double length();
        double angleWith(HalfEdge* edge);

};

double slope(Vertex* v1, Vertex* v2);

#endif