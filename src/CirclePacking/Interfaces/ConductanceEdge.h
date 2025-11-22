#ifndef CONDUCTANCEEDGE_H
#define CONDUCTANCEEDGE_H

#include "../../DCEL/Interfaces/HalfEdge.h"

class Vertex;

class ConductanceEdge : public HalfEdge {

    public:

        ConductanceEdge(Vertex* org);
    
        double conductance;

};

#endif