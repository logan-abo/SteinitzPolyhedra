#include "../Interfaces/ConductanceEdge.h"
#include "../../DCEL/Interfaces/Vertex.h"

ConductanceEdge::ConductanceEdge(Vertex* org) : 
    HalfEdge(org),
    conductance(0) {}