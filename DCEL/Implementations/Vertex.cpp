#include "../Interfaces/Vertex.h"
#include "../Interfaces/HalfEdge.h"

#include <cmath>
#include <array>

Vertex::Vertex() {}

Vertex::Vertex(const std::array<double, 3>& loc) {

    position = loc;

}

Vertex::Vertex(double radius,
               double angle) { 
    
    //This constructor is for creating planar embeddings
    position[2] = 0;

    position[0] = radius*cos(angle);
    position[1] = radius*sin(angle);

}

Vertex::~Vertex() {}

void Vertex::relocate(const std::array<double, 3>& loc) {

    position = loc;
}

