#include "DCEL/Interfaces/ObjectViewer.h"
#include "DCEL/Interfaces/DCEL.h"
#include "DCEL/Interfaces/HalfEdge.h"
#include "DCEL/Interfaces/Vertex.h"

#include "Graphs/Interfaces/SimpleGraph.h"
#include "Graphs/Interfaces/PlanarEmbedding.h"

#include "CirclePacking/Interfaces/CirclePacking.h"
#include "CirclePacking/Interfaces/PackingDisplay.h"
// #include "CirclePacking/Interfaces/SteinitzPolyhedron.h"

#include <iostream>
#include <vector>
#include <array>

using std::vector;
using std::array;

int main() {

    // SimpleGraph g({
    //     {0, 1, 1, 0},
    //     {1, 0, 1, 1},
    //     {1, 1, 0, 1},
    //     {0, 1, 1, 0}
    // });
    // vector<array<double, 3>> vertices = {
    //     {-1,  1, 0},
    //     { 1,  1, 0},
    //     {-1, -1, 0},
    //     { 1, -1, 0}
    // };

    SimpleGraph g({
        {0, 1, 1, 0, 0, 0, 0, 1},
        {1, 0, 1, 1, 0, 0, 0, 0},
        {1, 1, 0, 0, 1, 0, 1, 1},
        {0, 1, 0, 0, 1, 0, 0, 0},
        {0, 0, 1, 1, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0, 1, 0},
        {0, 0, 1, 0, 0, 1, 0, 1},
        {1, 0, 1, 0, 0, 0, 1, 0}
    });
    vector<array<double, 3>> vertices = {
        {0.86053,0.71747,0},
        {0.94882,-0.71224,0},
        {-0.19636,-0.1477,0},
        {0.28733,-1.84507,0},
        {-0.85909,-1.46321,0},
        {-1.83687,-0.57504,0},
        {-1.34235,0.62755,0},
        {-0.27462,1.32438,0}
    };

    SimpleGraph herschel({
      // A, B, C, D, E, F, G, H, I, J, K
        {0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0}, // A
        {1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0}, // B
        {0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1}, // C
        {1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0}, // D
        {1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0}, // E
        {0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1}, // F
        {1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0}, // G
        {0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0}, // H
        {0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1}, // I
        {0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0}, // J
        {0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0}  // K
    });
    vector<array<double, 3>> enneahedron = {
        {0, 0, 0},
        {-0.5,0.5,0},
        {0, 1, 0},
        {0.5, 0.5, 0},
        {0.5, -0.5, 0},
        {1, 0, 0},
        {-0.5, -0.5, 0},
        {-1, 0, 0},
        {0, -1, 0},
        {-1.5, 0, 0},
        {1.5, 0, 0}
    };


    // Create Planar Embedding
    // PlanarEmbedding planar(g, vertices);
    PlanarEmbedding planar(herschel, enneahedron);
    
    // CirclePacking DCEL object
    CirclePacking packingAttempt2(planar);

    // Choose a different exterior face for Herschels Enneahedron
    double minX = 100;
    Vertex* newExteriorVertex;
    for (Vertex* vertex : packingAttempt2.exteriorVertices) {

        if (vertex->position[0] < minX) {

            minX = vertex->position[0];
            newExteriorVertex = vertex;
        }
    }
    packingAttempt2.updateExteriorFace(newExteriorVertex->leaving->twin->face);

    // DCEL viewer for modification
    ObjectViewer viewWindow(packingAttempt2);
    viewWindow.display();

    // Run packing algorithm
    packingAttempt2.pack();

    // Display circle packing
    PackingDisplay packingDisplay(packingAttempt2);
    packingDisplay.display();

    // Graphics for Polyhedron (3D)
    // SteinitzPolyhedron poly(packingAttempt2);
    // viewWindow.rotating = true;
    // viewWindow.display();

}