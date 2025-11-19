#include "DCEL/Interfaces/ObjectViewer.h"
#include "DCEL/Interfaces/DCEL.h"

#include "Graphs/Interfaces/SimpleGraph.h"
#include "Graphs/Interfaces/PlanarEmbedding.h"

#include "Circle Packing/Interfaces/Circle.h"
#include "Circle Packing/Interfaces/CirclePacking.h"
#include "Circle Packing/Interfaces/PackingDisplay.h"
#include "Circle Packing/Interfaces/SteinitzPolyhedron.h"

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
    // Create DCEL Structure of plane graph
    DCEL embed(planar); 

    // Choose a different exterior face for Herschels Enneahedron
    double minX = 100;
    Vertex* newExteriorVertex;
    for (Vertex* vertex : embed.exteriorVertices) {

        if (vertex->position[0] < minX) {

            minX = vertex->position[0];
            newExteriorVertex = vertex;
        }
    }
    embed.updateExteriorFace(newExteriorVertex->leaving->twin->face);

    // Graph GUI for modification
    ObjectViewer viewWindow(embed);
    viewWindow.display();
    
    // Pack the modified graph
    CirclePacking packingAttempt1(embed);

    // Display circle packing
    PackingDisplay packingDisplay(packingAttempt1);
    packingDisplay.display();

    // Make Steinitz Polyhedron
    SteinitzPolyhedron poly(packingAttempt1);

    // viewWindow.setScale(400);
    viewWindow.rotating = true;
    viewWindow.display();

}