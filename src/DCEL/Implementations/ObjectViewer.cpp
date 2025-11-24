#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <cmath>

#include <SFML/Graphics.hpp>

#include "../Interfaces/ObjectViewer.h"
#include "../Interfaces/Vertex.h"
#include "../Interfaces/HalfEdge.h"
#include "../Interfaces/Face.h"
#include "../Interfaces/DCEL.h"

using std::vector;
using std::array;

std::array<double, 3> rotate(std::array<double, 3> vector, double theta, double psi) {

    // Pre-calculate sines and cosines for the angles
    double cos_theta = std::cos(theta); // Y-axis rotation (Yaw)
    double sin_theta = std::sin(theta);
    double cos_psi   = std::cos(psi);   // X-axis rotation (Pitch)
    double sin_psi   = std::sin(psi);

    // Get original coordinates
    double x = vector[0];
    double y = vector[1];
    double z = vector[2];

    // --- Step 1: Apply rotation around Y-axis (theta) ---
    // Rotation matrix for Y:
    // [ cos(t)  0   sin(t) ]
    // [   0     1     0    ]
    // [-sin(t)  0   cos(t) ]
    double x_prime = x * cos_theta + z * sin_theta;
    double y_prime = y;
    double z_prime = -x * sin_theta + z * cos_theta;

    // --- Step 2: Apply rotation around X-axis (psi) to the intermediate result ---
    // Rotation matrix for X:
    // [ 1     0       0    ]
    // [ 0   cos(p)  -sin(p) ]
    // [ 0   sin(p)   cos(p) ]
    double x_final = x_prime;
    double y_final = y_prime * cos_psi - z_prime * sin_psi;
    double z_final = y_prime * sin_psi + z_prime * cos_psi;

    // Return the final rotated vector
    return {x_final, y_final, z_final};
}

ObjectViewer::ObjectViewer(DCEL& obj) :
    scale(150), 
    width(1000), 
    height(850) {
    
    object = &obj;
}

void ObjectViewer::setScale(double newScale) {
    scale = newScale;
}

void ObjectViewer::recomputeDisplayObjects() {

    drawableShapes.clear();

    computeFaces();
    // computeIncircles();
}

void ObjectViewer::computeFaces() {

    if (rotating) {
        for (Vertex* vertex : object->vertices) {

            vertex->position = rotate(vertex->position, 0.02, 0.03);
        }
    }

    for (Face* face : object->faces) {

        if (face->normal()[2] > 0) {

            sf::ConvexShape* convex = new sf::ConvexShape();
            convex->setOutlineThickness(1);
            convex->setOutlineColor(sf::Color::Black);
            convex->setFillColor(sf::Color::Transparent);

            vector<array<double, 3>> points;

            HalfEdge* start = face->edge;
            HalfEdge* current = start;

            do {

                points.push_back(toWindowCoords(current->origin->position));
                current = current->next;

            } while ( current != start );

            convex->setPointCount(points.size());

            for (int i=0 ; i<points.size() ; i++) {

                convex->setPoint(i, {points[i][0], points[i][1]});
            }

            drawableShapes.push_back(unique_ptr<sf::Drawable>(convex));
        }

    }

}

void ObjectViewer::computeIncircles() {

    for (const Face* face : object->faces) {

        double radius = face->inradius * scale;

        double a = face->edge->length();
        double b = face->edge->next->length();
        double c = face->edge->next->next->length();

        auto oa = face->edge->next->next->origin->position;
        auto ob = face->edge->origin->position;
        auto oc = face->edge->next->origin->position;

        array<double, 3> center = {(a*oa[0] + b*ob[0] + c*oc[0]) / (a+b+c), 
                                   (a*oa[1] + b*ob[1] + c*oc[1]) / (a+b+c), 0};
        center = toWindowCoords(center);

        sf::Vector2f position(
            center[0]-radius, center[1]-radius
        );

        sf::CircleShape* drawableCircle = new sf::CircleShape(radius);

        drawableCircle->setPosition(position);

        drawableCircle->setOutlineThickness(1);
        drawableCircle->setOutlineColor(sf::Color::Black);
        drawableCircle->setFillColor(sf::Color::Transparent);

        drawableShapes.push_back(unique_ptr<sf::Drawable>(drawableCircle));

    }

};


void ObjectViewer::display() {

    recomputeDisplayObjects();

    sf::RenderWindow window(sf::VideoMode({width, height}), "Object Viewer");
    window.setFramerateLimit(60);

    while (window.isOpen()) {

        sf::Event event;
        while (window.pollEvent(event)) {

            if (event.type == sf::Event::Closed) {
                window.close();
            }

            if (event.type == sf::Event::MouseButtonPressed) {

                int mouseX = sf::Mouse::getPosition(window).x;
                int mouseY = sf::Mouse::getPosition(window).y;

                array<double, 3> coords = toObjectCoords({
                    (double)mouseX, 
                    (double)mouseY, 
                    0
                });

                object->addVertex(coords);
                recomputeDisplayObjects();

            }

        }

        window.clear(sf::Color::White);

        recomputeDisplayObjects();

        for (const auto& drawable : drawableShapes) {

            window.draw(*drawable);

        }

        window.display();
    }

}


array<double, 3> ObjectViewer::toWindowCoords(array<double, 3> coords) {

    double x = scale * coords[0] + (width/2.0);
    double y = -scale * coords[1] + (height/2.0);

    return {x, y, coords[2]};

}

array<double, 3> ObjectViewer::toObjectCoords(array<double, 3> coords) {

    double x = (coords[0] - (width/2.0))/scale;
    double y = -(coords[1] - (height/2.0))/scale;

    return {x, y, coords[2]};

}



// void ObjectViewer::triangulate(int x, int y) {
// 
//     sf::Vector2f pos(x, y);
// 
//     for (int i=faceShapes.size()-1 ; i>=0 ; i--) {
// 
//         if (faceShapes[i].getGlobalBounds().contains(pos)) {
// 
//             object->triangulate(i);
//             computeFaces();
// 
//             return;
// 
//         }
// 
//     }
// 
// }
