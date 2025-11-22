#include <iostream>
#include <fstream>
#include <vector>
#include <array>

#include <SFML/Graphics.hpp>

#include "../Interfaces/ObjectViewer.h"
#include "../Interfaces/Vertex.h"
#include "../Interfaces/HalfEdge.h"
#include "../Interfaces/Face.h"
#include "../Interfaces/DCEL.h"

using std::vector;
using std::array;


ObjectViewer::ObjectViewer(DCEL& obj) :
    scale(150), 
    width(1000), 
    height(850) {
    
    object = &obj;

    computeFaces();
}

void ObjectViewer::recomputeDisplayObjects() {
    computeFaces();
    std::cout << "computed" << std::endl;
    computeIncircles();
}

void ObjectViewer::computeFaces() {

    faceShapes.clear();

    for (Face* face : object->faces) {

        if (! face->isExterior) {

            sf::ConvexShape convex;
            convex.setOutlineThickness(1);
            convex.setOutlineColor(sf::Color::Black);
            convex.setFillColor(sf::Color::Transparent);

            vector<array<double, 3>> points;

            HalfEdge* start = face->edge;
            HalfEdge* current = start;

            do {

                points.push_back(toWindowCoords(current->origin->position));
                current = current->next;

            } while ( current != start );

            convex.setPointCount(points.size());

            for (int i=0 ; i<points.size() ; i++) {
                convex.setPoint(i, {points[i][0], points[i][1]});
            }

            faceShapes.push_back(convex);
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

        sf::CircleShape drawableCircle(radius);

        drawableCircle.setPosition(position);

        drawableCircle.setOutlineThickness(1);
        drawableCircle.setOutlineColor(sf::Color::Black);
        drawableCircle.setFillColor(sf::Color::Transparent);

        incircles.push_back(drawableCircle);

    }

};


void ObjectViewer::display() {

    sf::RenderWindow window(sf::VideoMode({width, height}), "Object Viewer");

    while (window.isOpen()) {

        while (const std::optional event = window.pollEvent()) {

            if (event->is<sf::Event::Closed>()) {
                window.close();
            }

            if (event->is<sf::Event::MouseButtonPressed>()) {

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

        for (const sf::ConvexShape& face : faceShapes) {

            window.draw(face);

        }

        // Debug "leaving" pointer
        for (Vertex* vertex : object->exteriorVertices) {
            auto u = toWindowCoords(vertex->position);
            auto v = toWindowCoords(vertex->leaving->twin->origin->position);

            sf::Vertex line[2];
            line[0].position = sf::Vector2f(u[0], u[1]);
            line[0].color  = sf::Color::Red;
            line[1].position = sf::Vector2f(v[0], v[1]);
            line[1].color = sf::Color::Blue;

            window.draw(line, 2, sf::PrimitiveType::Lines);
        }

        for (const sf::CircleShape& incircle : incircles) {

            window.draw(incircle);

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
