#include <iostream>
#include <fstream>
#include <vector>
#include <array>

#include <SFML/Graphics.hpp>

#include "../Interfaces/ObjectViewer.h"
#include "../Interfaces/DCEL.h"
#include "../Interfaces/Face.h"

using std::vector;
using std::array;


ObjectViewer::ObjectViewer(DCEL& obj) :
    scale(100), 
    width(1000), 
    height(850) {
    
    object = &obj;

    computeFaces();
}


void ObjectViewer::computeFaces() {

    faceShapes.clear();

    for (Face* face : object->faces) {

        if (! face->isExterior) {
            HalfEdge* start = face->edge;
            HalfEdge* current = start;

            sf::ConvexShape convex;
            convex.setOutlineThickness(1);
            convex.setOutlineColor(sf::Color::Black);
            convex.setFillColor(sf::Color::Transparent);

            vector<array<double, 3>> points;

            do {

                points.push_back(toWindowCoords(current->origin->position));
                current = current->next;

            } while ( current != start );

            convex.setPointCount(points.size());

            for (int i=0 ; i<points.size() ; i++) {
                convex.setPoint(i, {points[i][1], points[i][0]});
            }

            faceShapes.push_back(convex);
        }

    }

}

void ObjectViewer::display() {

    sf::RenderWindow window(sf::VideoMode({width, height}), "Object Viewer");

    while (window.isOpen()) {

        while (const std::optional event = window.pollEvent()) {

            if (event->is<sf::Event::Closed>()) {
                window.close();
            }

            // if (event->is<sf::Event::MouseButtonPressed>()) {

            //     int mouseX = sf::Mouse::getPosition(window).x;
            //     int mouseY = sf::Mouse::getPosition(window).y;

            //     triangulate(mouseX, mouseY);

            // }

        }

        window.clear(sf::Color::White);

        for (const sf::ConvexShape& face : faceShapes) {

            window.draw(face);

        }

        window.display();
    }

}


array<double, 3> ObjectViewer::toWindowCoords(array<double, 3> coords) {

    double x = scale * coords[0] + (width/2.0);
    double y = scale * coords[1] + (height/2.0);

    return {x, y, coords[2]};

}

array<double, 3> ObjectViewer::toObjectCoords(array<double, 3> coords) {

    double x = (coords[0] - (width/2.0))/scale;
    double y = (coords[1] - (height/2.0))/scale;

    return {x, y, coords[2]};

}


void ObjectViewer::triangulate(int x, int y) {

    sf::Vector2f pos(x, y);

    for (int i=faceShapes.size()-1 ; i>=0 ; i--) {

        if (faceShapes[i].getGlobalBounds().contains(pos)) {

            object->triangulate(i);
            computeFaces();

            return;

        }

    }

}
