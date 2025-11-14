#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <iostream>

#include <SFML/Graphics.hpp>

#include "../../Graphs/Interfaces/PlanarEmbedding.h"
#include "../../DCEL/Interfaces/Vertex.h"
#include "../../DCEL/Interfaces/HalfEdge.h"
#include "../../DCEL/Interfaces/DCEL.h"
#include "../Interfaces/PackingDisplay.h"
#include "../Interfaces/CirclePacking.h"
#include "../Interfaces/Circle.h"

using std::vector;
using std::array;


PackingDisplay::PackingDisplay(CirclePacking& obj) :
    scale(200), 
    width(1000), 
    height(850) {
    
    object = &obj;

    computeShapes();
}


void PackingDisplay::computeShapes() {

    circleShapes.clear();

    for (int i=0 ; i<object->centers.size() ; i++) {

        double radius = object->centers[i]->radius * scale;
        array<double, 2> center = toWindowCoords({object->centers[i]->position[0],
                                                  object->centers[i]->position[1]});

        // FLIP ACROSS Y=X
        sf::Vector2f position(
            center[1]-radius, center[0]-radius
        );

        sf::CircleShape drawableCircle(radius);

        drawableCircle.setPosition(position);

        drawableCircle.setOutlineThickness(1);
        drawableCircle.setOutlineColor(sf::Color::Black);
        drawableCircle.setFillColor(sf::Color::Transparent);

        circleShapes.push_back(drawableCircle);

        // return;
    }
}


void PackingDisplay::display() {

    sf::RenderWindow window(sf::VideoMode({width, height}), "Circle Packing");

    while (window.isOpen()) {

        while (const std::optional event = window.pollEvent()) {

            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        window.clear(sf::Color::White);

        for (const sf::CircleShape& circle : circleShapes) {

            window.draw(circle);

        }

        for (Face* face : object->object->faces) {

            HalfEdge* start = face->edge;
            HalfEdge* current = start;

            sf::ConvexShape convex;
            convex.setOutlineThickness(1);
            convex.setOutlineColor(sf::Color::Black);
            convex.setFillColor(sf::Color::Transparent);

            vector<array<double, 2>> points;

            do {

                points.push_back(toWindowCoords({current->origin->position[0],current->origin->position[1]}));
                current = current->next;

            } while ( current != start );

            convex.setPointCount(points.size());

            // FLIP ACROSS Y=X
            for (int i=0 ; i<points.size() ; i++) {
                convex.setPoint(i, {points[i][1], points[i][0]});
            }

            window.draw(convex);

        }

        window.display();
    }

}


array<double, 2> PackingDisplay::toWindowCoords(array<double, 2> coords) {

    double x = scale * coords[0] + (width/2.0);
    double y = scale * coords[1] + (height/2.0);

    return {x, y};

}

array<double, 2> PackingDisplay::toObjectCoords(array<double, 2> coords) {

    double x = (coords[0] - (width/2.0)) / scale;
    double y = (coords[1] - (height/2.0)) / scale;

    return {x, y};

}