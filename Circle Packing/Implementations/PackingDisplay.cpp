#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <memory>

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
using std::unique_ptr;


PackingDisplay::PackingDisplay(CirclePacking& obj) :
    scale(300), 
    width(1000), 
    height(850) {
    
    object = &obj;

    recomputeShapes();
}


void PackingDisplay::recomputeShapes() {

    drawableShapes.clear();
    computeCircles();
    computeUnderlyingGraph();

}

void PackingDisplay::computeCircles() {

    for (int i=0 ; i<object->centers.size() ; i++) {

        double radius = object->centers[i]->radius * scale;
        array<double, 2> center = toWindowCoords({object->centers[i]->position[0],
                                                  object->centers[i]->position[1]});

        // FLIP ACROSS Y=X
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
}

void PackingDisplay::computeUnderlyingGraph() {

    for (Face* face : object->object->faces) {

        HalfEdge* start = face->edge;
        HalfEdge* current = start;

        sf::ConvexShape* convex = new sf::ConvexShape();
        convex->setOutlineThickness(1);
        convex->setOutlineColor(sf::Color::Black);
        convex->setFillColor(sf::Color::Transparent);

        vector<array<double, 2>> points;

        do {

            points.push_back(toWindowCoords({current->origin->position[0],current->origin->position[1]}));
            current = current->next;

        } while ( current != start );

        convex->setPointCount(points.size());

        for (int i=0 ; i<points.size() ; i++) {
            convex->setPoint(i, {points[i][0], points[i][1]});
        }

        drawableShapes.push_back(unique_ptr<sf::Drawable>(convex));

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

        //Display circle Packing
        for (const auto& drawablePtr : drawableShapes) {

            window.draw(*drawablePtr);

        }

        //Display underlying graph of circle centers
        // for (Face* face : object->object->faces) {

        //     HalfEdge* start = face->edge;
        //     HalfEdge* current = start;

        //     sf::ConvexShape convex;
        //     convex.setOutlineThickness(1);
        //     convex.setOutlineColor(sf::Color::Black);
        //     convex.setFillColor(sf::Color::Transparent);

        //     vector<array<double, 2>> points;

        //     do {

        //         points.push_back(toWindowCoords({current->origin->position[0],current->origin->position[1]}));
        //         current = current->next;

        //     } while ( current != start );

        //     convex.setPointCount(points.size());

        //     for (int i=0 ; i<points.size() ; i++) {
        //         convex.setPoint(i, {points[i][0], points[i][1]});
        //     }

        //     window.draw(convex);

        // }

        //Display Incircles
        // for (const Face* face : object->object->faces) {

        //     double radius = face->inradius * scale;

        //     double a = face->edge->length();
        //     double b = face->edge->next->length();
        //     double c = face->edge->next->next->length();

        //     auto oa = face->edge->next->next->origin->position;
        //     auto ob = face->edge->origin->position;
        //     auto oc = face->edge->next->origin->position;

        //     array<double, 2> center = {(a*oa[0] + b*ob[0] + c*oc[0]) / (a+b+c), (a*oa[1] + b*ob[1] + c*oc[1]) / (a+b+c)};

        //     center = toWindowCoords(center);

        //     // array<double, 3> center = toWindowCoords(face->centroid());
        //     sf::Vector2f position(
        //         center[1]-radius, center[0]-radius
        //     );

        //     sf::CircleShape drawableCircle(radius);

        //     drawableCircle.setPosition(position);

        //     drawableCircle.setOutlineThickness(1);
        //     drawableCircle.setOutlineColor(sf::Color::Black);
        //     drawableCircle.setFillColor(sf::Color::Transparent);

        //     window.draw(drawableCircle);

        // }

        window.display();
    }

}


array<double, 2> PackingDisplay::toWindowCoords(array<double, 2> coords) {

    double x = scale * coords[0] + (width/2.0);
    double y = -scale * coords[1] + (height/2.0);

    return {x, y};

}

array<double, 2> PackingDisplay::toObjectCoords(array<double, 2> coords) {

    double x = (coords[0] - (width/2.0)) / scale;
    double y = -(coords[1] - (height/2.0)) / scale;

    return {x, y};

}