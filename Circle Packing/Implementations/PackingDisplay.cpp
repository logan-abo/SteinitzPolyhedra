#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <iostream>

#include <SFML/Graphics.hpp>

#include "../../Graphs/Interfaces/PlanarEmbedding.h"
#include "../../DCEL/Interfaces/Vertex.h"
#include "../Interfaces/PackingDisplay.h"
#include "../Interfaces/CirclePacking.h"
#include "../Interfaces/Circle.h"

using std::vector;
using std::array;


PackingDisplay::PackingDisplay(CirclePacking& obj) :
    scale(100), 
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

        sf::Vector2f position(
            center[0]-radius, center[1]-radius
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

        for (const sf::CircleShape& circle : circleShapes) {

            window.draw(circle);

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