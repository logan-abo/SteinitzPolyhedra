#ifndef PACKINGDISPLAY_H
#define PACKINGDISPLAY_H

#include <vector>
#include <array>

using std::vector;
using std::array;

#include <SFML/Graphics.hpp>


class CirclePacking;

class PackingDisplay {    

    private:

        double scale;

        int width;
        int height;

        array<double, 2> toWindowCoords(array<double, 2> coord);
        array<double, 2> toObjectCoords(array<double, 2> coord);

        void computeShapes();

        CirclePacking* object;
        vector<sf::CircleShape> circleShapes;

    public:

        PackingDisplay(CirclePacking& obj);

        void display();

};

#endif