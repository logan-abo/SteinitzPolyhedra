#ifndef PACKINGDISPLAY_H
#define PACKINGDISPLAY_H

#include <vector>
#include <array>
#include <memory>

using std::vector;
using std::array;
using std::unique_ptr;

#include <SFML/Graphics.hpp>


class CirclePacking;

class PackingDisplay {    

    private:

        double scale;

        int width;
        int height;

        array<double, 2> toWindowCoords(array<double, 2> coord);
        array<double, 2> toObjectCoords(array<double, 2> coord);

        void recomputeShapes();
        void computeCircles();
        void computeUnderlyingGraph();
        void computeIncircles();

        CirclePacking* object;
        vector<unique_ptr<sf::Drawable>> drawableShapes;

    public:

        PackingDisplay(CirclePacking& obj);

        void display();

};

#endif