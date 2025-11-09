#ifndef OBJECTVIEWER_H
#define OBJECTVIEWER_H

#include <vector>
#include <array>

using std::vector;
using std::array;

#include <SFML/Graphics.hpp>

class DCEL;
class Face;

class ObjectViewer {    

    private:

        double scale;

        int width;
        int height;

        array<double, 3> toWindowCoords(array<double, 3> coords);
        array<double, 3> toObjectCoords(array<double, 3> coords);

        void computeFaces();
        void triangulate(int x, int y);

        DCEL* object;
        vector<sf::ConvexShape> faceShapes;

    public:

        ObjectViewer(DCEL& obj);

        void display();

};

#endif