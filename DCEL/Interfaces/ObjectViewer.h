#ifndef OBJECTVIEWER_H
#define OBJECTVIEWER_H

#include <vector>
#include <array>
#include <memory>

using std::vector;
using std::array;
using std::unique_ptr;

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

        void recomputeDisplayObjects();
        void computeFaces();
        void computeIncircles();

        void triangulate(int x, int y);

        DCEL* object;
        vector<unique_ptr<sf::Drawable>> drawableShapes;

    public:

        ObjectViewer(DCEL& obj);

        void setScale(double newScale);

        void display();

        bool rotating = false;

};

#endif