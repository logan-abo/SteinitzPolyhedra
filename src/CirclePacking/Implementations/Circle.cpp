#include "../Interfaces/Circle.h"

#include <vector>
#include <cmath>
#include <array>

using std::vector;
using std::array;

Circle::Circle() {

    r = 1;
    x = 0;
    y = 0;

}

Circle::Circle(array<double, 2> center,
               double radius) {
    x = center[0];
    y = center[1];
    r = radius;
}

Circle::Circle(double radius, 
               double xCoord, 
               double yCoord) 
    : r(radius), x(xCoord), y(yCoord) {}

array<double, 2> Circle::getCenter() const {
    return {x, y};
}

double Circle::getX() const {
    return x;
}
double Circle::getY() const {
    return y;
}
double Circle::radius() const {
    return r;
}

vector<double> Circle::at(double t) const {

    vector<double> coords = {
        x+r*cos(t), 
        y+r*sin(t)
    };

    return coords;

}
vector<double> Circle::mapAt(double t) const {

    vector<double> c = at(t);

    return j(c[0], c[1]);

}

vector<double> Circle::j(double x, double y) const {

    double D = 1 + x*x + y*y;
    vector<double> inverseStereographic = {
        2*x/D, 
        2*y/D, 
        (D-2)/D
    };

    return inverseStereographic;

}

bool tangent(Circle c1, Circle c2) {

    double xDif = c1.getX()-c2.getX();
    double yDif = c1.getY()-c2.getY();

    double dist = sqrt((xDif*xDif)+(yDif*yDif));

    return dist == (c1.radius()+c2.radius());

}

