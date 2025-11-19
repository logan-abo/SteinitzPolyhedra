#ifndef CIRCLE_H
#define CIRCLE_H

#include <vector>
#include <array>

using std::vector;
using std::array;

class Circle {

    public: 

        Circle();
        Circle(array<double, 2> center, 
               double radius);

        Circle(double radius, 
               double xCoord, 
               double yCoord);

        array<double, 2> getCenter() const;

        double getX() const;
        double getY() const;
        double radius() const;

        vector<double> at(double t) const;
        vector<double> mapAt(double t) const;

    private:

        double r;
        double x;
        double y;

        vector<double> j(double x, double y) const;

};

bool tangent(Circle c1, Circle c2);


#endif