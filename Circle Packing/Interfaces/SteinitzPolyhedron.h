#ifndef STEINITZPOLYHEDRON_H
#define STEINITZPOLYHEDRON_H

#include <vector>
#include <array>
#include <unordered_map>
#include <memory>

using std::vector;
using std::array;
using std::unordered_map;
using std::unique_ptr;

class Plane;
class HalfEdge;
class CirclePacking;

class SteinitzPolyhedron {

    private:

        unordered_map<HalfEdge*, array<double, 3>> tangencies();

        // CIRCLE PACKING SHOULD EXTEND DCEL
        CirclePacking* packed;

        void projectToSphere();


    public:

        SteinitzPolyhedron(CirclePacking& packing);

};

#endif