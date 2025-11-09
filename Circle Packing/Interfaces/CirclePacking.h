#ifndef CIRCLEPACKING_H
#define CIRCLEPACKING_H

#include <vector>
#include <array>

using std::vector;
using std::array;

class Circle;
class PlanarEmbedding;
class DCEL;
class HalfEdge;
class Vertex;

class CirclePacking {

    private:

        DCEL* object;
        PlanarEmbedding* embedding;

        vector<double> aims;

        void computeEffectiveRadii(); //DONE
        double sectorRadius(HalfEdge* counterClockwiseMostEdge) const; //DONE

        // double aim(int vertexIndex) const; //DONE

        double estimateBoundingRadius() const; //DONE
        double sumExteriorOverRho(double rho) const; //DONE

        void placeExteriorCircles(); //DONE

        
        void computeInradii() const;


        void createInitialSectors();

        int interiorVertexCount;

    public: 

        vector<Vertex*> centers;

        CirclePacking(DCEL& planeGraph);

};

#endif