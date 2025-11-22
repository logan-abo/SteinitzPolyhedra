#ifndef CIRCLEPACKING_H
#define CIRCLEPACKING_H

#include "../../DCEL/Interfaces/DCEL.h"

#include <vector>
#include <array>
#include <unordered_map>
#include <iostream>

#include <Eigen/Dense>

using std::vector;
using std::array;
using std::unordered_map;

class CircleVertex;
class ConductanceEdge;
class PlanarEmbedding;
class DCEL;
class HalfEdge;
class Vertex;

class CirclePacking : public DCEL {

    private:

        PlanarEmbedding* embedding;

        // Vertices need to be sorted interior first, then exterior
        void sortVertices();

        // To avoid mass templating
        CircleVertex* cast(Vertex* vertex) const;
        ConductanceEdge* cast(HalfEdge* edge) const;

        // Override Factory methods
        Vertex* allocateVertex(array<double, 3> coords) override;
        HalfEdge* allocateHalfEdge(Vertex* vertex) override;

        int interiorVertexCount;

        // This should exist but returns to issue of desync with DCEL structure
        // unordered_map<Vertex*, int> vertexLookup;

        //Step C
        void computeEffectiveRadii(); //DONE
        double sectorRadius(HalfEdge* counterClockwiseMostEdge) const; //DONE

        //Step A
        void placeExteriorCircles(); //DONE
        void scaleToUnitDisc(double boundingRadius);
        double estimateBoundingRadius() const; //DONE
        double sumExteriorOverRho(double rho) const; //DONE

        //Step B
        void placeInteriorCircles();
        void computeInradii(); //DONE
        void computeEdgeConductance(); //DONE
        Eigen::MatrixXd transitionProbabilities() const;


        //Do one full iteration of packing approximation algorithm
        void approximationStep() {
            placeExteriorCircles();
            placeInteriorCircles();
            computeEffectiveRadii();
        }


    public: 

        CirclePacking(const PlanarEmbedding& graph);

        double getRadius(Vertex* vertex) const;

        void pack();

};

#endif