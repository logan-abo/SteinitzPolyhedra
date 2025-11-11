#ifndef CIRCLEPACKING_H
#define CIRCLEPACKING_H

#include <vector>
#include <array>
#include <unordered_map>

#include <Eigen/Dense>

using std::vector;
using std::array;
using std::unordered_map;

class Circle;
class PlanarEmbedding;
class DCEL;
class HalfEdge;
class Vertex;

class CirclePacking {

    private:

        PlanarEmbedding* embedding;
        vector<vector<int>> edgeConductance;

        unordered_map<Vertex*, int> vertexLookup;

        void createAdjacencyMatrix();

        //Step B
        void computeEffectiveRadii(); //DONE
        double sectorRadius(HalfEdge* counterClockwiseMostEdge) const; //DONE

        //Step A
        void placeExteriorCircles(); //DONE
        double estimateBoundingRadius() const; //DONE
        double sumExteriorOverRho(double rho) const; //DONE

        //Step C
        void placeInteriorCircles();
        void computeInradii(); //DONE
        void computeEdgeConductance(); //DONE
        Eigen::MatrixXd interiorTransitionProbabilities() const;
        Eigen::MatrixXd exteriorTransitionProbabilities() const;


        //Do one full iteration of packing approximation algorithm
        void approximationStep() {
            placeExteriorCircles();
            placeInteriorCircles();
            computeEffectiveRadii();
        }
        

        int interiorVertexCount;

    public: 

        CirclePacking(DCEL& planeGraph);
        
        DCEL* object;

        vector<Vertex*> centers;

};

#endif