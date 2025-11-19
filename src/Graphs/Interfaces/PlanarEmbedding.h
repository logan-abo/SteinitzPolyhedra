#ifndef PLANAREMBEDDING_H
#define PLANAREMBEDDING_H

#include <vector>
#include <array>

using std::vector;
using std::array;

class DCEL;
class SimpleGraph;

class PlanarEmbedding {

    private:

        int numExteriorVertices;

        vector<vector<int>> adjacencyList;
        vector<array<double, 3>> vertices;


    public: 


        PlanarEmbedding(const DCEL& dcel);

        PlanarEmbedding(const SimpleGraph& g, 
                        const vector<array<double, 3>>& points);
        

        int clockwiseNextFrom(int vertex, int previous) const;

        array<double, 3> vertex(int u) const;


        int order() const;
        int size() const;

        vector<int> neighbors(int v) const;
        int degree(int v) const;

        vector<vector<int>> adjacencyCopy() const;

};

#endif