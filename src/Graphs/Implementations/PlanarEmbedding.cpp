#include "../Interfaces/PlanarEmbedding.h"
#include "../Interfaces/SimpleGraph.h"

#include "../../DCEL/Interfaces/DCEL.h"
#include "../../DCEL/Interfaces/Vertex.h"
#include "../../DCEL/Interfaces/HalfEdge.h"

#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>
#include <array>
#include <cmath>

using std::vector;
using std::array;



PlanarEmbedding::PlanarEmbedding(const DCEL& dcel) {

    vector<Vertex*> sortedVertices;

    //Add all Interior vertices first
    for (Face* face : dcel.faces) {
        if (! face->isExterior) {
            for (Vertex* vertex : face->vertices()) {

                sortedVertices.push_back(vertex);
            }
        }
    }
    //Then add all Exterior vertices
    for (Face* face : dcel.faces) {
        if (face->isExterior) {
            for (Vertex* vertex : face->vertices()) {

                sortedVertices.push_back(vertex);
            }
            numExteriorVertices = face->vertices().size();
            break;
        }
    }

    for (int i=0 ; i<dcel.vertices.size() ; i++) {
        vector<int> empty;
        adjacencyList.push_back(empty);
    }

    for (HalfEdge* edge : dcel.edges) {

        // int index = std::find();
        // adjacencyList[]

    }


}

PlanarEmbedding::PlanarEmbedding(const SimpleGraph& g,
                                 const vector<array<double, 3>>& points) {

    if (points.size() != g.order()) {
        throw std::runtime_error("Number of points provided does not match size of given graph: " +
                                 std::to_string(points.size()) + 
                                 " =/= " + 
                                 std::to_string(g.size())
        );
    }

    for (int i=0 ; i<points.size() ; i++) {

        const auto& c = points[i];
        vertices.push_back(c);
        
        vector<std::pair<double, int>> sorter;

        for (int neighbor : g.neighbors(i)) {

            const auto& p = points[neighbor];
            double angle = atan2(p[1]-c[1], p[0]-c[0]);

            sorter.push_back({angle, neighbor});

        }

        std::sort(sorter.begin(), sorter.end(), std::less<std::pair<double, int>>());

        vector<int> clockwiseNeighbors;
        for (const auto& pair : sorter) {
            clockwiseNeighbors.push_back(pair.second);
        }

        adjacencyList.push_back(clockwiseNeighbors);

    }
}

int PlanarEmbedding::clockwiseNextFrom(int vertex, int previous) const {

    const vector<int>& allNeighbors = adjacencyList[vertex];

    auto previousIndex = std::find(allNeighbors.begin(), 
                                  allNeighbors.end(),
                                  previous);

    if (previousIndex==allNeighbors.end()) {
        return -1;
    }

    if (*previousIndex==allNeighbors[allNeighbors.size()-1]) {
        return allNeighbors[0];
    }
    return *(++previousIndex);

}


int PlanarEmbedding::order() const {
    return adjacencyList.size();
}

int PlanarEmbedding::size() const {
    int degreeCount = 0;
    for (const vector<int>& adjacencies : adjacencyList) {
        degreeCount += adjacencies.size();
    }
    return degreeCount/2;
}

array<double, 3> PlanarEmbedding::vertex(int v) const {
    return vertices[v];
}

vector<int> PlanarEmbedding::neighbors(int v) const {
    return adjacencyList[v];
}

int PlanarEmbedding::degree(int v) const {
    return adjacencyList[v].size();
}

vector<vector<int>> PlanarEmbedding::adjacencyCopy() const {
    return adjacencyList;
}