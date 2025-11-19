#include "../Interfaces/SimpleGraph.h"

#include <numeric>


SimpleGraph::SimpleGraph(const vector<vector<int>>& adjacency) {

    adjacencyMatrix = adjacency;

}


int SimpleGraph::order() const {
    
    return adjacencyMatrix.size();

}

int SimpleGraph::size() const {

    int degreeSum = std::accumulate(adjacencyMatrix.begin(),
                                    adjacencyMatrix.end(),
        0,
        
        [](int currentSum, const std::vector<int>& innerVec) {
            return std::accumulate(innerVec.begin(), innerVec.end(), currentSum);
        }
    );

    return degreeSum/2;

}


vector<int> SimpleGraph::neighbors(int v) const {

    vector<int> adjacencies;

    for (int i=0 ; i<adjacencyMatrix[v].size() ; i++){
        if (adjacencyMatrix[v][i]==1) {
            adjacencies.push_back(i);
        }
    }

    return adjacencies;
}

int SimpleGraph::degree(int v) const {
    
    const vector<int>& adjacencies = adjacencyMatrix[v];

    return std::accumulate(adjacencies.begin(), 
                           adjacencies.end(), 
                           0);
}