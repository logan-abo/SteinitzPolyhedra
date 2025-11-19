#include "Graph.h"

#include <cstddef>
#include <cmath>
#include <new>

//Copies given adjacency matrix for Dipgraph object
Graph::Graph(const std::vector<std::vector<int>>& matrix) 
    : order(matrix.size()), adjacencyMatrix(matrix) {}

int Graph::get(size_t row, size_t col) const {

    return adjacencyMatrix[row][col];

}

int Graph::rowSum(size_t row) const {
    int sum = 0;

    for (size_t i=0 ; i<order ; i++) {
        sum += adjacencyMatrix[row][i];
    }

    return sum;
}

int Graph::colSum(size_t col) const {

    int sum = 0;

    for (size_t i=0 ; i<order ; i++) {
        sum += adjacencyMatrix[i][col];
    }

    return sum;
}

bool Graph::isBalanced(size_t index) const {

    return rowSum(index)==colSum(index);

}

void balance(Graph g) {
    
    size_t vertex = 0;

    if (std::abs(g.rowSum(vertex) - g.colSum(vertex)) > 1) {

        //find path
        return;
        
    }

}
