#include <vector>

using std::vector;

#include "Graph.h"


class SimpleGraph {

    private:
    
        vector<vector<int>> adjacencyMatrix;

    public:

        SimpleGraph(const vector<vector<int>>& adjacency);

        int order() const;
        int size() const;

        vector<int> neighbors(int v) const;
        int degree(int v) const;

};