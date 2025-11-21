
#include "Graph.h"

class SimpleGraph : public Graph {

    private:
    
        vector<vector<int>> adjacencyMatrix;

    public:

        SimpleGraph(const vector<vector<int>>& adjacency);

        int order() const override;
        int size() const override;

        vector<int> neighbors(int v) const override;
        int degree(int v) const override;

};