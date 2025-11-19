#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <cstddef>

using std::vector;

class Graph {

    private:

    public:

        virtual int order() const = 0;
        virtual int size() const = 0;

        virtual vector<int> neighbors(int v) const = 0;
        virtual int degree(int v) const = 0;

};

#endif