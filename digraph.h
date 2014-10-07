#ifndef DIGRAPH_H
#define DIGRAPH_H

#include "simple_set.h"
#include <assert.h>
#include <vector>
#include <utility>

namespace skl {

// Directed graph without weight
class digraph
{
public:
    typedef simple_set<int> row;
    typedef row::iterator edge_iterator;
    typedef edge_iterator const_edge_iterator;

    digraph() { }

    explicit digraph(std::size_t v)
        : adj_(v)
    {
    }

    inline std::size_t vertexes() const
    {
        return adj_.size();
    }

    inline void add_edge(int from, int to)
    {
        adj_[from].add(to);
    }

    inline void remove_edge(int from, int to)
    {
        adj_[from].erase(to);
    }

    inline edge_iterator edge_begin(int v) const
    {
        return adj_[v].begin();
    }

    inline edge_iterator edge_end(int v) const
    {
        return adj_[v].end();
    }

    digraph reverse() const
    {
        int size = adj_.size();
        digraph rev(size);

        for (int i = 0; i < size; ++i)
        {
            const auto &r = adj_[i];
            for (auto v : r)
                rev.add_edge(v, i);
        }

        return std::move(rev);
    }

private:
    std::vector<row> adj_;
};

} // namespace skl

#endif // DIGRAPH_H
