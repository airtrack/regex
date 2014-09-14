#ifndef DIGRAPH_H
#define DIGRAPH_H

#include "simple_set.h"
#include <vector>
#include <utility>

namespace skl {

// Directed graph without weight
class digraph
{
public:
    typedef simple_set<int> row;
    typedef typename row::iterator edge_iterator;
    typedef edge_iterator const_edge_iterator;

    explicit digraph(std::size_t v)
        : adj_(v)
    {
    }

    digraph() = default;
    digraph(const digraph&) = default;
    digraph(digraph &&) = default;

    digraph& operator = (const digraph&) = default;
    digraph& operator = (digraph &&) = default;

    std::size_t vertex() const
    {
        return adj_.size();
    }

    void add_edge(int from, int to)
    {
        adj_[from].add(to);
    }

    edge_iterator edge_begin(int v) const
    {
        return adj_[v].begin();
    }

    edge_iterator edge_end(int v) const
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

// DFS algorithm for digraph
class digraph_dfs
{
public:
    explicit digraph_dfs(const digraph &g)
        : g_(g), visited_(g.vertex())
    {
    }

    digraph_dfs(const digraph &g, int v)
        : g_(g), visited_(g.vertex())
    {
        dfs(v);
    }

    template<typename Iter>
    digraph_dfs(const digraph &g, Iter begin, Iter end)
        : g_(g), visited_(g.vertex())
    {
        dfs(begin, end);
    }

    std::size_t vertex() const
    {
        return visited_.size();
    }

    // Vertex v is marked or not
    bool marked(int v) const
    {
        return visited_[v];
    }

    // DFS vertex v, add all new marked vertexes to container c
    template<typename Container = std::vector<int>>
    void dfs(int v, Container *c = nullptr)
    {
        if (!visited_[v])
        {
            visited_[v] = true;
            if (c) c->push_back(v);

            auto it = g_.edge_begin(v);
            auto end = g_.edge_end(v);
            for (; it != end; ++it)
                dfs(*it, c);
        }
    }

    // DFS vertexes [begin, end), add all new marked vertexes to container c
    template<typename Iter, typename Container = std::vector<int>>
    void dfs(Iter begin, Iter end, Container *c = nullptr)
    {
        for (Iter it = begin; it != end; ++it)
            dfs(*it, c);
    }

    template<typename Func>
    void dfs(int v, const Func &f)
    {
        if (!visited_[v])
        {
            visited_[v] = true;
            f(v);

            auto it = g_.edge_begin(v);
            auto end = g_.edge_end(v);
            for (; it != end; ++it)
                dfs(*it, f);
        }
    }

    template<typename Iter, typename Func>
    void dfs(Iter begin, Iter end, const Func &f)
    {
        for (Iter it = begin; it != end; ++it)
            dfs(*it, f);
    }

    // Clear all mark
    void clear_mark()
    {
        auto size = vertex();
        for (std::size_t i = 0; i < size; ++i)
            visited_[i] = false;
    }

    template<typename Container>
    void get_all_marked(Container &c)
    {
        int v = vertex();
        for (int i = 0; i < v; ++i)
            if (marked(i))
                c.push_back(i);
    }

private:
    const digraph &g_;
    std::vector<bool> visited_;
};

} // namespace skl

#endif // DIGRAPH_H
