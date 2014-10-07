#ifndef SIMPLE_SET_H
#define SIMPLE_SET_H

#include <algorithm>
#include <initializer_list>
#include <vector>

namespace skl {

// Simple implementation of small set, after set has been constructed,
// supply elements traverse performance better than std::set.
template<typename T>
class simple_set
{
public:
    typedef T value_type;
    typedef typename std::vector<T>::const_iterator iterator;
    typedef iterator const_iterator;

    simple_set() { }

    template<typename U>
    simple_set(std::initializer_list<U> il)
    {
        elems_.reserve(il.size());
        for (const auto &e : il)
            add(e);
    }

    inline bool empty() const
    {
        return elems_.empty();
    }

    inline void clear()
    {
        elems_.clear();
    }

    inline std::size_t size() const
    {
        return elems_.size();
    }

    inline void add(const T &o)
    {
        if (!exist(o))
            elems_.push_back(o);
    }

    inline void erase(const T &o)
    {
        auto it = std::find(begin(), end(), o);
        if (it != end())
            elems_.erase(it);
    }

    inline bool exist(const T &o)
    {
        return std::find(begin(), end(), o) != end();
    }

    inline iterator begin() const
    {
        return elems_.begin();
    }

    inline iterator end() const
    {
        return elems_.end();
    }

private:
    std::vector<T> elems_;
};

} // namespace skl

#endif // SIMPLE_SET_H
