#ifndef SIMPLE_SET_H
#define SIMPLE_SET_H

#include <algorithm>
#include <initializer_list>
#include <vector>

namespace skl {

// Simple implementation for small set
template<typename T>
class simple_set
{
public:
    typedef T value_type;
    typedef typename std::vector<T>::const_iterator iterator;
    typedef iterator const_iterator;

    template<typename U>
    simple_set(std::initializer_list<U> il)
    {
        elems_.reserve(il.size());
        for (const auto &e : il)
            add(e);
    }

    simple_set() = default;
    simple_set(const simple_set&) = default;
    simple_set(simple_set &&) = default;

    simple_set& operator = (const simple_set&) = default;
    simple_set& operator = (simple_set &&) = default;

    bool empty() const
    {
        return elems_.empty();
    }

    void clear()
    {
        elems_.clear();
    }

    std::size_t size() const
    {
        return elems_.size();
    }

    void add(const T &o)
    {
        if (!exist(o))
            elems_.push_back(o);
    }

    bool exist(const T &o)
    {
        return std::find(begin(), end(), o) != elems_.end();
    }

    iterator begin() const
    {
        return elems_.begin();
    }

    iterator end() const
    {
        return elems_.end();
    }

private:
    std::vector<T> elems_;
};

} // namespace skl

#endif // SIMPLE_SET_H
