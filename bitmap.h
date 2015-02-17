#ifndef BITMAP_H
#define BITMAP_H

#include <assert.h>
#include <cstddef>

namespace skl {

class bitmap
{
public:
    explicit bitmap(std::size_t bits_num)
        : size_(bits_num)
    {
        assert(size_ > 0);
        uint_num_ = (size_ - 1) / bits_per_uint + 1;
        bits_ = new uint_type[uint_num_];
        clear();
    }

    bitmap(const bitmap&) = delete;
    void operator = (const bitmap&) = delete;

    ~bitmap()
    {
        delete [] bits_;
    }

    // Bits number
    inline std::size_t size() const
    {
        return size_;
    }

    // Bit v is set or not
    inline bool is_set(std::size_t v) const
    {
        auto index = v / bits_per_uint;
        auto bit = v % bits_per_uint;
        return (bits_[index] & (0x1 << bit)) != 0;
    }

    // Set bit v
    inline void set(std::size_t v)
    {
        auto index = v / bits_per_uint;
        auto bit = v % bits_per_uint;
        bits_[index] |= (0x1 << bit);
    }

    // Clear all bits
    inline void clear()
    {
        for (std::size_t i = 0; i < uint_num_; ++i)
            bits_[i] = 0;
    }

private:
    typedef unsigned int uint_type;
    static const std::size_t bits_per_uint = sizeof(uint_type) * 8;

    std::size_t size_;
    std::size_t uint_num_;
    uint_type *bits_;
};

} // namespace skl

#endif // BITMAP_H
