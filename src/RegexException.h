#ifndef REGEX_EXCEPTION_H
#define REGEX_EXCEPTION_H

#include <string>

namespace regex
{
    class ParseException
    {
    public:
        ParseException(const std::string &desc, std::size_t pos)
            : desc_(desc), pos_(pos)
        {
        }

        std::string What() const
        {
            return desc_;
        }

        std::size_t Position() const
        {
            return pos_;
        }

    private:
        std::string desc_;
        std::size_t pos_;
    };
} // namespace regex

#endif // REGEX_EXCEPTION_H
