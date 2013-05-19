#ifndef REGEX_EXCEPTION_H
#define REGEX_EXCEPTION_H

#include <string>

namespace regex
{
    class ParseException
    {
    public:
        explicit ParseException(const std::string &desc)
            : desc_(desc)
        {
        }

        std::string What() const
        {
            return desc_;
        }

    private:
        std::string desc_;
    };
} // namespace regex

#endif // REGEX_EXCEPTION_H
