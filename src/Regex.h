#ifndef REGEX_H
#define REGEX_H

#include <string>

namespace regex
{
    class Regex
    {
    public:
        explicit Regex(const std::string &re);

        Regex(const Regex &) = delete;
        void operator = (const Regex &) = delete;

    private:
        std::string re_;
    };
} // namespace regex

#endif // REGEX_H
