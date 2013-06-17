#ifndef REGEX_MATCHER_H
#define REGEX_MATCHER_H

#include "Regex.h"

namespace regex
{
    class RegexMatcher
    {
    public:
        explicit RegexMatcher(const Regex &regex);

        RegexMatcher(const RegexMatcher &) = delete;
        void operator = (const RegexMatcher &) = delete;

        // Check [begin, end) characters is match regex_ or not
        bool Match(const char *begin, const char *end) const;

    private:
        const Regex &regex_;
    };
} // namespace regex

#endif // REGEX_MATCHER_H
