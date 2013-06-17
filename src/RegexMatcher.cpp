#include "RegexMatcher.h"

namespace regex
{
    RegexMatcher::RegexMatcher(const Regex &regex)
        : regex_(regex)
    {
    }

    bool RegexMatcher::Match(const char *begin, const char *end) const
    {
        return false;
    }
} // namespace regex
