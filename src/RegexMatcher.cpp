#include "RegexMatcher.h"
#include <assert.h>

namespace regex
{
    RegexMatcher::RegexMatcher(const Regex &regex)
        : regex_(regex)
    {
    }

    bool RegexMatcher::Match(const char *begin, const char *end) const
    {
        auto current_state = regex_.GetStartState();
        assert(current_state);

        for (auto current = begin; current != end; ++current)
        {
            // If *current is not belongs to the char set of
            // regex_, then match fail.
            auto next = regex_.GetNextStateIndex(*current);
            if (!next.first)
                return false;

            // If next state is nullptr, then match fail.
            current_state = current_state->next_[next.second];
            if (!current_state)
                return false;
        }

        // If current_state is accept state, then match success.
        return current_state->accept_;
    }

    void RegexMatcher::Search(const char *begin, const char *end,
                              MatchResults &match_results) const
    {
    }
} // namespace regex
