#include "RegexMatcher.h"
#include <assert.h>

namespace regex
{
    RegexMatcher::RegexMatcher(const Regex &regex)
        : regex_(regex),
          current_(nullptr),
          end_(nullptr),
          start_(nullptr),
          last_accept_(nullptr)
    {
    }

    bool RegexMatcher::IsMatch(const char *begin, const char *end) const
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
                              MatchResults &match_results)
    {
        current_ = begin;
        end_ = end;

        while (current_ < end_)
        {
            Match match;
            if (MatchOne(match))
                match_results.AddMatch(match);
        }
    }

    bool RegexMatcher::MatchOne(Match &match)
    {
        auto current_state = regex_.GetStartState();
        assert(current_state);

        start_ = nullptr;
        last_accept_ = nullptr;
        for (; current_ != end_; ++current_)
        {
            auto next = regex_.GetNextStateIndex(*current_);
            if (next.first && current_state->next_[next.second])
            {
                start_ = current_;
                break;
            }
        }

        for (; current_ != end_; ++current_)
        {
            auto next = regex_.GetNextStateIndex(*current_);
            if (!next.first)
                return CheckLastAccept(match);

            current_state = current_state->next_[next.second];
            if (!current_state)
                return CheckLastAccept(match);
            else if (current_state->accept_)
                last_accept_ = current_;
        }

        return CheckLastAccept(match);
    }

    bool RegexMatcher::CheckLastAccept(Match &match)
    {
        if (last_accept_)
        {
            match.begin_ = start_;
            match.end_ = last_accept_ + 1;
            current_ = match.end_;
            return true;
        }
        else if (start_)
        {
            current_ = start_ + 1;
            return false;
        }
        else
        {
            return false;
        }
    }
} // namespace regex
