#include "RegexMatcher.h"
#include <assert.h>

namespace regex
{
    RegexMatcher::RegexMatcher(const Regex &regex)
        : state_machine_(regex.GetStateMachine()),
          current_(nullptr),
          end_(nullptr),
          start_(nullptr),
          last_accept_(nullptr)
    {
    }

    RegexMatcher::RegexMatcher(const automata::StateMachine *state_machine)
        : state_machine_(state_machine),
          current_(nullptr),
          end_(nullptr),
          start_(nullptr),
          last_accept_(nullptr)
    {
    }

    bool RegexMatcher::IsMatch(const char *begin, const char *end)
    {
        auto match = MatchPart(begin, end, &begin);
        return match && begin == end;
    }

    bool RegexMatcher::MatchPart(const char *begin, const char *end,
                                 const char **new_begin)
    {
        auto current_state = state_machine_->start_state_;
        assert(current_state);

        current_ = begin;
        end_ = end;
        last_accept_ = nullptr;

        while (current_ != end)
        {
            auto complete = current_state->CompleteState(current_, end_);
            if (complete.first)
            {
                if (current_state->accept_)
                    last_accept_ = current_ - 1;

                if (complete.second)
                    current_state = complete.second;
                else
                    current_state = current_state->GetNextState(state_machine_, current_, end_);
            }

            if (!complete.first || !current_state)
            {
                if (new_begin && last_accept_)
                    *new_begin = last_accept_ + 1;
                return last_accept_ != nullptr;
            }
        }

        auto complete = current_state->CompleteState(current_, end_);
        if (complete.first && current_state->accept_)
            last_accept_ = current_ - 1;

        if (new_begin && last_accept_)
            *new_begin = last_accept_ + 1;
        return last_accept_ != nullptr;
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
        auto current_state = state_machine_->start_state_;
        assert(current_state);

        start_ = current_;
        last_accept_ = nullptr;

        while (current_ != end_)
        {
            auto complete = current_state->CompleteState(current_, end_);
            if (complete.first)
            {
                if (current_state->accept_)
                    last_accept_ = current_ - 1;

                if (complete.second)
                    current_state = complete.second;
                else
                    current_state = current_state->GetNextState(state_machine_, current_, end_);
            }

            if (!complete.first || !current_state)
                return CheckLastAccept(match);
        }

        auto complete = current_state->CompleteState(current_, end_);
        if (complete.first && current_state->accept_)
            last_accept_ = current_ - 1;

        return CheckLastAccept(match);
    }

    bool RegexMatcher::CheckLastAccept(Match &match)
    {
        if (last_accept_)
        {
            match.begin_ = start_;
            match.end_ = last_accept_ + 1;
            // If 'match' is empty, then set 'current_'
            // pointer to next character.
            if (match.begin_ == match.end_)
                current_ = match.end_ + 1;
            else
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
