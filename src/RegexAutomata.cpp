#include "RegexAutomata.h"
#include "RegexMatcher.h"
#include "RegexConstructor.h"
#include <iostream>
#include <assert.h>

namespace regex
{
    namespace automata
    {
        const State * State::GetNextState(const StateMachine *state_machine,
                                          const char *&current, const char *end) const
        {
            if (current == end)
                return nullptr;

            auto next = state_machine->GetNextStateIndex(*current++);
            if (!next.first)
                return nullptr;
            else
                return next_[next.second];
        }

        std::pair<bool, const State *> State::CompleteState(const char *&current,
                                                            const char *begin,
                                                            const char *end) const
        {
            return std::make_pair(true, nullptr);
        }

        std::pair<bool, std::size_t> StateMachine::GetNextStateIndex(int c) const
        {
            std::size_t left = 0;
            std::size_t right = char_set_.size();
            while (left < right)
            {
                std::size_t middle = (left + right) / 2;
                const CharRange &range = char_set_[middle];

                if (c >= range.first_ && c <= range.last_)
                    return std::make_pair(true, middle);
                else if (c < range.first_)
                    right = middle;
                else
                    left = middle + 1;
            }

            return std::make_pair(false, 0);
        }

        std::pair<bool, const State *> RepeatState::CompleteState(const char *&current,
                                                                  const char *begin,
                                                                  const char *end) const
        {
            RegexMatcher matcher(sub_state_machine_.get());

            int match_times = 0;
            const char *last = current;
            while (match_times < repeat_max_ &&
                   matcher.MatchPart(current, end, &current))
            {
                ++match_times;
                // If 'matcher' match success but do not comsume
                // any character, then break when 'match_times'
                // >= 'repeat_min_'
                if (last == current && match_times >= repeat_min_)
                    break;
                last = current;
            }

            return std::make_pair(match_times >= repeat_min_, direct_next_);
        }

        std::pair<bool, const State *> LineHeadState::CompleteState(const char *&current,
                                                                    const char *begin,
                                                                    const char *end) const
        {
            bool head = current == begin || *(current - 1) == '\n';
            return std::make_pair(head, direct_next_);
        }

        std::pair<bool, const State *> LineTailState::CompleteState(const char *&current,
                                                                    const char *begin,
                                                                    const char *end) const
        {
            bool tail = current == end || *current == '\n';
            return std::make_pair(tail, direct_next_);
        }

        std::unique_ptr<StateMachine> ConstructStateMachineFromNFA(std::unique_ptr<NFA> nfa,
                                                                   const char *debug)
        {
            std::cout << debug << std::endl;
            nfa->Debug();
            std::cout << std::endl;

            DFAConstructor dfa_constructor(std::move(nfa));
            dfa_constructor.Debug();
            std::cout << std::endl;

            auto dfa = dfa_constructor.ConstructDFA();
            dfa->Debug();

            return dfa->ConstructStateMachine();
        }

        std::unique_ptr<StateMachine> ConstructStateMachine(const std::string &re)
        {
            auto ast = parser::Parse(re);
            auto nfa = ConvertASTToNFA(std::move(ast));
            return ConstructStateMachineFromNFA(std::move(nfa), "Main regex:");
        }
    } // namespace automata
} // namespace regex
