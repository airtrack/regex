#ifndef REGEX_AUTOMATA_H
#define REGEX_AUTOMATA_H

#include <string>
#include <vector>
#include <memory>

namespace regex
{
    namespace automata
    {
        struct CharRange
        {
            int first_;
            int last_;

            CharRange(int first, int last) : first_(first), last_(last) { }
        };

        struct State
        {
            // Accept state or not
            bool accept_;

            // Next states
            std::vector<const State *> next_;

            State(bool accept, std::size_t next_state_count)
                : accept_(accept),
                  next_(next_state_count)
            {
            }
        };

        struct StateMachine
        {
            // Character set
            std::vector<CharRange> char_set_;
            // All states
            std::vector<std::unique_ptr<State>> states_;
            // Start state of the state machine
            const State *start_state_;

            StateMachine() : start_state_(nullptr) { }

            StateMachine(const StateMachine &) = delete;
            void operator = (const StateMachine &) = delete;

            // Get index of next states in State by character 'c' as
            // second value of return pair when first value of return
            // pair is true.
            std::pair<bool, std::size_t> GetNextStateIndex(int c) const;
        };

        std::unique_ptr<StateMachine> ConstructStateMachine(const std::string &re);
    } // namespace automata
} // namespace regex

#endif // REGEX_AUTOMATA_H
