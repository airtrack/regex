#include "Regex.h"

namespace regex
{
    Regex::Regex(const std::string &re)
        : re_(re),
          state_machine_(automata::ConstructStateMachine(re))
    {
    }

    const automata::State * Regex::GetStartState() const
    {
        return state_machine_->start_state_;
    }
} // namespace regex
