#include "Regex.h"

namespace regex
{
    Regex::Regex(const std::string &re)
        : re_(re),
          state_machine_(automata::ConstructStateMachine(re))
    {
    }
} // namespace regex
