#ifndef REGEX_H
#define REGEX_H

#include "RegexAutomata.h"
#include <string>

namespace regex
{
    class Regex
    {
    public:
        explicit Regex(const std::string &re);

        Regex(const Regex &) = delete;
        void operator = (const Regex &) = delete;

        const automata::State * GetStartState() const;

    private:
        std::string re_;
        std::unique_ptr<automata::StateMachine> state_machine_;
    };
} // namespace regex

#endif // REGEX_H
