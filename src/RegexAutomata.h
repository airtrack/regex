#ifndef REGEX_AUTOMATA_H
#define REGEX_AUTOMATA_H

#include <functional>

namespace regex
{
    namespace automata
    {
        struct Node
        {
            enum Type
            {
                Type_Start,
                Type_Accept,
                Type_Normal,
            } type_;
        };

        struct Edge
        {
            // Edge check function, return true
            // when character belongs to the edge.
            std::function<bool (int)> e_;

            // Is edge epsilon or not
            bool is_epsilon_;
        };
    } // namespace automata
} // namespace regex

#endif // REGEX_AUTOMATA_H
