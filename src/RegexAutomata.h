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
            };

            // Type of this node
            Type type_;

            // Index and number of this node
            std::size_t index_;

            Node(std::size_t index, Type t)
                : index_(index), type_(t) { }
        };

        struct Edge
        {
            // Edge check function, return true
            // when character belongs to the edge.
            std::function<bool (int)> e_;

            // Is edge epsilon or not
            bool is_epsilon_;

            Edge() : is_epsilon_(true) { }

            explicit Edge(const std::function<bool (int)> &e)
                : e_(e), is_epsilon_(false) { }
        };
    } // namespace automata
} // namespace regex

#endif // REGEX_AUTOMATA_H
