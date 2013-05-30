#ifndef REGEX_AUTOMATA_H
#define REGEX_AUTOMATA_H

#include <cstdlib>

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
            // Character range [first_, last_]
            int first_;
            int last_;

            // Is epsilon edge or not
            bool is_epsilon_;

            Edge() : first_(0), last_(0), is_epsilon_(true) { }

            Edge(int first, int last)
                : first_(first), last_(last), is_epsilon_(false)
            {
            }
        };
    } // namespace automata
} // namespace regex

#endif // REGEX_AUTOMATA_H
