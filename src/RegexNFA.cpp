#include "RegexNFA.h"
#include <iostream>

namespace regex
{
    namespace automata
    {
        void NFA::Debug() const
        {
            std::cout << "NFA:" << std::endl;
            std::cout << "nodes:" << std::endl;
            for (auto &n : nodes_)
            {
                std::cout << "  " << n.get() << ": (" << n->TypeDesc() << ","
                << n->index_ << ")" << std::endl;
            }

            std::cout << "edges:" << std::endl;
            std::cout << "  " << &epsilon_ << ": epsilon" << std::endl;
            std::cout << "  " << edge_set_->GetDirectEdge() << ": direct" << std::endl;

            for (auto it = edge_set_->Begin(); it != edge_set_->End(); ++it)
            {
                std::cout << "  " << &(*it) << ": [" << it->first_ << "-"
                << it->last_ << "]" << std::endl;
            }

            std::cout << "node-edge map:" << std::endl;
            for (auto it = node_map_.Begin(); it != node_map_.End(); ++it)
            {
                std::cout << "  " << it->first.first << " -> " << it->first.second
                << " -> " << it->second << std::endl;
            }
        }
    } // namespace automata
} // namespace regex
