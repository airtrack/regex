#include "RegexDFA.h"
#include "RegexNFA.h"
#include <iostream>
#include <queue>
#include <assert.h>

namespace regex
{
    namespace automata
    {
        DFA::DFA(std::vector<std::unique_ptr<Node>> &&nodes,
                 NodeMap<Node, Edge> &&node_map,
                 const std::shared_ptr<EdgeSet> &edge_set)
            : edge_set_(edge_set),
              nodes_(std::move(nodes)),
              node_map_(std::move(node_map))
        {
            Minimization();
        }

        std::unique_ptr<StateMachine> DFA::ConstructStateMachine()
        {
            std::unique_ptr<StateMachine> state_machine(new StateMachine);
            std::unordered_map<const Edge *, std::size_t> edge_index;
            std::unordered_map<const NodeList *, State *> node_list_state;

            // Transform edges to char set
            TransformEdgesToCharSet(state_machine, edge_index);

            // Transform node_lists to states
            TransformNodeListsToStates(state_machine, node_list_state);

            // Fill all state transitions
            FillTransitions(state_machine, edge_index, node_list_state);

            return std::move(state_machine);
        }

        void DFA::Debug() const
        {
            std::cout << "DFA:" << std::endl;

            std::cout << "not minimization:" << std::endl;
            std::cout << "nodes:" << std::endl;
            for (auto &n : nodes_)
            {
                std::cout << "  " << n.get() << ": (" << n->TypeDesc()
                << "," << n->index_ << ")" << std::endl;
            }

            std::cout << "node-edge map:" << std::endl;
            for (auto it = node_map_.Begin(); it != node_map_.End(); ++it)
            {
                std::cout << "  " << it->first.first << " -> " << it->first.second
                << " -> " << it->second << std::endl;
            }

            std::cout << "\nminimization:" << std::endl;
            std::cout << "nodes:" << std::endl;
            for (auto &nl : node_list_set_)
            {
                std::cout << "  " << nl.get() << ":";
                for (auto &n : *nl)
                {
                    std::cout << " " << n->TypeDesc();
                }
                std::cout << std::endl;
            }

            std::cout << "node-edge map:" << std::endl;
            for (auto it = node_list_map_.Begin();
                 it != node_list_map_.End(); ++it)
            {
                std::cout << "  " << it->first.first << " -> " << it->first.second
                << " -> " << it->second << std::endl;
            }
        }

        void DFA::Minimization()
        {
            std::unique_ptr<NodeList> accept_node_list(new NodeList);
            std::unique_ptr<NodeList> not_accept_node_list(new NodeList);
            NodeOwnerMap node_owner_map;

            for (auto &n : nodes_)
            {
                if (n->type_ & Node::Type_Accept)
                {
                    accept_node_list->push_back(n.get());
                    node_owner_map[n.get()] = accept_node_list.get();
                }
                else
                {
                    not_accept_node_list->push_back(n.get());
                    node_owner_map[n.get()] = not_accept_node_list.get();
                }
            }

            node_list_set_.push_back(std::move(accept_node_list));
            node_list_set_.push_back(std::move(not_accept_node_list));
            DoMinimization(node_owner_map);
            BuildNodeListEdgeMap(node_owner_map);
        }

        void DFA::BuildNodeListEdgeMap(NodeOwnerMap &node_owner_map)
        {
            for (auto node_list_it = node_list_set_.begin();
                 node_list_it != node_list_set_.end(); ++node_list_it)
            {
                auto node_list = node_list_it->get();
                if (node_list->empty())
                    continue;

                auto node = *node_list->begin();
                for (auto edge_it = edge_set_->Begin();
                     edge_it != edge_set_->End(); ++edge_it)
                {
                    auto edge = &(*edge_it);
                    BuildNodeListEdgeMap(edge, node, node_list, node_owner_map);
                }

                BuildNodeListEdgeMap(edge_set_->GetDirectEdge(),
                                     node, node_list, node_owner_map);
            }
        }

        void DFA::BuildNodeListEdgeMap(const Edge *edge,
                                       const Node *node,
                                       const NodeList *node_list,
                                       NodeOwnerMap &node_owner_map)
        {
            auto mapped_node = node_map_.Map(node, edge);
            if (mapped_node)
            {
                auto mapped_node_list = node_owner_map[mapped_node];
                node_list_map_.Set(node_list, edge, mapped_node_list);
            }
        }

        void DFA::DoMinimization(NodeOwnerMap &node_owner_map)
        {
            std::size_t old_size = 0;

            do
            {
                old_size = node_list_set_.size();
                SplitAll(node_owner_map);
            } while (old_size != node_list_set_.size());
        }

        void DFA::SplitAll(NodeOwnerMap &node_owner_map)
        {
            std::queue<NodeList *> work_list;

            for (auto &node_list : node_list_set_)
                work_list.push(node_list.get());

            while (!work_list.empty())
            {
                auto node_list = work_list.front();
                work_list.pop();
                Split(*node_list, node_owner_map);
            }
        }

        void DFA::Split(NodeList &node_list,
                        NodeOwnerMap &node_owner_map)
        {
            // If node count less equal than 1, it can not be split
            if (node_list.size() <= 1)
                return ;

            for (auto edge_it = edge_set_->Begin();
                 edge_it != edge_set_->End(); ++edge_it)
            {
                auto edge = &(*edge_it);
                if (SplitByEdge(edge, node_list, node_owner_map))
                    return ;
            }

            SplitByEdge(edge_set_->GetDirectEdge(), node_list, node_owner_map);
        }

        bool DFA::SplitByEdge(const Edge *edge, NodeList &node_list,
                              NodeOwnerMap &node_owner_map)
        {
            auto node_it = node_list.begin();
            auto mapped_node = node_map_.Map(*node_it++, edge);
            const NodeList *mapped_node_list = nullptr;

            if (mapped_node)
                mapped_node_list = node_owner_map[mapped_node];

            NodeList *new_list = nullptr;
            while (node_it != node_list.end())
            {
                auto new_node = *node_it;
                auto new_mapped_node = node_map_.Map(new_node, edge);
                const NodeList *new_mapped_node_list = nullptr;
                if (new_mapped_node)
                    new_mapped_node_list = node_owner_map[new_mapped_node];

                if (new_mapped_node_list != mapped_node_list)
                {
                    if (!new_list)
                    {
                        new_list = new NodeList;
                        node_list_set_.push_back(std::unique_ptr<NodeList>(new_list));
                    }

                    node_owner_map[new_node] = new_list;
                    new_list->splice(new_list->end(), node_list, node_it++);
                }
                else
                {
                    ++node_it;
                }
            }

            // If split success, we return true
            return new_list != nullptr;
        }

        void DFA::TransformEdgesToCharSet(std::unique_ptr<StateMachine> &state_machine,
                                          std::unordered_map<const Edge *, std::size_t> &edge_index)
        {
            for (auto it = edge_set_->Begin(); it != edge_set_->End(); ++it)
            {
                auto index = state_machine->char_set_.size();
                edge_index[&(*it)] = index;
                state_machine->char_set_.push_back(CharRange(it->first_, it->last_));
            }
        }

        void DFA::TransformNodeListsToStates(std::unique_ptr<StateMachine> &state_machine,
                                             std::unordered_map<const NodeList *, State *> &node_list_state)
        {
            for (auto &node_list : node_list_set_)
            {
                if (node_list->empty())
                    continue;

                bool accept = false;
                bool start = false;
                State *state = nullptr;
                for (auto node : *node_list)
                {
                    if (node->type_ & Node::Type_Accept)
                        accept = true;
                    if (node->type_ & Node::Type_Start)
                        start = true;
                    if (node->type_ & Node::Type_Repeat)
                    {
                        assert(!state);
                        auto sub_state_machine = ConstructStateMachineFromNFA(std::move(node->sub_nfa_),
                                                                              "Sub regex:");
                        state = new RepeatState(state_machine->char_set_.size(),
                                                node->repeat_min_, node->repeat_max_,
                                                std::move(sub_state_machine));
                    }
                    if (node->type_ & Node::Type_LineHead)
                    {
                        assert(!state);
                        state = new LineHeadState(state_machine->char_set_.size());
                    }
                    if (node->type_ & Node::Type_LineTail)
                    {
                        assert(!state);
                        state = new LineTailState(state_machine->char_set_.size());
                    }
                }

                if (!state)
                    state = new State(state_machine->char_set_.size());

                state->accept_ = accept;
                state_machine->states_.push_back(std::unique_ptr<State>(state));
                node_list_state[node_list.get()] = state;

                if (start)
                {
                    assert(!state_machine->start_state_);
                    state_machine->start_state_ = state;
                }
            }
            assert(state_machine->start_state_);
        }

        void DFA::FillTransitions(std::unique_ptr<StateMachine> &state_machine,
                                  std::unordered_map<const Edge *, std::size_t> &edge_index,
                                  std::unordered_map<const NodeList *, State *> &node_list_state)
        {
            for (auto it = node_list_map_.Begin(); it != node_list_map_.End(); ++it)
            {
                auto from_state = node_list_state[it->first.first];
                auto to_state = node_list_state[it->second];

                if (it->first.second == edge_set_->GetDirectEdge())
                {
                    auto *from = static_cast<DirectNextState *>(from_state);
                    from->direct_next_ = to_state;
                }
                else
                {
                    auto index = edge_index[it->first.second];

                    assert(from_state);
                    assert(to_state);
                    assert(index < from_state->next_.size());
                    assert(index < state_machine->char_set_.size());

                    from_state->next_[index] = to_state;
                }
            }
        }
    } // namespace automata
} // namespace regex
