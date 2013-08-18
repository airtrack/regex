#ifndef REGEX_DFA_H
#define REGEX_DFA_H

#include "RegexFAElement.h"
#include "RegexAutomata.h"
#include <memory>
#include <vector>
#include <list>
#include <unordered_map>

namespace regex
{
    namespace automata
    {
        class DFA
        {
        public:
            DFA(std::vector<std::unique_ptr<Node>> &&nodes,
                NodeMap<Node, Edge> &&node_map,
                const std::shared_ptr<EdgeSet> &edge_set);

            DFA(const DFA &) = delete;
            void operator = (const DFA &) = delete;

            std::unique_ptr<StateMachine> ConstructStateMachine();

            void Debug() const;

        private:
            typedef std::list<Node *> NodeList;
            typedef std::vector<std::unique_ptr<NodeList>> NodeListSet;
            typedef NodeMap<NodeList, Edge> NodeListEdgeMap;
            typedef std::unordered_map<const Node *, NodeList *> NodeOwnerMap;

            // For DFA minimization
            void Minimization();
            void BuildNodeListEdgeMap(NodeOwnerMap &node_owner_map);
            void BuildNodeListEdgeMap(const Edge *edge,
                                      const Node *node,
                                      const NodeList *node_list,
                                      NodeOwnerMap &node_owner_map);
            void DoMinimization(NodeOwnerMap &node_owner_map);
            void SplitAll(NodeOwnerMap &node_owner_map);
            void Split(NodeList &node_list,
                       NodeOwnerMap &node_owner_map);
            bool SplitByEdge(const Edge *edge,
                             NodeList &node_list,
                             NodeOwnerMap &node_owner_map);

            // For construct StateMachine
            void TransformEdgesToCharSet(std::unique_ptr<StateMachine> &state_machine,
                                         std::unordered_map<const Edge *, std::size_t> &edge_index);
            void TransformNodeListsToStates(std::unique_ptr<StateMachine> &state_machine,
                                            std::unordered_map<const NodeList *, State *> &node_list_state);
            void FillTransitions(std::unique_ptr<StateMachine> &state_machine,
                                 std::unordered_map<const Edge *, std::size_t> &edge_index,
                                 std::unordered_map<const NodeList *, State *> &node_list_state);

            // All edges
            std::shared_ptr<EdgeSet> edge_set_;

            // Not minimization DFA's nodes and node-edge map
            std::vector<std::unique_ptr<Node>> nodes_;
            NodeMap<Node, Edge> node_map_;

            // Minimization DFA's node_lists and node_list-edge map
            NodeListSet node_list_set_;
            NodeListEdgeMap node_list_map_;
        };
    } // namespace automata
} // namespace regex

#endif // REGEX_DFA_H
