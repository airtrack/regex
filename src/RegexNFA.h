#ifndef REGEX_NFA_H
#define REGEX_NFA_H

#include "RegexFAElement.h"
#include <vector>
#include <memory>

namespace regex
{
    namespace automata
    {
        class NFA
        {
        public:
            explicit NFA(const std::shared_ptr<EdgeSet> &edge_set)
                : edge_set_(edge_set), start_(nullptr), accept_(nullptr)
            {
            }

            NFA(const NFA &) = delete;
            void operator = (const NFA &) = delete;

            typedef std::vector<std::unique_ptr<Node>> NodeList;
            typedef EdgeSet::Iterator EdgeIterator;

            void SetMap(const Node *node1, const Edge *edge, const Node *node2)
            {
                node_map_.Set(node1, edge, node2);
            }

            const Node * Map(const Node *node, const Edge *edge)
            {
                return node_map_.Map(node, edge);
            }

            auto MapEqualRange(const Node *node, const Edge *edge)
                -> decltype(NodeMap<Node, Edge>().EqualRange(nullptr, nullptr)) const
            {
                return node_map_.EqualRange(node, edge);
            }

            const Node * AddNode(Node::Type t = Node::Type_Normal)
            {
                auto index = nodes_.size();
                auto node = new Node(index, t);
                nodes_.push_back(std::unique_ptr<Node>(node));
                return node;
            }

            const Node * AddRepeatNode(int repeat_min, int repeat_max,
                                       std::unique_ptr<NFA> sub_nfa)
            {
                auto index = nodes_.size();
                auto node = new Node(index, repeat_min, repeat_max, std::move(sub_nfa));
                nodes_.push_back(std::unique_ptr<Node>(node));
                return node;
            }

            void SetStartNode(const Node *node)
            {
                start_ = node;
                const_cast<Node *>(start_)->type_ = Node::Type_Start;
            }

            const Node * GetStartNode() const
            {
                return start_;
            }

            void SetAcceptNode(const Node *node)
            {
                accept_ = node;
                const_cast<Node *>(accept_)->type_ = Node::Type_Accept;
            }

            const Node * GetAcceptNode() const
            {
                return accept_;
            }

            std::size_t NodeCount() const
            {
                return nodes_.size();
            }

            // Get node by index
            Node * GetNode(std::size_t index)
            {
                if (index < nodes_.size())
                    return nodes_[index].get();
                else
                    return nullptr;
            }

            // Get epsilon edge
            const Edge * GetEpsilon() const
            {
                return &epsilon_;
            }

            // Get direct edge
            const Edge * GetDirectEdge() const
            {
                return edge_set_->GetDirectEdge();
            }

            // Get edge iterator range
            std::pair<EdgeIterator, EdgeIterator> SearchEdge(int c) const
            {
                return edge_set_->Search(c);
            }

            std::pair<EdgeIterator, EdgeIterator> SearchEdge(int first, int last) const
            {
                return edge_set_->Search(first, last);
            }

            // Get all edges begin and end iterator pair without epsilon edge
            std::pair<EdgeIterator, EdgeIterator> GetEdgeIterators() const
            {
                return std::make_pair(edge_set_->Begin(), edge_set_->End());
            }

            std::shared_ptr<EdgeSet> GetEdgeSet() const
            {
                return edge_set_;
            }

            void Debug() const;

        private:
            // All NFA nodes
            NodeList nodes_;
            // Epsilon edge
            Edge epsilon_;
            // Non-epsilon edges
            std::shared_ptr<EdgeSet> edge_set_;
            // Node-Edge map
            NodeMap<Node, Edge> node_map_;

            // Start and accept node
            const Node *start_;
            const Node *accept_;
        };
    } // namespace automata
} // namespace regex

#endif // REGEX_NFA_H
