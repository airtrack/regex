#include "RegexConstructor.h"
#include <iostream>
#include <functional>
#include <assert.h>

namespace regex
{
    namespace automata
    {
        using namespace parser;

        // Collect all edges from CharRangeNode 'ast' and store in
        // EdgeSet 'edge_set'.
        void CollectEdge(const CharRangeNode *ast, EdgeSet *edge_set)
        {
            for (auto &range : ast->ranges_)
                edge_set->Insert(range.first_, range.last_);

            for (auto c : ast->chars_)
                edge_set->Insert(c);
        }

        // Call 'func' with all ranges which exclude all edges in EdgeSet.
        void DoWithExcludeEdge(const EdgeSet &exclude,
                               const std::function<void (int, int)> &func)
        {
            int current = Edge::Min;
            for (auto it = exclude.Begin(); it != exclude.End(); ++it)
            {
                if (current < it->first_)
                    func(current, it->first_ - 1);
                current = it->last_ + 1;
            }

            if (current <= Edge::Max)
                func(current, Edge::Max);
        }

        // Visitor class construct edge set from AST
        class EdgeSetConstructorVisitor : public Visitor
        {
        public:
            explicit EdgeSetConstructorVisitor(EdgeSet *edge_set)
            : edge_set_(edge_set)
            {
            }

            EdgeSetConstructorVisitor(const EdgeSetConstructorVisitor &) = delete;
            void operator = (const EdgeSetConstructorVisitor &) = delete;

            VISIT_NODE(CharNode);
            VISIT_NODE(CharRangeNode);
            VISIT_NODE(ConcatenationNode);
            VISIT_NODE(AlternationNode);
            VISIT_NODE(ClosureNode);
            VISIT_NODE(RepeatNode);
            VISIT_NODE(DotNode);
            VISIT_NODE(QuestionMarkNode);

        private:
            EdgeSet *edge_set_;
        };

        void EdgeSetConstructorVisitor::Visit(CharNode *ast, void *data)
        {
            edge_set_->Insert(ast->c_);
        }

        void EdgeSetConstructorVisitor::Visit(CharRangeNode *ast, void *data)
        {
            if (ast->exclude_)
            {
                EdgeSet exclude;
                CollectEdge(ast, &exclude);
                void (EdgeSet::*insert)(int, int) = &EdgeSet::Insert;
                DoWithExcludeEdge(exclude,
                                  std::bind(insert, edge_set_,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
            }
            else
            {
                CollectEdge(ast, edge_set_);
            }
        }

        void EdgeSetConstructorVisitor::Visit(ConcatenationNode *ast, void *data)
        {
            for (auto &node : ast->nodes_)
                node->Accept(this, data);
        }

        void EdgeSetConstructorVisitor::Visit(AlternationNode *ast, void *data)
        {
            for (auto &node : ast->nodes_)
                node->Accept(this, data);
        }

        void EdgeSetConstructorVisitor::Visit(ClosureNode *ast, void *data)
        {
            ast->node_->Accept(this, data);
        }

        void EdgeSetConstructorVisitor::Visit(RepeatNode *ast, void *data)
        {
            // ast->node_ is sub-regex, we don't visit sub-regex to generate
            // edges in current regex, we will generate edges when sub-regex
            // construct(sub-NFA construct).
        }

        void EdgeSetConstructorVisitor::Visit(DotNode *ast, void *data)
        {
            // Dot stand for any character.
            edge_set_->Insert(Edge::Min, Edge::Max);
        }

        void EdgeSetConstructorVisitor::Visit(QuestionMarkNode *ast, void *data)
        {
            ast->node_->Accept(this, data);
        }

        // Visitor for convert AST to NFA
        class NFAConverterVisitor : public Visitor
        {
        public:
            struct DataType
            {
                const Node *first;
                const Node *second;
                bool need_direct_edge;

                DataType() : first(nullptr), second(nullptr), need_direct_edge(false)
                {
                }
            };

            explicit NFAConverterVisitor(NFA *nfa) : nfa_(nfa) { }

            NFAConverterVisitor(const NFAConverterVisitor &) = delete;
            void operator = (const NFAConverterVisitor &) = delete;

            VISIT_NODE(CharNode);
            VISIT_NODE(CharRangeNode);
            VISIT_NODE(ConcatenationNode);
            VISIT_NODE(AlternationNode);
            VISIT_NODE(ClosureNode);
            VISIT_NODE(RepeatNode);
            VISIT_NODE(DotNode);
            VISIT_NODE(QuestionMarkNode);

        private:
            void FillData(void *data, const Node *node1, const Node *node2)
            {
                auto pair = reinterpret_cast<DataType *>(data);
                pair->first = node1;
                pair->second = node2;
            }

            NFA *nfa_;
        };

        void NFAConverterVisitor::Visit(CharNode *ast, void *data)
        {
            auto node1 = nfa_->AddNode();
            auto node2 = nfa_->AddNode();

            auto edges = nfa_->SearchEdge(ast->c_);
            for (; edges.first != edges.second; ++edges.first)
            {
                nfa_->SetMap(node1, &(*edges.first), node2);
            }

            FillData(data, node1, node2);
        }

        void NFAConverterVisitor::Visit(CharRangeNode *ast, void *data)
        {
            auto node1 = nfa_->AddNode();
            auto node2 = nfa_->AddNode();

            if (ast->exclude_)
            {
                // Set node-edge map with excluded edge set
                EdgeSet exclude;
                CollectEdge(ast, &exclude);
                DoWithExcludeEdge(exclude, [node1, node2, this](int first, int last) {
                    auto edges = nfa_->SearchEdge(first, last);
                    for (; edges.first != edges.second; ++edges.first)
                        nfa_->SetMap(node1, &(*edges.first), node2);
                });
            }
            else
            {
                // Set node-edge map with edges
                for (auto &range : ast->ranges_)
                {
                    auto edges = nfa_->SearchEdge(range.first_, range.last_);
                    for (; edges.first != edges.second; ++edges.first)
                        nfa_->SetMap(node1, &(*edges.first), node2);
                }
                for (auto c : ast->chars_)
                {
                    auto edges = nfa_->SearchEdge(c);
                    for (; edges.first != edges.second; ++edges.first)
                        nfa_->SetMap(node1, &(*edges.first), node2);
                }
            }

            FillData(data, node1, node2);
        }

        void NFAConverterVisitor::Visit(ConcatenationNode *ast, void *data)
        {
            assert(!ast->nodes_.empty());

            const Node *first = nullptr;
            const Node *last = nullptr;

            DataType pair;
            ast->nodes_[0]->Accept(this, &pair);
            first = pair.first;
            last = pair.second;
            bool need_direct_edge = pair.need_direct_edge;

            for (std::size_t i = 1; i < ast->nodes_.size(); ++i)
            {
                DataType pair;
                ast->nodes_[i]->Accept(this, &pair);

                if (need_direct_edge && pair.need_direct_edge)
                    nfa_->SetMap(last, nfa_->GetDirectEdge(), pair.first);
                else
                    nfa_->SetMap(last, nfa_->GetEpsilon(), pair.first);

                last = pair.second;
                need_direct_edge = pair.need_direct_edge;
            }

            FillData(data, first, last);
        }

        void NFAConverterVisitor::Visit(AlternationNode *ast, void *data)
        {
            std::vector<const Node *> all_first;
            std::vector<const Node *> all_last;

            for (auto &node : ast->nodes_)
            {
                DataType pair;
                node->Accept(this, &pair);
                all_first.push_back(pair.first);
                all_last.push_back(pair.second);
            }

            auto node1 = nfa_->AddNode();
            for (auto first : all_first)
                nfa_->SetMap(node1, nfa_->GetEpsilon(), first);

            auto node2 = nfa_->AddNode();
            for (auto last : all_last)
                nfa_->SetMap(last, nfa_->GetEpsilon(), node2);

            FillData(data, node1, node2);
        }

        void NFAConverterVisitor::Visit(ClosureNode *ast, void *data)
        {
            auto node1 = nfa_->AddNode();
            auto node2 = nfa_->AddNode();

            DataType pair;
            ast->node_->Accept(this, &pair);

            nfa_->SetMap(pair.second, nfa_->GetEpsilon(), pair.first);
            nfa_->SetMap(node1, nfa_->GetEpsilon(), pair.first);
            nfa_->SetMap(pair.second, nfa_->GetEpsilon(), node2);
            nfa_->SetMap(node1, nfa_->GetEpsilon(), node2);

            FillData(data, node1, node2);
        }

        void NFAConverterVisitor::Visit(RepeatNode *ast, void *data)
        {
            // Construct sub-NFA of sub-regex
            auto sub_nfa = ConvertASTToNFA(ast->node_);
            auto node = nfa_->AddRepeatNode(ast->min_times_, ast->max_times_,
                                            std::move(sub_nfa));

            reinterpret_cast<DataType *>(data)->need_direct_edge = true;
            FillData(data, node, node);
        }

        void NFAConverterVisitor::Visit(DotNode *ast, void *data)
        {
            auto node1 = nfa_->AddNode();
            auto node2 = nfa_->AddNode();

            auto edges = nfa_->SearchEdge(Edge::Min, Edge::Max);
            for (; edges.first != edges.second; ++edges.first)
            {
                nfa_->SetMap(node1, &(*edges.first), node2);
            }

            FillData(data, node1, node2);
        }

        void NFAConverterVisitor::Visit(QuestionMarkNode *ast, void *data)
        {
            auto node1 = nfa_->AddNode();
            auto node2 = nfa_->AddNode();

            DataType pair;
            ast->node_->Accept(this, &pair);

            nfa_->SetMap(node1, nfa_->GetEpsilon(), node2);
            nfa_->SetMap(node1, nfa_->GetEpsilon(), pair.first);
            nfa_->SetMap(pair.second, nfa_->GetEpsilon(), node2);

            FillData(data, node1, node2);
        }

        std::unique_ptr<NFA> ConvertASTToNFA(const std::unique_ptr<ASTNode> &ast)
        {
            // Construct EdgeSet
            std::shared_ptr<EdgeSet> edge_set(new EdgeSet);

            EdgeSetConstructorVisitor edge_set_visitor(edge_set.get());
            ast->Accept(&edge_set_visitor, nullptr);

            // Construct NFA
            std::unique_ptr<NFA> nfa(new NFA(edge_set));

            NFAConverterVisitor visitor(nfa.get());
            NFAConverterVisitor::DataType pair;
            ast->Accept(&visitor, &pair);

            nfa->SetStartNode(pair.first);
            nfa->SetAcceptNode(pair.second);

            return std::move(nfa);
        }

        BitSet::BitSet(std::size_t elem_count)
            : elem_count_(elem_count),
              bits_(GetIntCount(elem_count))
        {
        }

        bool BitSet::MergeFrom(const BitSet &other)
        {
            assert(elem_count_ == other.elem_count_);

            bool changed = false;
            std::size_t count = bits_.size();
            for (std::size_t i = 0; i < count; ++i)
            {
                auto &self = bits_[i];
                auto old = self;
                self |= other.bits_[i];
                changed = self != old;
            }

            return changed;
        }

        void BitSet::Set(std::size_t index)
        {
            std::size_t unsigned_int_index = index / kUnsignedIntBits;
            std::size_t bit_index = index % kUnsignedIntBits;

            if (unsigned_int_index < bits_.size())
                bits_[unsigned_int_index] |= 0x1 << bit_index;
        }

        bool BitSet::IsSet(std::size_t index) const
        {
            std::size_t unsigned_int_index = index / kUnsignedIntBits;
            std::size_t bit_index = index % kUnsignedIntBits;

            if (unsigned_int_index < bits_.size())
                return (bits_[unsigned_int_index] & (0x1 << bit_index)) != 0;
            else
                return false;
        }

        std::size_t BitSet::GetIntCount(std::size_t elem_count)
        {
            return elem_count / kUnsignedIntBits +
            (elem_count % kUnsignedIntBits == 0 ? 0 : 1);
        }

        DFAConstructor::DFAConstructor(std::unique_ptr<NFA> nfa)
            : nfa_(std::move(nfa))
        {
            ConstructEpsilonClosure();
            SubsetConstruction();
        }

        std::unique_ptr<DFA> DFAConstructor::ConstructDFA() const
        {
            NodeMap<Node, Edge> dfa_node_map;
            std::vector<std::unique_ptr<Node>> dfa_nodes;
            std::unordered_map<const BitSet *, const Node *> bitset_node_map;

            // Construct all DFA nodes
            for (auto it = subsets_.Begin(); it != subsets_.End(); ++it)
            {
                auto &subset = *it;

                std::unique_ptr<Node> n(new Node);
                size_t elem_count = subset->ElemCount();
                for (size_t i = 0; i < elem_count; ++i)
                {
                    if (subset->IsSet(i))
                    {
                        auto nfa_node = nfa_->GetNode(i);
                        n->SetExtraData(nfa_node);
                        n->type_ |= nfa_node->type_;
                    }
                }

                n->index_ = dfa_nodes.size();

                bitset_node_map[subset.get()] = n.get();
                dfa_nodes.push_back(std::move(n));
            }

            // Transform BitSet-Edge map to DFA's Node-Edge map
            for (auto it = node_map_.Begin(); it != node_map_.End(); ++it)
            {
                auto bitset1 = it->first.first;
                auto bitset2 = it->second;
                auto edge = it->first.second;
                auto dfa_node1 = bitset_node_map[bitset1];
                auto dfa_node2 = bitset_node_map[bitset2];
                dfa_node_map.Set(dfa_node1, edge, dfa_node2);
            }

            return std::unique_ptr<DFA>(new DFA(std::move(dfa_nodes),
                                                std::move(dfa_node_map),
                                                nfa_->GetEdgeSet()));
        }

        void DFAConstructor::Debug() const
        {
            std::cout << "subsets:" << std::endl;
            for (auto it = subsets_.Begin(); it != subsets_.End(); ++it)
            {
                std::cout << "  " << it->get() << ": ";
                if ((*it)->IsSet(nfa_->GetStartNode()->index_))
                    std::cout << "start ";
                if ((*it)->IsSet(nfa_->GetAcceptNode()->index_))
                    std::cout << "accept ";
                std::cout << std::endl;
            }

            std::cout << "subset-edge map:" << std::endl;
            for (auto it = node_map_.Begin(); it != node_map_.End(); ++it)
            {
                std::cout << "  " << it->first.first << " -> " << it->first.second
                << " -> " << it->second << std::endl;
            }
        }

        void DFAConstructor::ConstructEpsilonClosure()
        {
            std::queue<const Node *> work_list;
            std::set<std::pair<const Node *, const Node *>> pre_epsilon_node;
            const Node *min_ptr = 0x0;
            const Node *max_ptr = reinterpret_cast<const Node *>(-1);

            std::size_t node_count = nfa_->NodeCount();
            for (std::size_t i = 0; i < node_count; ++i)
            {
                auto node = nfa_->GetNode(i);
                work_list.push(node);

                auto bitset = new BitSet(node_count);
                bitset->Set(i);
                epsilon_extend_.push_back(std::unique_ptr<BitSet>(bitset));
            }

            while (!work_list.empty())
            {
                auto node = work_list.front();
                work_list.pop();

                bool changed = false;
                auto &bitset = epsilon_extend_[node->index_];

                auto range = nfa_->MapEqualRange(node, nfa_->GetEpsilon());
                for (; range.first != range.second; ++range.first)
                {
                    auto next_node = range.first->second;
                    pre_epsilon_node.insert(std::make_pair(next_node, node));

                    if (bitset->MergeFrom(*epsilon_extend_[next_node->index_]))
                        changed = true;
                }

                if (changed)
                {
                    auto it = pre_epsilon_node.lower_bound(std::make_pair(node, min_ptr));
                    auto end = pre_epsilon_node.upper_bound(std::make_pair(node, max_ptr));

                    for (; it != end; ++it)
                        work_list.push(it->second);
                }
            }
        }

        bool DFAConstructor::EpsilonClosure(BitSet &bitset, const Node *node)
        {
            return bitset.MergeFrom(*epsilon_extend_[node->index_]);
        }

        const BitSet * DFAConstructor::ConstructStartBitSet()
        {
            auto start_index = nfa_->GetStartNode()->index_;
            return subsets_.AddBitSet(*epsilon_extend_[start_index]).first;
        }

        std::pair<const BitSet *, bool> DFAConstructor::ConstructDelta(const BitSet *q,
                                                                       const Edge *edge)
        {
            assert(nfa_->NodeCount() == q->ElemCount());
            BitSet bitset(nfa_->NodeCount());

            bool changed = false;
            auto count = q->ElemCount();
            for (std::size_t i = 0; i < count; ++i)
            {
                if (q->IsSet(i))
                {
                    auto node1 = nfa_->GetNode(i);
                    auto node2 = nfa_->Map(node1, edge);
                    if (node2 && EpsilonClosure(bitset, node2))
                        changed = true;
                }
            }

            if (changed)
            {
                return subsets_.AddBitSet(bitset);
            }
            else
            {
                return std::make_pair(nullptr, false);
            }
        }

        void DFAConstructor::SubsetConstruction()
        {
            auto q0 = ConstructStartBitSet();

            std::queue<const BitSet *> work_list;
            work_list.push(q0);

            auto edge_iter_pair = nfa_->GetEdgeIterators();

            while (!work_list.empty())
            {
                auto q = work_list.front();
                work_list.pop();

                for (auto it = edge_iter_pair.first;
                     it != edge_iter_pair.second; ++it)
                {
                    ConstructDeltaByEdge(q, &(*it), work_list);
                }

                ConstructDeltaByEdge(q, nfa_->GetDirectEdge(), work_list);
            }
        }

        void DFAConstructor::ConstructDeltaByEdge(const BitSet *q,
                                                  const Edge *edge,
                                                  std::queue<const BitSet *> &work_list)
        {
            auto next = ConstructDelta(q, edge);
            // We set bitset-edge map when the next bitset
            // is existed.
            if (next.first)
                node_map_.Set(q, edge, next.first);

            if (next.second)
                work_list.push(next.first);
        }
    } // namespace automata
} // namespace regex
