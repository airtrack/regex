#include "RegexAutomata.h"
#include "RegexParser.h"
#include <iostream>
#include <list>
#include <queue>
#include <set>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <assert.h>

namespace regex
{
    namespace automata
    {
        using namespace parser;

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
            : index_(index), type_(t)
            {
            }

            const char * TypeDesc() const
            {
                switch (type_)
                {
                    case Type_Start:
                        return "Type_Start";
                    case Type_Accept:
                        return "Type_Accept";
                    case Type_Normal:
                        return "Type_Normal";
                }
            }
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

        bool operator < (const Edge &left, const Edge &right)
        {
            // We assume left and right are equivalent when left is
            // right's subset or right is left's subset, so we return false.
            if (left.first_ >= right.first_ && left.last_ <= right.last_)
                return false;
            else if (right.first_ >= left.first_ && right.last_ <= left.last_)
                return false;
            else
                return left.last_ < right.first_;
        }

        class EdgeSet
        {
        public:
            typedef std::set<Edge> EdgeContainer;
            typedef EdgeContainer::iterator Iterator;

            EdgeSet() { }
            EdgeSet(const EdgeSet &) = delete;
            void operator = (const EdgeSet &) = delete;

            Iterator Begin() const
            {
                return edges_.begin();
            }

            Iterator End() const
            {
                return edges_.end();
            }

            std::pair<Iterator, Iterator> Search(int c) const
            {
                return Search(c, c);
            }

            std::pair<Iterator, Iterator> Search(int first, int last) const
            {
                return Search(Edge(first, last));
            }

            std::pair<Iterator, Iterator> Search(const Edge &range) const
            {
                Edge first(range.first_, range.first_);
                Edge last(range.last_, range.last_);
                return std::make_pair(edges_.lower_bound(first),
                                      edges_.upper_bound(last));
            }

            void Insert(int c)
            {
                Insert(Edge(c, c));
            }

            void Insert(int first, int last)
            {
                Insert(Edge(first, last));
            }

            void Insert(const Edge &range)
            {
                int current = range.first_;
                int last = range.last_;

                while (current <= last)
                {
                    Edge cur(current, current);
                    Iterator it = edges_.lower_bound(cur);
                    if (it == edges_.end())
                    {
                        edges_.insert(Edge(current, last));
                        current = last + 1;
                    }
                    else
                    {
                        if (current < it->first_)
                        {
                            if (last < it->first_)
                            {
                                edges_.insert(Edge(current, last));
                                current = last + 1;
                            }
                            else
                            {
                                edges_.insert(Edge(current, it->first_ - 1));
                                current = it->first_;
                            }
                        }
                        else if (current == it->first_)
                        {
                            if (last < it->last_)
                            {
                                int back_first = last + 1;
                                int back_last = it->last_;
                                edges_.erase(it);
                                edges_.insert(Edge(current, last));
                                edges_.insert(Edge(back_first, back_last));
                                current = last + 1;
                            }
                            else
                            {
                                current = it->last_ + 1;
                            }
                        }
                        else
                        {
                            if (last < it->last_)
                            {
                                int front_first = it->first_;
                                int front_last = current - 1;
                                int middle_first = current;
                                int middle_last = last;
                                int back_first = last + 1;
                                int back_last = it->last_;
                                edges_.erase(it);
                                edges_.insert(Edge(front_first, front_last));
                                edges_.insert(Edge(middle_first, middle_last));
                                edges_.insert(Edge(back_first, back_last));
                                current = last + 1;
                            }
                            else
                            {
                                int front_first = it->first_;
                                int front_last = current - 1;
                                int back_first = current;
                                int back_last = it->last_;
                                edges_.erase(it);
                                edges_.insert(Edge(front_first, front_last));
                                edges_.insert(Edge(back_first, back_last));
                                current = back_last + 1;
                            }
                        }
                    }
                }
            }

        private:
            EdgeContainer edges_;
        };

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

        private:
            EdgeSet *edge_set_;
        };

        void EdgeSetConstructorVisitor::Visit(CharNode *ast, void *data)
        {
            edge_set_->Insert(ast->c_);
        }

        void EdgeSetConstructorVisitor::Visit(CharRangeNode *ast, void *data)
        {
            edge_set_->Insert(ast->first_, ast->last_);
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

        // Template class to map (NodeType, EdgeType) to NodeType
        template<typename NodeType, typename EdgeType>
        class NodeMap
        {
        public:
            struct NodeEdgeHash
            {
                std::size_t operator () (const std::pair<const NodeType *,
                                         const EdgeType *> &pair) const
                {
                    return std::hash<const NodeType *>()(pair.first) *
                           std::hash<const EdgeType *>()(pair.second);
                }
            };

            typedef std::unordered_multimap<std::pair<const NodeType *, const EdgeType *>,
                                            const NodeType *, NodeEdgeHash> NodeEdgeMap;
            typedef typename NodeEdgeMap::const_iterator Iterator;

            NodeMap() { }

            NodeMap(NodeMap &&) = default;
            NodeMap& operator = (NodeMap &&) = default;

            NodeMap(const NodeMap &) = delete;
            void operator = (const NodeMap &) = delete;

            // Get first node-edge mapped node
            const NodeType * Map(const NodeType *node, const EdgeType *edge) const
            {
                auto key = std::make_pair(node, edge);
                auto it = map_.find(key);
                if (it != map_.end())
                    return it->second;
                else
                    return nullptr;
            }

            // Add node1-edge-node2 map
            void Set(const NodeType *node1, const EdgeType *edge, const NodeType *node2)
            {
                auto key = std::make_pair(node1, edge);
                map_.insert(std::make_pair(key, node2));
            }

            std::pair<Iterator, Iterator> EqualRange(const NodeType *node,
                                                     const EdgeType *edge) const
            {
                return map_.equal_range(std::make_pair(node, edge));
            }

            Iterator Begin() const
            {
                return map_.begin();
            }

            Iterator End() const
            {
                return map_.end();
            }

        private:
            NodeEdgeMap map_;
        };

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
            const Node * GetNode(std::size_t index) const
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

            void Debug() const
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

        // Visitor for convert AST to NFA
        class NFAConverterVisitor : public Visitor
        {
        public:
            typedef std::pair<const Node *, const Node *> DataType;

            explicit NFAConverterVisitor(NFA *nfa) : nfa_(nfa) { }

            NFAConverterVisitor(const NFAConverterVisitor &) = delete;
            void operator = (const NFAConverterVisitor &) = delete;

            VISIT_NODE(CharNode);
            VISIT_NODE(CharRangeNode);
            VISIT_NODE(ConcatenationNode);
            VISIT_NODE(AlternationNode);
            VISIT_NODE(ClosureNode);

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

            auto edges = nfa_->SearchEdge(ast->first_, ast->last_);
            for (; edges.first != edges.second; ++edges.first)
            {
                nfa_->SetMap(node1, &(*edges.first), node2);
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

            for (std::size_t i = 1; i < ast->nodes_.size(); ++i)
            {
                ast->nodes_[i]->Accept(this, &pair);
                nfa_->SetMap(last, nfa_->GetEpsilon(), pair.first);
                last = pair.second;
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

        // Bit set for subset construction
        class BitSet
        {
            friend struct BitSetPtrHash;
        public:
            explicit BitSet(std::size_t elem_count);

            // Merge other to itself, return true when
            // merge change itself.
            bool MergeFrom(const BitSet &other);

            // Set has index element
            void Set(std::size_t index);

            // Is index element set
            bool IsSet(std::size_t index) const;

            // Get element count of set
            std::size_t ElemCount() const
            {
                return elem_count_;
            }

            friend bool operator == (const BitSet &left,
                                     const BitSet &right)
            {
                return left.elem_count_ == right.elem_count_ &&
                       left.bits_ == right.bits_;
            }

            friend bool operator != (const BitSet &left,
                                     const BitSet &right)
            {
                return !(left == right);
            }

        private:
            static const std::size_t kUnsignedIntBits = 32;
            std::size_t GetIntCount(std::size_t elem_count);

            std::size_t elem_count_;
            std::vector<unsigned int> bits_;
        };

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

        struct BitSetPtrHash
        {
            std::size_t operator () (const std::unique_ptr<BitSet> &bitset) const
            {
                std::hash<unsigned int> h;

                std::size_t result = 1;
                for (auto it = bitset->bits_.begin();
                     it != bitset->bits_.end(); ++it)
                    result *= h(*it);

                return result;
            }
        };

        struct BitSetPtrEqual
        {
            bool operator () (const std::unique_ptr<BitSet> &left,
                              const std::unique_ptr<BitSet> &right) const
            {
                return *left == *right;
            }
        };

        // Set of BitSet for subset construction
        class BitSetSet
        {
        public:
            typedef std::unordered_set<std::unique_ptr<BitSet>,
                                       BitSetPtrHash, BitSetPtrEqual> SetType;
            typedef SetType::const_iterator Iterator;

            BitSetSet() { }

            BitSetSet(const BitSetSet &) = delete;
            void operator = (const BitSetSet &) = delete;

            // Add a BitSet, return the pointer of BitSet in the
            // BitSetSet and the bitset inserted success or not.
            std::pair<const BitSet *, bool> AddBitSet(const BitSet &bitset)
            {
                auto p = new BitSet(bitset);
                auto pair = bitsets_.insert(std::unique_ptr<BitSet>(p));
                return std::make_pair(pair.first->get(), pair.second);
            }

            Iterator Begin() const
            {
                return bitsets_.begin();
            }

            Iterator End() const
            {
                return bitsets_.end();
            }

        private:
            SetType bitsets_;
        };

        class DFA
        {
        public:
            DFA(std::vector<std::unique_ptr<Node>> &&nodes,
                NodeMap<Node, Edge> &&node_map,
                const std::shared_ptr<EdgeSet> &edge_set);

            DFA(const DFA &) = delete;
            void operator = (const DFA &) = delete;

            std::unique_ptr<StateMachine> ConstructStateMachine() const;

            void Debug() const
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

        private:
            typedef std::list<const Node *> NodeList;
            typedef std::vector<std::unique_ptr<NodeList>> NodeListSet;
            typedef NodeMap<NodeList, Edge> NodeListEdgeMap;
            typedef std::unordered_map<const Node *, NodeList *> NodeOwnerMap;

            // For DFA minimization
            void Minimization();
            void DoMinimization(NodeOwnerMap &node_owner_map);
            void SplitAll(NodeOwnerMap &node_owner_map);
            void Split(NodeList &node_list,
                       NodeOwnerMap &node_owner_map);

            // All edges
            std::shared_ptr<EdgeSet> edge_set_;

            // Not minimization DFA's nodes and node-edge map
            std::vector<std::unique_ptr<Node>> nodes_;
            NodeMap<Node, Edge> node_map_;

            // Minimization DFA's node_lists and node_list-edge map
            NodeListSet node_list_set_;
            NodeListEdgeMap node_list_map_;
        };

        DFA::DFA(std::vector<std::unique_ptr<Node>> &&nodes,
                 NodeMap<Node, Edge> &&node_map,
                 const std::shared_ptr<EdgeSet> &edge_set)
            : edge_set_(edge_set),
              nodes_(std::move(nodes)),
              node_map_(std::move(node_map))
        {
            Minimization();
        }

        std::unique_ptr<StateMachine> DFA::ConstructStateMachine() const
        {
            std::unique_ptr<StateMachine> state_machine(new StateMachine);
            std::unordered_map<const Edge *, std::size_t> edge_index;
            std::unordered_map<const NodeList *, State *> node_list_state;

            // Transform edges to char set
            for (auto it = edge_set_->Begin(); it != edge_set_->End(); ++it)
            {
                auto index = state_machine->char_set_.size();
                edge_index[&(*it)] = index;
                state_machine->char_set_.push_back(CharRange(it->first_, it->last_));
            }

            // Transform node_lists to states
            for (auto &node_list : node_list_set_)
            {
                bool accept = false;
                bool start = false;
                for (auto node : *node_list)
                {
                    switch (node->type_)
                    {
                        case Node::Type_Accept:
                            accept = true;
                            break;
                        case Node::Type_Start:
                            start = true;
                            break;
                        default:
                            break;
                    }
                }

                auto *state = new State(accept, state_machine->char_set_.size());
                state_machine->states_.push_back(std::unique_ptr<State>(state));
                node_list_state[node_list.get()] = state;

                if (start)
                {
                    assert(!state_machine->start_state_);
                    state_machine->start_state_ = state;
                }
            }
            assert(state_machine->start_state_);

            // Fill all state transition
            for (auto it = node_list_map_.Begin(); it != node_list_map_.End(); ++it)
            {
                auto from_state = node_list_state[it->first.first];
                auto to_state = node_list_state[it->second];
                auto index = edge_index[it->first.second];

                assert(from_state);
                assert(to_state);
                assert(index < from_state->next_.size());
                assert(index < state_machine->char_set_.size());

                from_state->next_[index] = to_state;
            }

            return std::move(state_machine);
        }

        void DFA::Minimization()
        {
            std::unique_ptr<NodeList> accept_node_list(new NodeList);
            std::unique_ptr<NodeList> not_accept_node_list(new NodeList);
            NodeOwnerMap node_owner_map;

            for (auto &n : nodes_)
            {
                if (n->type_ == Node::Type_Accept)
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
            assert(!node_list.empty());

            for (auto edge_it = edge_set_->Begin();
                 edge_it != edge_set_->End(); ++edge_it)
            {
                auto edge = &(*edge_it);
                auto node_it = node_list.begin();
                auto mapped_node_list = node_list_map_.Map(&node_list, edge);
                if (!mapped_node_list)
                {
                    const Node *mapped_node = nullptr;
                    while (!mapped_node && node_it != node_list.end())
                        mapped_node = node_map_.Map(*node_it++, edge);
                    if (mapped_node)
                    {
                        mapped_node_list = node_owner_map[mapped_node];
                        node_list_map_.Set(&node_list, edge, mapped_node_list);
                    }
                }

                NodeList *new_list = nullptr;
                while (node_it != node_list.end())
                {
                    auto mapped_node = node_map_.Map(*node_it, edge);
                    if (mapped_node)
                    {
                        auto mapped_node_owner = node_owner_map[mapped_node];
                        if (mapped_node_owner != mapped_node_list)
                        {
                            if (!new_list)
                            {
                                new_list = new NodeList;
                                node_list_set_.push_back(std::unique_ptr<NodeList>(new_list));
                                node_list_map_.Set(new_list, edge, mapped_node_owner);
                            }

                            node_owner_map[*node_it] = new_list;
                            new_list->splice(new_list->end(), node_list, node_it++);
                            continue;
                        }
                    }

                    ++node_it;
                }

                // We split new_list from node_list, so we split completely and return.
                if (new_list)
                    return ;
            }
        }

        // Construct DFA from NFA
        class DFAConstructor
        {
        public:
            explicit DFAConstructor(std::unique_ptr<NFA> nfa);

            DFAConstructor(const DFAConstructor &) = delete;
            void operator = (const DFAConstructor &) = delete;

            std::unique_ptr<DFA> ConstructDFA() const;

            void Debug() const
            {
                std::cout << "subsets:" << std::endl;
                for (auto it = subsets_.Begin(); it != subsets_.End(); ++it)
                {
                    std::cout << "  " << it->get() << ":";
                    if ((*it)->IsSet(nfa_->GetStartNode()->index_))
                        std::cout << " start ";
                    if ((*it)->IsSet(nfa_->GetAcceptNode()->index_))
                        std::cout << " accept";
                    std::cout << std::endl;
                }

                std::cout << "subset-edge map:" << std::endl;
                for (auto it = node_map_.Begin(); it != node_map_.End(); ++it)
                {
                    std::cout << "  " << it->first.first << " -> " << it->first.second
                              << " -> " << it->second << std::endl;
                }
            }

        private:
            // Construct epsilon closure
            void ConstructEpsilonClosure();

            // Epsilon closure the bitset from node, return true
            // when bitset changed
            bool EpsilonClosure(BitSet &bitset, const Node *node);

            // Subset construct start BitSet from NFA
            const BitSet * ConstructStartBitSet();

            // First value of return pair is next BitSet from 'q' BitSet through edge,
            // second value of return pair indicate first value of return pair need add
            // to work list or not
            std::pair<const BitSet *, bool> ConstructDelta(const BitSet *q,
                                                           const Edge *edge);

            // Construct subsets_ from NFA
            void SubsetConstruction();

            std::unique_ptr<NFA> nfa_;
            BitSetSet subsets_;
            NodeMap<BitSet, Edge> node_map_;

            std::vector<std::unique_ptr<BitSet>> epsilon_extend_;
        };

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
            auto nfa_start = nfa_->GetStartNode();
            auto nfa_accept = nfa_->GetAcceptNode();
            for (auto it = subsets_.Begin(); it != subsets_.End(); ++it)
            {
                auto index = dfa_nodes.size();
                Node::Type type = Node::Type_Normal;

                if ((*it)->IsSet(nfa_start->index_))
                    type = Node::Type_Start;
                else if ((*it)->IsSet(nfa_accept->index_))
                    type = Node::Type_Accept;

                std::unique_ptr<Node> n(new Node(index, type));
                bitset_node_map[it->get()] = n.get();
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

        void DFAConstructor::ConstructEpsilonClosure()
        {
            std::queue<const Node *> work_list;
            std::set<std::pair<const Node *, const Node *>> pre_epsilon_node;
            const Node *min_ptr = std::numeric_limits<const Node *>::min();
            const Node *max_ptr = std::numeric_limits<const Node *>::max();

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
                    auto next = ConstructDelta(q, &(*it));
                    // We set bitset-edge map when the next bitset
                    // is existed.
                    if (next.first)
                        node_map_.Set(q, &(*it), next.first);

                    if (next.second)
                        work_list.push(next.first);
                }
            }
        }

        std::unique_ptr<StateMachine> ConstructStateMachine(const std::string &re)
        {
            auto ast = Parse(re);
            auto nfa = ConvertASTToNFA(std::move(ast));
            nfa->Debug();

            std::cout << std::endl;

            DFAConstructor dfa_constructor(std::move(nfa));
            dfa_constructor.Debug();

            std::cout << std::endl;

            auto dfa = dfa_constructor.ConstructDFA();
            dfa->Debug();

            return dfa->ConstructStateMachine();
        }
    } // namespace automata
} // namespace regex
