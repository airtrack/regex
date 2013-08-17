#include "RegexAutomata.h"
#include "RegexParser.h"
#include "RegexMatcher.h"
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
        class NFA;

        struct Node
        {
            enum Type
            {
                Type_Start = (1 << 0),
                Type_Accept = (1 << 1),
                Type_Normal = (1 << 2),
                Type_Repeat = (1 << 3),
            };

            // Type of this node
            int type_;

            // Index and number of this node
            std::size_t index_;

            // Union data of each type node
            union
            {
                // For Type_Repeat node
                struct
                {
                    std::unique_ptr<NFA> sub_nfa_;
                    int repeat_min_;
                    int repeat_max_;
                };
            };

            Node()
                : type_(0), index_(0),
                  repeat_min_(0), repeat_max_(0)
            {
            }

            Node(std::size_t index, int type)
                : type_(type), index_(index),
                  repeat_min_(0), repeat_max_(0)
            {
            }

            // Construct Type_Repeat node
            Node(std::size_t index, int repeat_min, int repeat_max,
                 std::unique_ptr<NFA> sub_nfa)
                : type_(Type_Repeat), index_(index),
                  repeat_min_(repeat_min), repeat_max_(repeat_max)
            {
                new(&sub_nfa_) std::unique_ptr<NFA>(std::move(sub_nfa));
            }

            ~Node()
            {
                DestructSubDFA();
            }

            void SetExtraData(Node *other)
            {
                if (other->type_ & Type_Repeat)
                {
                    repeat_min_ = other->repeat_min_;
                    repeat_max_ = other->repeat_max_;
                    DestructSubDFA();
                    new(&sub_nfa_) std::unique_ptr<NFA>(std::move(other->sub_nfa_));
                }
            }

            void DestructSubDFA()
            {
                if (type_ & Type_Repeat)
                    sub_nfa_.~unique_ptr();
            }

            std::string TypeDesc() const
            {
                std::string desc;
                if (type_ & Type_Start)
                {
                    desc += "Type_Start";
                }
                if (type_ & Type_Accept)
                {
                    if (!desc.empty())
                        desc += " ";
                    desc += "Type_Accept";
                }
                if (type_ & Type_Normal)
                {
                    if (!desc.empty())
                        desc += " ";
                    desc += "Type_Normal";
                }
                if (type_ & Type_Repeat)
                {
                    if (!desc.empty())
                        desc += " ";
                    desc += "Type_Repeat";
                }
                return desc;
            }
        };

        struct Edge
        {
            // Character range [first_, last_]
            int first_;
            int last_;

            Edge() : first_(0), last_(0) { }
            Edge(int first, int last) : first_(first), last_(last) { }
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

            const Edge * GetDirectEdge() const
            {
                return &direct_edge_;
            }

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
            Edge direct_edge_;
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
            VISIT_NODE(RepeatNode);

        private:
            EdgeSet *edge_set_;
        };

        void EdgeSetConstructorVisitor::Visit(CharNode *ast, void *data)
        {
            edge_set_->Insert(ast->c_);
        }

        void EdgeSetConstructorVisitor::Visit(CharRangeNode *ast, void *data)
        {
            for (auto &range : ast->ranges_)
                edge_set_->Insert(range.first_, range.last_);

            for (auto c : ast->chars_)
                edge_set_->Insert(c);
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

            for (auto &range : ast->ranges_)
            {
                auto edges = nfa_->SearchEdge(range.first_, range.last_);
                for (; edges.first != edges.second; ++edges.first)
                {
                    nfa_->SetMap(node1, &(*edges.first), node2);
                }
            }

            for (auto c : ast->chars_)
            {
                auto edges = nfa_->SearchEdge(c);
                for (; edges.first != edges.second; ++edges.first)
                {
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

        std::unique_ptr<NFA> ConvertASTToNFA(const std::unique_ptr<ASTNode> &ast);

        void NFAConverterVisitor::Visit(RepeatNode *ast, void *data)
        {
            // Construct sub-NFA of sub-regex
            auto sub_nfa = ConvertASTToNFA(ast->node_);
            auto node = nfa_->AddRepeatNode(ast->min_times_, ast->max_times_,
                                            std::move(sub_nfa));

            reinterpret_cast<DataType *>(data)->need_direct_edge = true;
            FillData(data, node, node);
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

            std::unique_ptr<StateMachine> ConstructStateMachine();

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

        std::unique_ptr<StateMachine> ConstructStateMachineFromNFA(std::unique_ptr<NFA> nfa,
                                                                   const char *debug);

        void DFA::TransformNodeListsToStates(std::unique_ptr<StateMachine> &state_machine,
                                             std::unordered_map<const NodeList *, State *> &node_list_state)
        {
            for (auto &node_list : node_list_set_)
            {
                if (node_list->empty())
                    continue;

                bool accept = false;
                bool start = false;
                int min = 0;
                int max = 0;
                std::unique_ptr<StateMachine> sub_state_machine;
                for (auto node : *node_list)
                {
                    if (node->type_ & Node::Type_Accept)
                        accept = true;
                    if (node->type_ & Node::Type_Start)
                        start = true;
                    if (node->type_ & Node::Type_Repeat)
                    {
                        min = node->repeat_min_;
                        max = node->repeat_max_;
                        sub_state_machine = ConstructStateMachineFromNFA(std::move(node->sub_nfa_),
                                                                         "Sub regex:");
                    }
                }

                State *state = nullptr;
                if (sub_state_machine)
                    state = new RepeatState(state_machine->char_set_.size(), min, max,
                                            std::move(sub_state_machine));
                else
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
                    RepeatState *from = static_cast<RepeatState *>(from_state);
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
            void ConstructDeltaByEdge(const BitSet *q,
                                      const Edge *edge,
                                      std::queue<const BitSet *> &work_list);

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

        const State * State::GetNextState(const StateMachine *state_machine,
                                          const char *&current, const char *&end) const
        {
            if (current == end)
                return nullptr;

            auto next = state_machine->GetNextStateIndex(*current++);
            if (!next.first)
                return nullptr;
            else
                return next_[next.second];
        }

        std::pair<bool, const State *> State::CompleteState(const char *&current,
                                                            const char *&end) const
        {
            return std::make_pair(true, nullptr);
        }

        std::pair<bool, std::size_t> StateMachine::GetNextStateIndex(int c) const
        {
            std::size_t left = 0;
            std::size_t right = char_set_.size();
            while (left < right)
            {
                std::size_t middle = (left + right) / 2;
                const CharRange &range = char_set_[middle];

                if (c >= range.first_ && c <= range.last_)
                    return std::make_pair(true, middle);
                else if (c < range.first_)
                    right = middle;
                else
                    left = middle + 1;
            }

            return std::make_pair(false, 0);
        }

        std::pair<bool, const State *> RepeatState::CompleteState(const char *&current,
                                                                  const char *&end) const
        {
            RegexMatcher matcher(sub_state_machine_.get());

            int match_times = 0;
            const char *last = current;
            while (match_times < repeat_max_ &&
                   matcher.MatchPart(current, end, &current))
            {
                ++match_times;
                // If 'matcher' match success but do not comsume
                // any character, then break when 'match_times'
                // >= 'repeat_min_'
                if (last == current && match_times >= repeat_min_)
                    break;
                last = current;
            }

            return std::make_pair(match_times >= repeat_min_, direct_next_);
        }

        std::unique_ptr<StateMachine> ConstructStateMachineFromNFA(std::unique_ptr<NFA> nfa,
                                                                   const char *debug)
        {
            std::cout << debug << std::endl;
            nfa->Debug();
            std::cout << std::endl;

            DFAConstructor dfa_constructor(std::move(nfa));
            dfa_constructor.Debug();
            std::cout << std::endl;

            auto dfa = dfa_constructor.ConstructDFA();
            dfa->Debug();

            return dfa->ConstructStateMachine();
        }

        std::unique_ptr<StateMachine> ConstructStateMachine(const std::string &re)
        {
            auto ast = Parse(re);
            auto nfa = ConvertASTToNFA(std::move(ast));
            return ConstructStateMachineFromNFA(std::move(nfa), "Main regex:");
        }
    } // namespace automata
} // namespace regex
