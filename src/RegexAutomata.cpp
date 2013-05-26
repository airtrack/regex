#include "RegexAutomata.h"
#include "RegexParser.h"
#include <vector>
#include <queue>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <assert.h>

namespace regex
{
    namespace automata
    {
        using namespace parser;

        // Template class to map (NodeType, EdgeType) to NodeType
        template<typename NodeType, typename EdgeType>
        class NodeMap
        {
        public:
            NodeMap() { }
            NodeMap(const NodeMap &) = delete;
            void operator = (const NodeMap &) = delete;

            // Get first node-edge mapped node
            const NodeType * Map(const NodeType *node, const EdgeType *edge)
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

        private:
            struct NodeEdgeHash
            {
                std::size_t operator () (const std::pair<const NodeType *,
                                                         const EdgeType *> &pair) const
                {
                    return std::hash<const NodeType *>()(pair.first) *
                           std::hash<const EdgeType *>()(pair.second);
                }
            };

            std::unordered_multimap<std::pair<const NodeType *, const EdgeType *>,
                                    const NodeType *, NodeEdgeHash> map_;
        };

        class NFA
        {
        public:
            NFA() : start_(nullptr), accept_(nullptr) { }
            NFA(const NFA &) = delete;
            void operator = (const NFA &) = delete;

            typedef std::vector<std::unique_ptr<Node>> NodeList;
            typedef std::vector<std::unique_ptr<Edge>> EdgeList;
            typedef EdgeList::const_iterator EdgeIterator;

            const Edge * AddEdge(const std::function<bool (int)> &e)
            {
                auto edge = new Edge(e);
                edges_.push_back(std::unique_ptr<Edge>(edge));
                return edge;
            }

            const Node * AddNode(Node::Type t = Node::Type_Normal)
            {
                auto index = nodes_.size();
                auto node = new Node(index, t);
                nodes_.push_back(std::unique_ptr<Node>(node));
                return node;
            }

            void SetMap(const Node *node1, const Edge *edge, const Node *node2)
            {
                node_map_.Set(node1, edge, node2);
            }

            const Node * Map(const Node *node, const Edge *edge)
            {
                return node_map_.Map(node, edge);
            }

            const Edge * GetEpsilon() const
            {
                return &epsilon_;
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

            // Get edge begin and end iterator pair
            std::pair<EdgeIterator, EdgeIterator> GetEdgeIterators() const
            {
                return std::make_pair(edges_.begin(), edges_.end());
            }

            // Get node by index
            const Node * GetNode(std::size_t index) const
            {
                if (index < nodes_.size())
                    return nodes_[index].get();
                else
                    return nullptr;
            }

        private:
            EdgeList edges_;
            NodeList nodes_;
            NodeMap<Node, Edge> node_map_;
            Edge epsilon_;

            const Node *start_;
            const Node *accept_;
        };

        // Visitor for convert AST to NFA
        class NFAConverterVisitor : public Visitor
        {
        public:
            typedef std::pair<const Node *, const Node *> DataType;

            explicit NFAConverterVisitor(NFA *nfa) : nfa_(nfa) { }

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
            auto edge = nfa_->AddEdge([ast](int c) { return c == ast->c_; });
            nfa_->SetMap(node1, edge, node2);

            FillData(data, node1, node2);
        }

        void NFAConverterVisitor::Visit(CharRangeNode *ast, void *data)
        {
            auto node1 = nfa_->AddNode();
            auto node2 = nfa_->AddNode();

            std::function<bool (int)> f;
            if (ast->first_ <= ast->last_)
                f = [ast] (int c) { return c >= ast->first_ && c <= ast->last_; };
            else
                f = [ast] (int c) { return c == ast->first_ || c == ast->last_; };

            auto edge = nfa_->AddEdge(f);
            nfa_->SetMap(node1, edge, node2);

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

            for (auto it = ast->nodes_.begin(); it != ast->nodes_.end(); ++it)
            {
                DataType pair;
                (*it)->Accept(this, &pair);
                all_first.push_back(pair.first);
                all_last.push_back(pair.second);
            }

            auto node1 = nfa_->AddNode();
            for (auto it = all_first.begin(); it != all_first.end(); ++it)
                nfa_->SetMap(node1, nfa_->GetEpsilon(), *it);

            auto node2 = nfa_->AddNode();
            for (auto it = all_last.begin(); it != all_last.end(); ++it)
                nfa_->SetMap(*it, nfa_->GetEpsilon(), node2);

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
            std::unique_ptr<NFA> nfa(new NFA);

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
            friend struct BitSetHash;
        public:
            explicit BitSet(std::size_t elem_count);

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

        void BitSet::Set(std::size_t index)
        {
            std::size_t unsigned_int_index = index / kUnsignedIntBits;
            std::size_t bit_index = index % kUnsignedIntBits;

            if (unsigned_int_index < bits_.size())
                bits_[unsigned_int_index] |= 0x1 >> bit_index;
        }

        bool BitSet::IsSet(std::size_t index) const
        {
            std::size_t unsigned_int_index = index / kUnsignedIntBits;
            std::size_t bit_index = index % kUnsignedIntBits;

            if (unsigned_int_index < bits_.size())
                return (bits_[unsigned_int_index] & (0x1 >> bit_index)) != 0;
            else
                return false;
        }

        std::size_t BitSet::GetIntCount(std::size_t elem_count)
        {
            return elem_count / kUnsignedIntBits +
                  (elem_count % kUnsignedIntBits == 0 ? 0 : 1);
        }

        struct BitSetHash
        {
            std::size_t operator () (const BitSet &bitset) const
            {
                std::hash<unsigned int> h;

                std::size_t result = 1;
                for (auto it = bitset.bits_.begin(); it != bitset.bits_.end(); ++it)
                    result *= h(*it);

                return result;
            }
        };

        // Set of BitSet for subset construction
        class BitSetSet
        {
        public:
            BitSetSet() { }

            BitSetSet(const BitSetSet &) = delete;
            void operator = (const BitSetSet &) = delete;

            // Add a BitSet, return the pointer of BitSet in the
            // BitSetSet and the bitset is existed or not
            std::pair<const BitSet *, bool> AddBitSet(const BitSet &bitset)
            {
                auto pair = bitsets_.insert(bitset);
                return std::make_pair(&(*pair.first), !pair.second);
            }

        private:
            std::unordered_set<BitSet, BitSetHash> bitsets_;
        };

        // Epsilon closure the bitset
        void EpsilonClosure(const std::unique_ptr<NFA> &nfa, BitSet &bitset)
        {
        }

        // Subset construct start BitSet from NFA
        const BitSet * ConstructStartBitSet(const std::unique_ptr<NFA> &nfa,
                                            BitSetSet &subsets)
        {
            BitSet q0(nfa->NodeCount());
            q0.Set(nfa->GetStartNode()->index_);
            EpsilonClosure(nfa, q0);
            return subsets.AddBitSet(q0).first;
        }

        // First value of return pair is next BitSet from 'q' BitSet through edge,
        // second value of return pair indicate first value of return pair need add
        // to work list or not
        std::pair<const BitSet *, bool> ConstructDelta(const std::unique_ptr<NFA> &nfa,
                                                       const BitSet *q,
                                                       const Edge *edge,
                                                       BitSetSet &subsets)
        {
            assert(nfa->NodeCount() == q->ElemCount());
            BitSet t(nfa->NodeCount());

            auto count = q->ElemCount();
            for (std::size_t i = 0; i < count; ++i)
            {
                if (q->IsSet(i))
                {
                    auto node1 = nfa->GetNode(i);
                    auto node2 = nfa->Map(node1, edge);
                    if (node2)
                        t.Set(node2->index_);
                }
            }

            EpsilonClosure(nfa, t);
            auto pair = subsets.AddBitSet(t);

            return std::make_pair(pair.first, !pair.second);
        }

        void SubsetConstruction(const std::unique_ptr<NFA> &nfa)
        {
            BitSetSet subsets;
            auto q0 = ConstructStartBitSet(nfa, subsets);

            std::queue<const BitSet *> work_list;
            work_list.push(q0);

            auto edge_iter_pair = nfa->GetEdgeIterators();

            NodeMap<BitSet, Edge> node_map;
            while (!work_list.empty())
            {
                auto q = work_list.front();
                work_list.pop();

                for (auto it = edge_iter_pair.first;
                     it != edge_iter_pair.second; ++it)
                {
                    auto pair = ConstructDelta(nfa, q, (*it).get(), subsets);
                    node_map.Set(q, (*it).get(), pair.first);

                    if (pair.second)
                        work_list.push(pair.first);
                }
            }
        }
    } // namespace automata
} // namespace regex
