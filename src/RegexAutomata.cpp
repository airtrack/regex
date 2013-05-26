#include "RegexAutomata.h"
#include "RegexParser.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <assert.h>

namespace regex
{
    namespace automata
    {
        using namespace parser;

        class NodeMap
        {
        public:
            NodeMap() { }
            NodeMap(const NodeMap &) = delete;
            void operator = (const NodeMap &) = delete;

            // Get first node-edge mapped node
            const Node * Map(const Node *node, const Edge *edge)
            {
                auto key = std::make_pair(node, edge);
                auto it = map_.find(key);
                if (it != map_.end())
                    return it->second;
                else
                    return nullptr;
            }

            // Add node1-edge-node2 map
            void Set(const Node *node1, const Edge *edge, const Node *node2)
            {
                auto key = std::make_pair(node1, edge);
                map_.insert(std::make_pair(key, node2));
            }

        private:
            struct NodeEdgeHash
            {
                std::size_t operator () (const std::pair<const Node *,
                                                         const Edge *> &pair) const
                {
                    return std::hash<const Node *>()(pair.first) *
                           std::hash<const Edge *>()(pair.second);
                }
            };

            std::unordered_multimap<std::pair<const Node *, const Edge *>,
                                    const Node *, NodeEdgeHash> map_;
        };

        class NFA
        {
        public:
            NFA() : start_(nullptr), accept_(nullptr) { }
            NFA(const NFA &) = delete;
            void operator = (const NFA &) = delete;

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

        private:
            std::vector<std::unique_ptr<Edge>> edges_;
            std::vector<std::unique_ptr<Node>> nodes_;
            NodeMap node_map_;
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
            std::vector<const Node *> allFirst;
            std::vector<const Node *> allLast;

            for (auto it = ast->nodes_.begin(); it != ast->nodes_.end(); ++it)
            {
                DataType pair;
                (*it)->Accept(this, &pair);
                allFirst.push_back(pair.first);
                allLast.push_back(pair.second);
            }

            auto node1 = nfa_->AddNode();
            for (auto it = allFirst.begin(); it != allFirst.end(); ++it)
                nfa_->SetMap(node1, nfa_->GetEpsilon(), *it);

            auto node2 = nfa_->AddNode();
            for (auto it = allLast.begin(); it != allLast.end(); ++it)
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
            explicit BitSet(std::size_t elemCount);

            // Set has index element
            void Set(std::size_t index);

            // Is index element set
            bool IsSet(std::size_t index) const;

            friend bool operator == (const BitSet &left,
                                     const BitSet &right)
            {
                return left.elemCount_ == right.elemCount_ &&
                       left.bits_ == right.bits_;
            }

            friend bool operator != (const BitSet &left,
                                     const BitSet &right)
            {
                return !(left == right);
            }

        private:
            static const std::size_t kUnsignedIntBits = 32;
            std::size_t GetIntCount(std::size_t elemCount);

            std::size_t elemCount_;
            std::vector<unsigned int> bits_;
        };

        BitSet::BitSet(std::size_t elemCount)
            : elemCount_(elemCount),
              bits_(GetIntCount(elemCount))
        {
        }

        void BitSet::Set(std::size_t index)
        {
            std::size_t unsignedIntIndex = index / kUnsignedIntBits;
            std::size_t bitIndex = index % kUnsignedIntBits;

            if (unsignedIntIndex < bits_.size())
                bits_[unsignedIntIndex] |= 0x1 >> bitIndex;
        }

        bool BitSet::IsSet(std::size_t index) const
        {
            std::size_t unsignedIntIndex = index / kUnsignedIntBits;
            std::size_t bitIndex = index % kUnsignedIntBits;

            if (unsignedIntIndex < bits_.size())
                return (bits_[unsignedIntIndex] & (0x1 >> bitIndex)) != 0;
            else
                return false;
        }

        std::size_t BitSet::GetIntCount(std::size_t elemCount)
        {
            return elemCount / kUnsignedIntBits +
                  (elemCount % kUnsignedIntBits == 0 ? 0 : 1);
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

            // Add a BitSet and return the pointer of BitSet
            // in the BitSetSet
            const BitSet * AddBitSet(const BitSet &bitset)
            {
                auto pair = bitsets_.insert(bitset);
                return &(*pair.first);
            }

            // Get the pointer of BitSet from the BitSetSet
            const BitSet * GetBitSet(const BitSet &bitset)
            {
                auto it = bitsets_.find(bitset);
                if (it == bitsets_.end())
                    return nullptr;
                else
                    return &(*it);
            }

            // Is BitSetSet has BitSet or not
            bool HasBitSet(const BitSet &bitset)
            {
                return bitsets_.find(bitset) != bitsets_.end();
            }

        private:
            std::unordered_set<BitSet, BitSetHash> bitsets_;
        };
    } // namespace automata
} // namespace regex
