#ifndef REGEX_FA_ELEMENT_H
#define REGEX_FA_ELEMENT_H

#include <set>
#include <memory>
#include <string>
#include <limits>
#include <unordered_map>

namespace regex
{
    namespace automata
    {
        class NFA;

        struct Node
        {
            enum Type
            {
                Type_Start = (1 << 0),
                Type_Accept = (1 << 1),
                Type_Normal = (1 << 2),
                Type_Repeat = (1 << 3),
                Type_LineHead = (1 << 4),
                Type_LineTail = (1 << 5),
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

            std::string TypeDesc() const;
        };

        struct Edge
        {
            // Edge min max value
            static const int Min = 0;
            static const int Max = std::numeric_limits<int>::max() - 1;

            // Character range [first_, last_]
            int first_;
            int last_;

            Edge() : first_(0), last_(0) { }
            Edge(int first, int last) : first_(first), last_(last) { }
        };

        inline bool operator < (const Edge &left, const Edge &right)
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

            void Insert(const Edge &range);

        private:
            Edge direct_edge_;
            EdgeContainer edges_;
        };

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
    } // namespace automata
} // namespace regex

#endif // REGEX_FA_ELEMENT_H
