#ifndef REGEX_CONSTRUCTOR_H
#define REGEX_CONSTRUCTOR_H

#include "RegexParser.h"
#include "RegexFAElement.h"
#include "RegexNFA.h"
#include "RegexDFA.h"
#include <memory>
#include <vector>
#include <queue>
#include <unordered_set>

namespace regex
{
    namespace automata
    {
        using namespace parser;

        std::unique_ptr<NFA> ConvertASTToNFA(const std::unique_ptr<ASTNode> &ast);

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

        // Construct DFA from NFA
        class DFAConstructor
        {
        public:
            explicit DFAConstructor(std::unique_ptr<NFA> nfa);

            DFAConstructor(const DFAConstructor &) = delete;
            void operator = (const DFAConstructor &) = delete;

            std::unique_ptr<DFA> ConstructDFA() const;

            void Debug() const;

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
    } // namespace automata
} // namespace regex

#endif // REGEX_CONSTRUCTOR_H
