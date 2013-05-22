#include "RegexAutomata.h"
#include "RegexParser.h"
#include <vector>
#include <memory>
#include <map>

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

            Node * Map(Node *node, Edge *edge);

        private:
            std::map<std::pair<Node *, Edge *>, Node *> map_;
        };

        class NFA
        {
        public:
            NFA() { }
            NFA(const NFA &) = delete;
            void operator = (const NFA &) = delete;

        private:
            std::vector<std::unique_ptr<Edge>> edges_;
            std::vector<std::unique_ptr<Node>> nodes_;
            NodeMap node_map_;
        };

        class NFAConverterVisitor : public Visitor
        {
        public:
            explicit NFAConverterVisitor(NFA *nfa) : nfa_(nfa) { }

            VISIT_NODE(CharNode);
            VISIT_NODE(CharRangeNode);
            VISIT_NODE(ConcatenationNode);
            VISIT_NODE(AlternationNode);
            VISIT_NODE(ClosureNode);

        private:
            NFA *nfa_;
        };

        void NFAConverterVisitor::Visit(CharNode *node, void *data)
        {
        }

        void NFAConverterVisitor::Visit(CharRangeNode *node, void *data)
        {
        }

        void NFAConverterVisitor::Visit(ConcatenationNode *node, void *data)
        {
        }

        void NFAConverterVisitor::Visit(AlternationNode *node, void *data)
        {
        }

        void NFAConverterVisitor::Visit(ClosureNode *node, void *data)
        {
        }

        std::unique_ptr<NFA> ConvertASTToNFA(const std::unique_ptr<ASTNode> &ast)
        {
            std::unique_ptr<NFA> nfa(new NFA);

            NFAConverterVisitor visitor(nfa.get());
            ast->Accept(&visitor, nullptr);

            return std::move(nfa);
        }
    } // namespace automata
} // namespace regex
