#ifndef REGEX_PARSER_H
#define REGEX_PARSER_H

#include <memory>
#include <vector>
#include <string>

namespace regex
{
    namespace parser
    {
        class Visitor;

#define ACCEPT_VISITOR()                                \
    virtual void Accept(Visitor *v, void *data)

#define VISIT_NODE(node)                                \
    virtual void Visit(node *n, void *data)

        // Abstract syntax tree node for regex
        class ASTNode
        {
        public:
            ACCEPT_VISITOR() = 0;
            virtual ~ASTNode() { }
        };

        // Node for one character
        class CharNode : public ASTNode
        {
        public:
            explicit CharNode(char c) : c_(c) { }

            ACCEPT_VISITOR();

            char c_;
        };

        // Node for character range. e.g: [a-z]
        class CharRangeNode : public ASTNode
        {
        public:
            CharRangeNode(char f, char l) : first_(f), last_(l) { }

            ACCEPT_VISITOR();

            char first_;
            char last_;
        };

        // Concat two or more regex together. e.g: a[a-z]b
        class ConcatenationNode : public ASTNode
        {
        public:
            void AddNode(std::unique_ptr<ASTNode> node)
            {
                nodes_.push_back(std::move(node));
            }

            ACCEPT_VISITOR();

            std::vector<std::unique_ptr<ASTNode>> nodes_;
        };

        // Alternation for two or more regex. e.g: a | b | c
        class AlternationNode : public ASTNode
        {
        public:
            void AddNode(std::unique_ptr<ASTNode> node)
            {
                nodes_.push_back(std::move(node));
            }

            ACCEPT_VISITOR();

            std::vector<std::unique_ptr<ASTNode>> nodes_;
        };

        // Node for repeat zero or more regex. e.g: a*
        class ClosureNode : public ASTNode
        {
        public:
            explicit ClosureNode(std::unique_ptr<ASTNode> node)
                : node_(std::move(node))
            {
            }

            ACCEPT_VISITOR();

            std::unique_ptr<ASTNode> node_;
        };

        class Visitor
        {
        public:
            VISIT_NODE(CharNode) = 0;
            VISIT_NODE(CharRangeNode) = 0;
            VISIT_NODE(ConcatenationNode) = 0;
            VISIT_NODE(AlternationNode) = 0;
            VISIT_NODE(ClosureNode) = 0;

            virtual ~Visitor() { }
        };

        std::unique_ptr<ASTNode> Parse(const std::string &re);
    } // namespace parser
} // namespace parser

#endif // REGEX_PARSER_H
