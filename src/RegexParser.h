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

        class ASTNode
        {
        public:
            ACCEPT_VISITOR() = 0;
            virtual ~ASTNode() { }
        };

        class CharNode : public ASTNode
        {
        public:
            explicit CharNode(char c) : c_(c) { }

            ACCEPT_VISITOR();

            char c_;
        };

        class CharRangeNode : public ASTNode
        {
        public:
            CharRangeNode(char f, char l) : first_(f), last_(l) { }

            ACCEPT_VISITOR();

            char first_;
            char last_;
        };

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
