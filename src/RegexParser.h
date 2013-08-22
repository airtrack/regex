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
            explicit CharNode(int c) : c_(c) { }

            ACCEPT_VISITOR();

            int c_;
        };

        // Node for character range. e.g., [ ]
        class CharRangeNode : public ASTNode
        {
        public:
            struct Range
            {
                int first_;
                int last_;

                explicit Range(int first = 0, int last = 0)
                    : first_(first), last_(last)
                {
                }
            };

            CharRangeNode() { }

            void AddRange(int first, int last)
            {
                ranges_.push_back(Range(first, last));
            }

            void AddChar(int c)
            {
                chars_.push_back(c);
            }

            ACCEPT_VISITOR();

            std::vector<Range> ranges_;
            std::vector<int> chars_;
        };

        // Node for concatenate regex two or more. e.g., a[a-z]b
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

        // Node for alternate regex two or more. e.g., a | b | c
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

        // Node for repeat regex zero or more. e.g., a*
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

        // Node for repeat regex min to max times. e.g., a{1, 2}
        class RepeatNode : public ASTNode
        {
        public:
            RepeatNode(std::unique_ptr<ASTNode> node,
                       int min_times, int max_times, bool greedy)
                : node_(std::move(node)),
                  min_times_(min_times),
                  max_times_(max_times),
                  greedy_(greedy)
            {
            }

            ACCEPT_VISITOR();

            std::unique_ptr<ASTNode> node_;
            int min_times_;
            int max_times_;
            bool greedy_;
        };

        // Node for dot. e.g. '.'
        class DotNode : public ASTNode
        {
        public:
            ACCEPT_VISITOR();
        };

        // Node for question mark. e.g. "a?"
        class QuestionMarkNode : public ASTNode
        {
        public:
            explicit QuestionMarkNode(std::unique_ptr<ASTNode> node)
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
            VISIT_NODE(RepeatNode) = 0;
            VISIT_NODE(DotNode) = 0;
            VISIT_NODE(QuestionMarkNode) = 0;

            virtual ~Visitor() { }
        };

        std::unique_ptr<ASTNode> Parse(const std::string &re);
    } // namespace parser
} // namespace parser

#endif // REGEX_PARSER_H
