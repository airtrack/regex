#include "RegexParser.h"
#include "RegexException.h"

namespace regex
{
    namespace parser
    {
#define ACCEPT_VISITOR_IMPL(class_name)                     \
    void class_name::Accept(Visitor *v, void *data)         \
    {                                                       \
        v->Visit(this, data);                               \
    }

        ACCEPT_VISITOR_IMPL(CharNode)
        ACCEPT_VISITOR_IMPL(CharRangeNode)
        ACCEPT_VISITOR_IMPL(ConcatenationNode)
        ACCEPT_VISITOR_IMPL(AlternationNode)
        ACCEPT_VISITOR_IMPL(ClosureNode)

        class LexStream
        {
        public:
            explicit LexStream(const std::string &re) : re_(re), index_(0) { }

            LexStream(const LexStream &) = delete;
            void operator = (const LexStream &) = delete;

            int Get() const
            {
                if (index_ < re_.size())
                    return re_[index_];
                else
                    return EOF;
            }

            void Next()
            {
                ++index_;
            }

            std::size_t Index() const
            {
                return index_;
            }

        private:
            std::string re_;
            std::size_t index_;
        };

        bool IsEscapeChar(int c)
        {
            return c == '[' || c == ']' ||
                   c == '(' || c == ')' ||
                   c == '*' || c == '|';
        }

        std::unique_ptr<ASTNode> ParseRE(LexStream &stream);

        std::unique_ptr<ASTNode> ParseChar(LexStream &stream)
        {
            int c = stream.Get();
            if (c == EOF)
                throw ParseException("unexpect EOF", stream.Index());

            if (IsEscapeChar(c))
            {
                std::string err("unexpect '");
                err.push_back(c);
                err.push_back('\'');
                throw ParseException(err, stream.Index());
            }

            stream.Next();
            return std::unique_ptr<ASTNode>(new CharNode(c));
        }

        std::unique_ptr<ASTNode> ParseCharRange(LexStream &stream)
        {
            stream.Next();              // skip '['

            int first = stream.Get();

            if (first == EOF)
                throw ParseException("incomplete \"[a-b]\"", stream.Index());
            stream.Next();

            if (stream.Get() != '-')
                throw ParseException("expect '-'", stream.Index());
            stream.Next();              // skip '-'

            int last = stream.Get();

            if (last == EOF)
                throw ParseException("expect second char", stream.Index());
            stream.Next();

            if (stream.Get() != ']')
                throw ParseException("incomplete \"[a-b]\"", stream.Index());
            stream.Next();              // skip ']'

            return std::unique_ptr<ASTNode>(new CharRangeNode(first, last));
        }

        std::unique_ptr<ASTNode> ParseParentheseRE(LexStream &stream)
        {
            stream.Next();              // skip '('
            std::unique_ptr<ASTNode> node = ParseRE(stream);

            if (stream.Get() != ')')
                throw ParseException("incomplete \"()\"", stream.Index());
            stream.Next();

            return std::move(node);
        }

        std::unique_ptr<ASTNode> ParseBase(LexStream &stream)
        {
            if (stream.Get() == '[')
                return ParseCharRange(stream);
            else if (stream.Get() == '(')
                return ParseParentheseRE(stream);
            else
                return ParseChar(stream);
        }

        std::unique_ptr<ASTNode> ParseConcatBase(LexStream &stream)
        {
            std::unique_ptr<ASTNode> node = ParseBase(stream);

            if (stream.Get() == '*')
            {
                stream.Next();
                node = std::unique_ptr<ASTNode>(new ClosureNode(std::move(node)));
            }

            return std::move(node);
        }

        std::unique_ptr<ASTNode> ParseConcatenation(LexStream &stream)
        {
            std::unique_ptr<ConcatenationNode> concat(new ConcatenationNode);
            std::unique_ptr<ASTNode> node = ParseConcatBase(stream);

            concat->AddNode(std::move(node));

            while (stream.Get() != EOF && stream.Get() != '|' && stream.Get() != ')')
            {
                node = ParseConcatBase(stream);
                concat->AddNode(std::move(node));
            }

            return std::move(concat);
        }

        std::unique_ptr<ASTNode> ParseAlternation(LexStream &stream)
        {
            std::unique_ptr<AlternationNode> alter(new AlternationNode);
            std::unique_ptr<ASTNode> node = ParseConcatenation(stream);

            alter->AddNode(std::move(node));

            while (stream.Get() == '|')
            {
                stream.Next();
                node = ParseConcatenation(stream);
                alter->AddNode(std::move(node));
            }

            return std::move(alter);
        }

        std::unique_ptr<ASTNode> ParseRE(LexStream &stream)
        {
            return ParseAlternation(stream);
        }

        std::unique_ptr<ASTNode> Parse(const std::string &re)
        {
            LexStream stream(re);
            return ParseRE(stream);
        }
    } // namespace parser
} // namespace regex
