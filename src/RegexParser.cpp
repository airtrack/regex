#include "RegexParser.h"
#include "RegexException.h"
#include <ctype.h>
#include <stdlib.h>
#include <limits>

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
        ACCEPT_VISITOR_IMPL(RepeatNode)

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

        void CheckRange(int first, int last, std::size_t index)
        {
            bool ok = false;
            if (first >= 'a' && first <= 'z' &&
                last >= 'a' && last <= 'z' &&
                first <= last)
                ok = true;
            if (first >= 'A' && first <= 'Z' &&
                last >= 'A' && last <= 'Z' &&
                first <= last)
                ok = true;
            if (first >= '0' && first <= '9' &&
                last >= '0' && last <= '9' &&
                first <= last)
                ok = true;

            if (!ok)
            {
                std::string desc("invalid range ");
                desc.push_back(first);
                desc.push_back('-');
                desc.push_back(last);
                throw ParseException(desc, index);
            }
        }

        int ParseNumber(LexStream &stream)
        {
            if (!isdigit(stream.Get()))
                throw ParseException("expect digit", stream.Index());

            std::string number;
            number.push_back(stream.Get());
            stream.Next();

            while (isdigit(stream.Get()))
            {
                number.push_back(stream.Get());
                stream.Next();
            }

            return atoi(number.c_str());
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
            stream.Next();              // Skip '['

            std::unique_ptr<CharRangeNode> char_range(new CharRangeNode);

            // If range head is ']' or '-', then add it to char_range
            if (stream.Get() == ']' || stream.Get() == '-')
            {
                char_range->AddChar(stream.Get());
                stream.Next();
            }

            while (stream.Get() != EOF && stream.Get() != ']')
            {
                int f = stream.Get();
                stream.Next();
                if (f == '-')
                {
                    // If range tail is '-', then add it to char_range
                    if (stream.Get() == ']')
                        char_range->AddChar(f);
                }
                else
                {
                    int m = stream.Get();
                    if (m == '-')
                    {
                        stream.Next();
                        if (stream.Get() == ']' || stream.Get() == EOF)
                        {
                            // '-' is range tail
                            char_range->AddChar(f);
                            char_range->AddChar(m);
                        }
                        else
                        {
                            // Check range 'x-y', and add range 'x-y' to char_range
                            CheckRange(f, stream.Get(), stream.Index());
                            char_range->AddRange(f, stream.Get());
                            stream.Next();
                        }
                    }
                    else
                    {
                        // Add normal character
                        char_range->AddChar(f);
                    }
                }
            }

            if (stream.Get() == EOF)
                throw ParseException("incomplete []", stream.Index());
            else
                stream.Next();

            return std::move(char_range);
        }

        std::unique_ptr<ASTNode> ParseParentheseRE(LexStream &stream)
        {
            stream.Next();              // Skip '('
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

        std::unique_ptr<ASTNode> ParseRepeat(LexStream &stream, std::unique_ptr<ASTNode> node)
        {
            stream.Next();              // Skip '{'

            int min = ParseNumber(stream);

            if (stream.Get() != ',')
                throw ParseException("expect ,", stream.Index());
            stream.Next();              // Skip ','

            int max = std::numeric_limits<int>::max();
            if (stream.Get() != '}')
                max = ParseNumber(stream);

            if (stream.Get() != '}')
                throw ParseException("expect }", stream.Index());
            stream.Next();              // Skip '}'

            return std::unique_ptr<RepeatNode>(new RepeatNode(std::move(node), min, max, true));
        }

        std::unique_ptr<ASTNode> ParseConcatBase(LexStream &stream)
        {
            std::unique_ptr<ASTNode> node = ParseBase(stream);

            if (stream.Get() == '*')
            {
                stream.Next();
                node = std::unique_ptr<ASTNode>(new ClosureNode(std::move(node)));
            }
            else if (stream.Get() == '{')
            {
                node = ParseRepeat(stream, std::move(node));
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
