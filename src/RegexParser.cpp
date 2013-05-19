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

        std::unique_ptr<ASTNode> Parse(const std::string &re)
        {
            return std::unique_ptr<ASTNode>();
        }
    } // namespace parser
} // namespace regex
