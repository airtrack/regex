#include "RegexParser.h"
#include "RegexException.h"
#include <iostream>
#include <assert.h>

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        std::cout << "usage: " << argv[0] << " regex_str" << std::endl;
        return 0;
    }

    try
    {
        auto ast = regex::parser::Parse(argv[1]);
        assert(ast);
        std::cout << "regex \"" << argv[1] << "\" is ok." << std::endl;
    } catch (const regex::ParseException &e)
    {
        std::cout << "Parse error at index " << e.Position() << " :" << e.What() << std::endl;
    }

    return 0;
}
