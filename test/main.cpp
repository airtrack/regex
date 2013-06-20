#include "Regex.h"
#include "RegexMatcher.h"
#include "RegexException.h"
#include <iostream>
#include <assert.h>
#include <string.h>

int main(int argc, const char **argv)
{
    if (argc < 3)
    {
        std::cout << "usage: " << argv[0] << " regex_str match_str" << std::endl;
        return 0;
    }

    try
    {
        regex::Regex re(argv[1]);
        std::cout << "regex \"" << argv[1] << "\" is ok." << std::endl;

        regex::RegexMatcher matcher(re);
        std::size_t len = strlen(argv[2]);

        regex::MatchResults results;
        matcher.Search(argv[2], argv[2] + len, results);

        std::cout << "search all results:" << std::endl;
        for (auto it = results.Begin(); it != results.End(); ++it)
        {
            std::cout << it->GetString() << " ";
        }

        std::cout << std::endl;
    } catch (const regex::ParseException &e)
    {
        std::cout << "Parse error at index " << e.Position() << " :" << e.What() << std::endl;
    }

    return 0;
}
