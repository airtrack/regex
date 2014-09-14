#include "UnitTest.h"
#include "regex.h"

using namespace skl;

TEST_CASE(regex1)
{
    EXPECT_TRUE(regex_match("", ""));
    EXPECT_TRUE(regex_match("abc", "abc"));
    EXPECT_TRUE(regex_match("(abc)", "abc"));
}

TEST_CASE(regex2)
{
    EXPECT_TRUE(regex_match("a|b|c", "a"));
    EXPECT_TRUE(regex_match("a|b|c", "b"));
    EXPECT_TRUE(regex_match("a|b|c", "c"));
    EXPECT_TRUE(regex_match("(a|b|c)", "a"));
    EXPECT_TRUE(regex_match("(a|b|c)", "b"));
    EXPECT_TRUE(regex_match("(a|b|c)", "c"));
}

TEST_CASE(regex3)
{
    EXPECT_TRUE(regex_match(".", "a"));
    EXPECT_TRUE(regex_match("(.)", "b"));
    EXPECT_TRUE(!regex_match(".", "\n"));
}

TEST_CASE(regex4)
{
    EXPECT_TRUE(regex_match(".*", ""));
    EXPECT_TRUE(regex_match(".*", "abc"));
    EXPECT_TRUE(regex_match("(.*)", "abc"));
    EXPECT_TRUE(regex_match("(.*)*", "abc"));

    EXPECT_TRUE(regex_match("(ab|bc|cd)*", ""));
    EXPECT_TRUE(regex_match("(ab|bc|cd)*", "abbccd"));

    EXPECT_TRUE(regex_match("(a|b|c)*b", "b"));
    EXPECT_TRUE(regex_match("(a|b|c)*b", "bb"));

    EXPECT_TRUE(regex_match("(a|b|c)*b*", ""));
    EXPECT_TRUE(regex_match("(a|b|c)*b*", "b"));
    EXPECT_TRUE(regex_match("(a|b|c)*b*", "bbb"));
    EXPECT_TRUE(regex_match("(a|b|c)*b*", "bbbabc"));

    EXPECT_TRUE(regex_match("(a*)*", ""));
    EXPECT_TRUE(regex_match("(a*)*", "a"));
    EXPECT_TRUE(regex_match("(a*)*", "aa"));

    EXPECT_TRUE(regex_match("a*a*", ""));
    EXPECT_TRUE(regex_match("a*a*", "a"));
    EXPECT_TRUE(regex_match("a*a*", "aa"));
    EXPECT_TRUE(regex_match("a*a*", "aaa"));

    EXPECT_TRUE(regex_match("((ab*|ac*)*|ad*)*", ""));
    EXPECT_TRUE(regex_match("((ab*|ac*)*|ad*)*", "a"));
    EXPECT_TRUE(regex_match("((ab*|ac*)*|ad*|ef)*", "abbacadaaefa"));
}

TEST_CASE(regex5)
{
    EXPECT_TRUE(regex_match("a+", "a"));
    EXPECT_TRUE(regex_match("(a+)", "a"));
    EXPECT_TRUE(regex_match("(a+)*", "a"));
    EXPECT_TRUE(regex_match("(a+)+", "a"));
    EXPECT_TRUE(regex_match("(a+)+", "aa"));
    EXPECT_TRUE(regex_match("(a+)+|b+", "b"));
    EXPECT_TRUE(!regex_match("(a+)+|b+", "ba"));
}

TEST_CASE(regex6)
{
    EXPECT_TRUE(regex_match("a?", ""));
    EXPECT_TRUE(regex_match("a?", "a"));
    EXPECT_TRUE(!regex_match("a?", "aa"));

    EXPECT_TRUE(regex_match("(a?)", "a"));
    EXPECT_TRUE(regex_match("(a?)*", "a"));
    EXPECT_TRUE(regex_match("(a?)+", "a"));

    EXPECT_TRUE(regex_match("(ab?c|b)*", "ac"));
    EXPECT_TRUE(regex_match("(ab?c|b)*", "acbbbabcb"));

    EXPECT_TRUE(regex_match("(a?)*a*", "aaaaaaaaaaaaaaaaaaaaaaaaaaaaa"));
}

TEST_CASE(regex7)
{
    EXPECT_TRUE(regex_match("[]", "[]"));
    EXPECT_TRUE(regex_match("[]]", "]"));
    EXPECT_TRUE(!regex_match("[^]]", "]"));

    EXPECT_TRUE(regex_match("[-]", "-"));
    EXPECT_TRUE(regex_match("[abc-]", "-"));
    EXPECT_TRUE(regex_match("[-abc]", "-"));

    EXPECT_TRUE(regex_match("[abc]*", "abc"));
    EXPECT_TRUE(regex_match("[a-ch-ij]*", "abchij"));

    EXPECT_TRUE(regex_match("[^a-ch-ij]*", "xyz"));
    EXPECT_TRUE(!regex_match("[^a-ch-ij]*", "xyzb"));

    EXPECT_TRUE(regex_match("[[:alnum:]]*", "Az9"));
    EXPECT_TRUE(regex_match("[[:alpha:]]*", "Az"));
    EXPECT_TRUE(regex_match("[[:blank:]]*", " \t"));
    EXPECT_TRUE(regex_match("[[:cntrl:]]*", "\x01\x02\x7F"));
    EXPECT_TRUE(regex_match("[[:digit:]]*", "123"));
    EXPECT_TRUE(regex_match("[[:graph:]]*", "\x21\x22\x7E"));
    EXPECT_TRUE(regex_match("[[:lower:]]*", "abcz"));
    EXPECT_TRUE(regex_match("[[:print:]]*", "\x20\x21\x7E"));
    EXPECT_TRUE(regex_match("[[:punct:]]*", "][!\"#$%&'()*+,./:;<=>?@\\^_`{|}~-"));
    EXPECT_TRUE(regex_match("[[:space:]]*", " \t\r\n\v\f"));
    EXPECT_TRUE(regex_match("[[:upper:]]*", "ABCZ"));
    EXPECT_TRUE(regex_match("[[:word:]]*", "_abzABZ09_"));
    EXPECT_TRUE(regex_match("[[:xdigit:]]*", "0123AFaf"));
}

TEST_CASE(regex8)
{
    EXPECT_TRUE(regex_match("a{", "a{"));

    EXPECT_TRUE(regex_match("a{}", ""));
    EXPECT_TRUE(!regex_match("a{}", "a"));

    EXPECT_TRUE(regex_match("a{1}", "a"));
    EXPECT_TRUE(!regex_match("a{1}", "aa"));

    EXPECT_TRUE(regex_match("a{1,}", "a"));
    EXPECT_TRUE(regex_match("a{1,}", "aa"));

    EXPECT_TRUE(regex_match("a{1,2}", "a"));
    EXPECT_TRUE(regex_match("a{1,2}", "aa"));
    EXPECT_TRUE(!regex_match("a{1,2}", "aaa"));

    EXPECT_TRUE(regex_match("(a{2}){3}", "aaaaaa"));
}

TEST_CASE(regex9)
{
    EXPECT_TRUE(regex_match("\\|", "|"));
    EXPECT_TRUE(regex_match("\\.", "."));
    EXPECT_TRUE(regex_match("\\*", "*"));
    EXPECT_TRUE(regex_match("\\?", "?"));
    EXPECT_TRUE(regex_match("\\+", "+"));
    EXPECT_TRUE(regex_match("\\^", "^"));
    EXPECT_TRUE(regex_match("\\$", "$"));
    EXPECT_TRUE(regex_match("\\(", "("));
    EXPECT_TRUE(regex_match("\\)", ")"));
    EXPECT_TRUE(regex_match("\\[", "["));
    EXPECT_TRUE(regex_match("\\]", "]"));
    EXPECT_TRUE(regex_match("\\{", "{"));
    EXPECT_TRUE(regex_match("\\}", "}"));
    EXPECT_TRUE(regex_match("\\\\", "\\"));
    EXPECT_TRUE(regex_match("\\f", "\f"));
    EXPECT_TRUE(regex_match("\\n", "\n"));
    EXPECT_TRUE(regex_match("\\r", "\r"));
    EXPECT_TRUE(regex_match("\\t", "\t"));
    EXPECT_TRUE(regex_match("\\v", "\v"));

    EXPECT_TRUE(regex_match("\\d*", "0123456789"));
    EXPECT_TRUE(!regex_match("\\D*", "0123456789"));

    EXPECT_TRUE(regex_match("\\s*", "\f\n\r\t\v"));
    EXPECT_TRUE(!regex_match("\\S*", "\f\n\r\t\v"));

    EXPECT_TRUE(regex_match("\\w*", "abcXYZ012_"));
    EXPECT_TRUE(!regex_match("\\W*", "abcXYZ012_"));

    EXPECT_TRUE(regex_match("abc\\z", "abcz"));
    EXPECT_EXCEPTION(regex_error, regex_match("abc\\", "abc"));
}

TEST_CASE(regex10)
{
    std::string str;
    match_result match_res;

    str = "abcdefabcdef";
    EXPECT_TRUE(regex_match("abc(.*)def", str, &match_res));
    EXPECT_TRUE(match_res[1].str() == "defabc");

    str = "defabcabc";
    EXPECT_TRUE(regex_match("abc(.*)def|def(.*)", str, &match_res));
    EXPECT_TRUE(!match_res[1].is_captured());
    EXPECT_TRUE(match_res[2].str() == "abcabc");

    str = "xyzabcabc";
    EXPECT_TRUE(regex_match("(.)(.)(.).*", str, &match_res));
    EXPECT_TRUE(match_res[1].str() == "x");
    EXPECT_TRUE(match_res[2].str() == "y");
    EXPECT_TRUE(match_res[3].str() == "z");

    str = "abcydef";
    EXPECT_TRUE(regex_match("abc(x|y|z)def", str, &match_res));
    EXPECT_TRUE(match_res[1].str() == "y");

    str = "abc0def";
    EXPECT_TRUE(regex_match("abc(.)def|xyz(.)|(.)def|hij", str, &match_res));
    EXPECT_TRUE(match_res[1].str() == "0");
    EXPECT_TRUE(!match_res[2].is_captured());
    EXPECT_TRUE(!match_res[3].is_captured());

    str = "xyz_";
    EXPECT_TRUE(regex_match("abc(.)def|xyz(.)|(.)def|hij", str, &match_res));
    EXPECT_TRUE(!match_res[1].is_captured());
    EXPECT_TRUE(match_res[2].str() == "_");
    EXPECT_TRUE(!match_res[3].is_captured());

    str = "xdef";
    EXPECT_TRUE(regex_match("abc(.)def|xyz(.)|(.)def|hij", str, &match_res));
    EXPECT_TRUE(!match_res[1].is_captured());
    EXPECT_TRUE(!match_res[2].is_captured());
    EXPECT_TRUE(match_res[3].str() == "x");

    str = "hij";
    EXPECT_TRUE(regex_match("abc(.)def|xyz(.)|(.)def|hij", str, &match_res));
    EXPECT_TRUE(!match_res[1].is_captured());
    EXPECT_TRUE(!match_res[2].is_captured());
    EXPECT_TRUE(!match_res[3].is_captured());

    str = "haj";
    EXPECT_TRUE(regex_match("abc(.)def|xyz(.)|(.)def|h(x|y|z|[abc])j|hij", str, &match_res));
    EXPECT_TRUE(!match_res[1].is_captured());
    EXPECT_TRUE(!match_res[2].is_captured());
    EXPECT_TRUE(!match_res[3].is_captured());
    EXPECT_TRUE(match_res[4].str() == "a");

    str = "aac";
    EXPECT_TRUE(regex_match(".*(aac)", str, &match_res));
    EXPECT_TRUE(match_res[0].str() == "aac");
    EXPECT_TRUE(match_res[1].str() == "aac");

    str = "2014-08-08 23:10";
    EXPECT_TRUE(regex_match("([0-9]+)-([0-9]+)-([0-9]+) ([0-9]+):([0-9]+)", str, &match_res));
    EXPECT_TRUE(match_res[1].str() == "2014");
    EXPECT_TRUE(match_res[2].str() == "08");
    EXPECT_TRUE(match_res[3].str() == "08");
    EXPECT_TRUE(match_res[4].str() == "23");
    EXPECT_TRUE(match_res[5].str() == "10");
}

TEST_CASE(regex11)
{
    EXPECT_TRUE(regex_match("^$", ""));
    EXPECT_TRUE(regex_match("^abc$", "abc"));
    EXPECT_TRUE(regex_match("^abc$\\n^abc$", "abc\nabc"));

    EXPECT_TRUE(regex_match(".*er\\b", "never"));
    EXPECT_TRUE(regex_match(".*er\\b ", "never "));
    EXPECT_TRUE(!regex_match(".*er\\b", "verb"));

    EXPECT_TRUE(regex_match(".*er\\Bb", "verb"));
    EXPECT_TRUE(!regex_match(".*er\\B", "never"));
}

TEST_CASE(regex12)
{
    std::string str;
    match_result match_res;

    str = "abcdefabc";
    EXPECT_TRUE(regex_match("abc(?:def|opq)abc", str, &match_res));
    EXPECT_TRUE(match_res.size() == 1);

    str = "abcopq";
    EXPECT_TRUE(regex_match("abc(?:def|opq)", str, &match_res));
    EXPECT_TRUE(match_res.size() == 1);
}

TEST_CASE(regex13)
{
    std::string str;
    match_result match_res;

    str = "abc123";
    EXPECT_TRUE(regex_match("abc(?=123|456)", str, &match_res));
    EXPECT_TRUE(match_res[0].str() == "abc");

    str = "abc456";
    EXPECT_TRUE(regex_match("abc(?=123|456)", str, &match_res));
    EXPECT_TRUE(match_res[0].str() == "abc");

    str = "abc121";
    EXPECT_TRUE(!regex_match("abc(?=123|456)", str, &match_res));

    str = "abc123456789";
    EXPECT_TRUE(regex_match("abc(?=\\d*)", str, &match_res));
    EXPECT_TRUE(match_res[0].str() == "abc");

    str = "abc123456789_";
    EXPECT_TRUE(!regex_match("abc(?=\\d*)", str, &match_res));
}

TEST_CASE(regex14)
{
    std::string str;
    match_result match_res;

    str = "123abc";
    EXPECT_TRUE(regex_match("(?<=123|456)abc", str, &match_res));
    EXPECT_TRUE(match_res[0].str() == "abc");

    str = "456abc";
    EXPECT_TRUE(regex_match("(?<=123|456)abc", str, &match_res));
    EXPECT_TRUE(match_res[0].str() == "abc");

    str = "455abc";
    EXPECT_TRUE(!regex_match("(?<=123|456)abc", str, &match_res));

    str = "123456789abc";
    EXPECT_TRUE(regex_match("(?<=\\d*)abc", str, &match_res));
    EXPECT_TRUE(match_res[0].str() == "abc");

    str = "123456789_abc";
    EXPECT_TRUE(!regex_match("(?<=\\d*)abc", str, &match_res));
}

TEST_CASE(regex15)
{
    match_result match_res;

    std::string str = "abcdefabcdgf";
    auto begin = str.c_str();
    auto end = begin + str.size();
    regex re1("d.f");

    EXPECT_TRUE(regex_search(re1, begin, end, &match_res));
    EXPECT_TRUE(match_res[0].str() == "def");
    EXPECT_TRUE(regex_search(re1, match_res.end_pos(), end, &match_res));
    EXPECT_TRUE(match_res[0].str() == "dgf");
    EXPECT_TRUE(!regex_search(re1, match_res.end_pos(), end, &match_res));

    str = "I watch three climb before it's my turn. It's a tough one.  The"
          " guy before me tries twice.   He falls twice.    After the last"
          " one, he comes down.     He's finished for the day.";

    begin = str.c_str();
    end = begin + str.size();
    regex re2("(?<=\\.) {1,}(?=[A-Z])");

    EXPECT_TRUE(regex_search(re2, begin, end, &match_res));
    EXPECT_TRUE(match_res[0].str() == " ");
    EXPECT_TRUE(regex_search(re2, match_res.end_pos(), end, &match_res));
    EXPECT_TRUE(match_res[0].str() == "  ");
    EXPECT_TRUE(regex_search(re2, match_res.end_pos(), end, &match_res));
    EXPECT_TRUE(match_res[0].str() == "   ");
    EXPECT_TRUE(regex_search(re2, match_res.end_pos(), end, &match_res));
    EXPECT_TRUE(match_res[0].str() == "    ");
    EXPECT_TRUE(regex_search(re2, match_res.end_pos(), end, &match_res));
    EXPECT_TRUE(match_res[0].str() == "     ");
    EXPECT_TRUE(!regex_search(re2, match_res.end_pos(), end, &match_res));

    begin = str.c_str();
    end = begin + str.size();
    regex re3(" +[Hh](.*)\\.");

    EXPECT_TRUE(regex_search(re3, begin, end, &match_res));
    EXPECT_TRUE(match_res[1].str() == "e falls twice");
    EXPECT_TRUE(regex_search(re3, match_res.end_pos(), end, &match_res));
    EXPECT_TRUE(match_res[1].str() == "e comes down");
    EXPECT_TRUE(regex_search(re3, match_res.end_pos(), end, &match_res));
    EXPECT_TRUE(match_res[1].str() == "e's finished for the day");
}
