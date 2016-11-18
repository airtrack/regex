#include "regex.h"

// static
const int regex::state_of_begin_;

// static regex::char_classes_
regex::char_classes regex::char_classes_ =
{
    { "alnum", { char_range('A', 'Z'), char_range('a', 'z'), char_range('0', '9') } },
    { "alpha", { char_range('A', 'Z'), char_range('a', 'z') } },
    { "blank", { char_range(' '), char_range('\t') } },
    { "cntrl", { char_range(0x0, 0x1F), char_range(0x7F) } },
    { "digit", { char_range('0', '9') } },
    { "graph", { char_range(0x21, 0x7E) } },
    { "lower", { char_range('a', 'z') } },
    { "print", { char_range(0x20, 0x7E) } },
    { "punct", { char_range(']'), char_range('['), char_range('!'),
                 char_range('"'), char_range('#'), char_range('$'),
                 char_range('%'), char_range('&'), char_range('\''),
                 char_range('('), char_range(')'), char_range('*'),
                 char_range('+'), char_range(','), char_range('.'),
                 char_range('/'), char_range(':'), char_range(';'),
                 char_range('<'), char_range('='), char_range('>'),
                 char_range('?'), char_range('@'), char_range('\\'),
                 char_range('^'), char_range('_'), char_range('`'),
                 char_range('{'), char_range('|'), char_range('}'),
                 char_range('~'), char_range('-') } },
    { "space", { char_range(' '), char_range('\t'), char_range('\r'),
                 char_range('\n'), char_range('\v'), char_range('\f') } },
    { "upper", { char_range('A', 'Z') } },
    { "word", { char_range('A', 'Z'), char_range('a', 'z'), char_range('0', '9'), char_range('_') } },
    { "xdigit", { char_range('A', 'F'), char_range('a', 'f'), char_range('0', '9') } }
};

