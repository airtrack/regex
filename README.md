Regex
=====

NFA-based regular expression.

Build
=====
Build with clang 3.4.

	cmake -G "Unix Makefiles" && make

Syntax
======

The following syntax features have been supported.

Metacharacter|Description
-------------|-----------
`.`|Matches any single character exclude `\n`.
`|`|Alternation, `abc|def` matches `abc` or `def`.
`[ ]`|A bracket expression. Matches a single character that is contained within the brackets. For example, `[abcx-z]` matches `a b c x y z`.
`[^ ]`|Matches a single character that is not contained within the brackets. For example, `[^abc]` matches any character other than `a b c`.
`^`|Matches the beginning of a line or string.
`$`|Matches the end of a line or string.
`( )`|Group a series of pattern elements and capture the match result.
`(?:)`|Group a series of pattern elements, no capture.
`?`|Matches the preceding pattern element zero or one times.
`*`|Matches the preceding pattern element zero or more times.
`+`|Matches the preceding pattern element one or more times.
`{m,n}`|Matches the preceding pattern element at least `m` and not more than `n` times. `{3,5}` matches 3 to 5 times, `{3,}` matches 3 or more times, `{3}` matches 3 times.
`(?=)`|Group a series of pattern elements as prediction, matches not within the match result.
`(?<=)`|Group a series of pattern elements as preceding prediction, matches not within the math result.
`\b`|Matches a word boundary.
`\B`|Matches boundary except word boundary.
`\d`|Matches `[0-9]`.
`\D`|Matches `[^0-9]`.
`\f`|Matches `\f`.
`\n`|Matches `\n`.
`\r`|Matches `\r`.
`\s`|Matches `[\f\n\r\t\v]`.
`\S`|Matches `[^\f\n\r\t\v]`.
`\t`|Matches `\t`.
`\v`|Matches `\v`.
`\w`|Matches `[A-Za-z0-9_]`.
`\W`|Matches `[^A-Za-z0-9_]`.

Character classes|ASCII|Description
-----------------|-----|-----------
`[:alnum:]`|`[A-Z a-z 0-9]`|Alphanumeric characters
`[:alpha:]`|`[A-Z a-z]`|Alphabetic characters
`[:blank:]`|`[ \t]`|Space and tab
`[:cntrl:]`|`[\x00-\x1F\x7F]`|Control characters
`[:digit:]`|`[0-9]`|Digits
`[:graph:]`|`[\x21-\x7E]`|Visible characters
`[:lower:]`|`[a-z]`|Lowercase letters
`[:print:]`|`[\x20-\x7E]`|Visible characters and the space character
`[:punct:]`|``[][!"#$%&'()*+,./:;<=>?@\^_`{\|}~-]``|Punctuation characters
`[:space:]`|`[ \t\r\n\v\f]`|Whitespace characters
`[:upper:]`|`[A-Z]`|Uppercase letters
`[:word:]`|`[A-Za-z0-9_]`|Alphanumeric characters plus "_"
`[:xdigit:]`|`[A-Fa-f0-9]`|Hexadecimal digits
