#ifndef REGEX_MATCHER_H
#define REGEX_MATCHER_H

#include "Regex.h"
#include <vector>

namespace regex
{
    // One match result for RegexMatcher class.
    struct Match
    {
        const char *begin_;
        const char *end_;

        explicit Match(const char *begin = nullptr,
                       const char *end = nullptr)
            : begin_(begin), end_(end)
        {
        }

        std::string GetString() const
        {
            return std::string(begin_, end_);
        }
    };

    // Match results for RegexMatcher class.
    class MatchResults
    {
        friend class RegexMatcher;
    public:
        typedef std::vector<Match> Matchs;
        typedef Matchs::iterator Iterator;
        typedef Matchs::const_iterator ConstIterator;

        Iterator Begin()
        {
            return matchs_.begin();
        }

        Iterator End()
        {
            return matchs_.end();
        }

        ConstIterator Begin() const
        {
            return matchs_.begin();
        }

        ConstIterator End() const
        {
            return matchs_.end();
        }

    private:
        void AddMatch(const char *begin, const char *end)
        {
            matchs_.push_back(Match(begin, end));
        }

        void AddMatch(const Match &match)
        {
            matchs_.push_back(match);
        }

        Matchs matchs_;
    };

    class RegexMatcher
    {
    public:
        explicit RegexMatcher(const Regex &regex);

        RegexMatcher(const RegexMatcher &) = delete;
        void operator = (const RegexMatcher &) = delete;

        // Check [begin, end) characters is match regex_ or not
        bool IsMatch(const char *begin, const char *end) const;

        // Search all matchs from [begin, end), all matchs
        // store in match_results
        void Search(const char *begin, const char *end,
                    MatchResults &match_results);

    private:
        // Find first match of regex_ from [current_, end_),
        // if exist, then return true.
        bool MatchOne(Match &match);
        bool CheckLastAccept(Match &match);

        const Regex &regex_;
        const char *current_;
        const char *end_;
        const char *start_;
        const char *last_accept_;
    };
} // namespace regex

#endif // REGEX_MATCHER_H
