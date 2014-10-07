#include "regex.h"
#include "benchmark.h"
#include <stdio.h>
#include <regex>
#include <stdexcept>
#include <vector>
#include <memory>

class data
{
public:
    data()
    {
        auto file = fopen(BENCHMARK_TEST_FILE, "rb");
        if (!file)
            throw std::runtime_error("can not open file " + std::string(BENCHMARK_TEST_FILE) + ".");

        fseek(file, 0, SEEK_END);
        auto size = ftell(file);
        if (size == 0)
        {
            fclose(file);
            throw std::runtime_error(std::string(BENCHMARK_TEST_FILE) + " is empty file.");
        }

        fseek(file, 0, SEEK_SET);
        buffer_.resize(size);

        fread(&buffer_[0], 1, size, file);
        fclose(file);
    }

    data(const data &) = delete;
    void operator = (const data &) = delete;

    const char *begin() const
    {
        return &buffer_[0];
    }

    const char *end() const
    {
        return begin() + buffer_.size();
    }

private:
    std::vector<char> buffer_;
};

std::unique_ptr<data> g_data;

void std_regex_search_all(const std::string &re, const char *begin, const char *end)
{
    std::regex r(re);
    std::cmatch match;
    std::vector<std::cmatch> results;

    if (std::regex_search(begin, end, match, r))
    {
        results.push_back(match);
        while (std::regex_search(match[0].second, end, match, r))
            results.push_back(match);
    }
}

using namespace skl;

BENCHMARK(load, load mtent12.txt)
{
    g_data.reset(new data);
}

BENCHMARK(regex1, Twain)
{
    regex_search_all("Twain", g_data->begin(), g_data->end());
}

BENCHMARK(regex2, ^Twain)
{
    regex_search_all("^Twain", g_data->begin(), g_data->end());
}

BENCHMARK(regex3, Twain$)
{
    regex_search_all("Twain$", g_data->begin(), g_data->end());
}

BENCHMARK(regex4, Huck[a-zA-Z]+|Finn[a-zA-Z]+)
{
    regex_search_all("Huck[a-zA-Z]+|Finn[a-zA-Z]+", g_data->begin(), g_data->end());
}

BENCHMARK(regex5, a[^x]{20}b)
{
    regex_search_all("a[^x]{20}b", g_data->begin(), g_data->end());
}

BENCHMARK(regex6, Tom|Sawyer|Huckleberry|Finn)
{
    regex_search_all("Tom|Sawyer|Huckleberry|Finn", g_data->begin(), g_data->end());
}

BENCHMARK(regex7, (.{0,3}(Tom|Sawyer|Huckleberry|Finn)))
{
    regex_search_all(".{0,3}(Tom|Sawyer|Huckleberry|Finn)", g_data->begin(), g_data->end());
}

BENCHMARK(regex8, [a-zA-Z]+ing)
{
    regex_search_all("[a-zA-Z]+ing", g_data->begin(), g_data->end());
}

BENCHMARK(regex9, (^[a-zA-Z]{0,4}ing[^a-zA-Z]))
{
    regex_search_all("^[a-zA-Z]{0,4}ing[^a-zA-Z]", g_data->begin(), g_data->end());
}

BENCHMARK(regex10, [a-zA-Z]+ing$)
{
    regex_search_all("[a-zA-Z]+ing$", g_data->begin(), g_data->end());
}

BENCHMARK(regex11, (^[a-zA-Z ]{5,}$))
{
    regex_search_all("^[a-zA-Z ]{5,}$", g_data->begin(), g_data->end());
}

BENCHMARK(regex12, (^.{16,20}$))
{
    regex_search_all("^.{16,20}$", g_data->begin(), g_data->end());
}

BENCHMARK(regex13, ([a-f](.[d-m].){0,2}[h-n]){2})
{
    regex_search_all("([a-f](.[d-m].){0,2}[h-n]){2}", g_data->begin(), g_data->end());
}

BENCHMARK(regex14, ([A-Za-z]awyer|[A-Za-z]inn)[^a-zA-Z])
{
    regex_search_all("([A-Za-z]awyer|[A-Za-z]inn)[^a-zA-Z]", g_data->begin(), g_data->end());
}

BENCHMARK(regex15, (""[^""]{0,30}[?!\\.]""))
{
    regex_search_all("\"[^\"]{0,30}[?!\\.]\"", g_data->begin(), g_data->end());
}

BENCHMARK(regex16, (Tom.{10,25}river|river.{10,25}Tom))
{
    regex_search_all("Tom.{10,25}river|river.{10,25}Tom", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex1, Twain)
{
    std_regex_search_all("Twain", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex2, ^Twain)
{
    std_regex_search_all("^Twain", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex3, Twain$)
{
    std_regex_search_all("Twain$", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex4, Huck[a-zA-Z]+|Finn[a-zA-Z]+)
{
    std_regex_search_all("Huck[a-zA-Z]+|Finn[a-zA-Z]+", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex5, a[^x]{20}b)
{
    std_regex_search_all("a[^x]{20}b", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex6, Tom|Sawyer|Huckleberry|Finn)
{
    std_regex_search_all("Tom|Sawyer|Huckleberry|Finn", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex7, (.{0,3}(Tom|Sawyer|Huckleberry|Finn)))
{
    std_regex_search_all(".{0,3}(Tom|Sawyer|Huckleberry|Finn)", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex8, [a-zA-Z]+ing)
{
    std_regex_search_all("[a-zA-Z]+ing", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex9, (^[a-zA-Z]{0,4}ing[^a-zA-Z]))
{
    std_regex_search_all("^[a-zA-Z]{0,4}ing[^a-zA-Z]", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex10, [a-zA-Z]+ing$)
{
    std_regex_search_all("[a-zA-Z]+ing$", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex11, (^[a-zA-Z ]{5,}$))
{
    std_regex_search_all("^[a-zA-Z ]{5,}$", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex12, (^.{16,20}$))
{
    std_regex_search_all("^.{16,20}$", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex13, ([a-f](.[d-m].){0,2}[h-n]){2})
{
    std_regex_search_all("([a-f](.[d-m].){0,2}[h-n]){2}", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex14, ([A-Za-z]awyer|[A-Za-z]inn)[^a-zA-Z])
{
    std_regex_search_all("([A-Za-z]awyer|[A-Za-z]inn)[^a-zA-Z]", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex15, (""[^""]{0,30}[?!\\.]""))
{
    std_regex_search_all("\"[^\"]{0,30}[?!\\.]\"", g_data->begin(), g_data->end());
}

BENCHMARK(std_regex16, (Tom.{10,25}river|river.{10,25}Tom))
{
    std_regex_search_all("Tom.{10,25}river|river.{10,25}Tom", g_data->begin(), g_data->end());
}
