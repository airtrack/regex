#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <assert.h>
#include <time.h>
#include <string>
#include <vector>

class timer
{
public:
    timer() : start_(0) { }
    timer(const timer &) = delete;
    void operator = (const timer &) = delete;

    void start()
    {
        start_ = clock();
    }

    double elapsed_milliseconds() const
    {
        return static_cast<double>(clock() - start_)
            / CLOCKS_PER_SEC * 1000;
    }

private:
    clock_t start_;
};

class benchmark_base
{
public:
    benchmark_base(const std::string &name,
                   const std::string &desc)
        : name_(name), desc_(desc)
    {
    }

    benchmark_base(const benchmark_base &) = delete;
    void operator = (const benchmark_base &) = delete;

    const std::string& get_desc() const
    {
        return desc_;
    }

    const std::string& get_name() const
    {
        return name_;
    }

    virtual void benchmark() = 0;

private:
    std::string name_;
    std::string desc_;
};

class benchmarks
{
public:
    benchmarks() = default;
    benchmarks(const benchmarks &) = delete;
    void operator = (const benchmarks &) = delete;

    void add_benchmark(benchmark_base *b)
    {
        benchmarks_.push_back(b);
    }

    void run_benchmarks()
    {
        for (auto b : benchmarks_)
        {
            timer t;
            t.start();
            b->benchmark();

            auto elapsed = t.elapsed_milliseconds();
            auto &name = b->get_name();
            auto &desc = b->get_desc();

            if (desc.empty())
                printf("[%s]: %.2f milliseconds.\n",
                       name.c_str(), elapsed);
            else
                printf("[%s](%s): %.2f milliseconds.\n",
                       name.c_str(), desc.c_str(), elapsed);
        }
    }

private:
    std::vector<benchmark_base *> benchmarks_;
};

extern benchmarks g_benchmarks;

#define BENCHMARK(name, desc)                               \
    struct benchmark_##name : public benchmark_base         \
    {                                                       \
        benchmark_##name() : benchmark_base(#name, #desc)   \
        {                                                   \
            g_benchmarks.add_benchmark(this);               \
        }                                                   \
                                                            \
        virtual void benchmark() override;                  \
    } benchmark_##name##_obj;                               \
                                                            \
    void benchmark_##name::benchmark()

#endif // BENCHMARK_H
