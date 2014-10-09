#ifndef REGEX_H
#define REGEX_H

#include "bitmap.h"
#include "digraph.h"
#include <assert.h>
#include <ctype.h>
#include <stdlib.h>
#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <stack>
#include <string>
#include <stdexcept>
#include <vector>

namespace skl {

struct regex_error : public std::runtime_error
{
    using runtime_error::runtime_error;
};

enum state
{
    state_none = 0,
    state_branch,
    state_any,
    state_char,
    state_repeat,
    state_dot,
    state_match_range,
    state_exclude_range,
    state_capture_begin,
    state_capture_end,
    state_line_start,
    state_line_end,
    state_word_boundary,
    state_not_word_boundary,
    state_predict_begin,
    state_predict_end,
    state_reverse_predict_begin,
    state_reverse_predict_end,
};

struct state_data
{
    // Out states range [out_begin, out_end)
    int out_begin = 0;
    int out_end = 0;
    union
    {
        // Single char
        int c;
        // Repeat data for state_repeat
        struct
        {
            // Repeat begin state
            int repeat_begin;

            // Repeat counter index
            int index;

            // Repeat [min, max] times
            int min;
            int max;
        };
        // Range data for state_match_range
        // and state_exclude_range
        struct
        {
            // [char_range_begin, char_range_end) index
            int char_range_begin;
            int char_range_end;
        };
        // For state_capture_begin, state_capture_end,
        // state_predict_begin, state_reverse_predict_begin
        // The begin and the end pair states have the same
        // capture number, capture_num is the index of capture
        // data
        int capture_num;
    };
};

struct char_range
{
    // Character range [first, last]
    int first;
    int last;

    char_range(int f, int l)
        : first(f), last(l)
    {
    }

    explicit char_range(int c) : char_range(c, c) { }
};

struct states_data
{
    // Data of each state
    std::vector<state_data> data;
    // All character ranges
    std::vector<char_range> char_ranges;
    // Out states
    std::vector<int> out_states;
    // Count of state_repeat
    int repeat_count = 0;
    // Capture count
    int capture_count = 0;

    explicit states_data(std::size_t count)
        : data(count)
    {
    }
};

struct capture
{
    const char *begin;
    const char *end;

    explicit capture(const char *b = nullptr, const char *e = nullptr)
        : begin(b), end(e)
    {
    }

    inline bool is_captured() const
    {
        return begin && end;
    }

    std::string str() const
    {
        if (is_captured())
            return std::string(begin, end);
        else
            return std::string();
    }

    inline void reset()
    {
        begin = end = nullptr;
    }
};

// Match result of regex match or search
// index of subscript 0 is match result
// captures start from index of subscript 1
class match_result
{
    friend class regex;
public:
    typedef std::vector<capture> captures;
    typedef captures::const_iterator iterator;

    inline const char * start_pos() const
    {
        return captures_[0].begin;
    }

    inline const char * end_pos() const
    {
        return captures_[0].end;
    }

    inline std::size_t size() const
    {
        return captures_.size();
    }

    inline const capture& operator [] (std::size_t index) const
    {
        return captures_[index];
    }

    inline iterator begin() const
    {
        return captures_.begin();
    }

    inline iterator end() const
    {
        return captures_.end();
    }

private:
    // All capture data
    captures captures_;
};

// Thread for matching
struct thread
{
    // State index
    int state_index = 0;

    // Previous thread
    thread *prev = nullptr;
    // Next thread
    thread *next = nullptr;

    // For predict states
    capture predict;
    // For reverse predict states
    capture reverse_predict;

    const int repeat_num;
    const int capture_num;

    // Repeat times of all state_repeat
    int *repeat_times;
    // All captures' data
    capture *captures;

    // Reset thread data
    void reset()
    {
        state_index = 0;
        prev = nullptr;
        next = nullptr;
        predict.reset();
        reverse_predict.reset();

        for (int i = 0; i < repeat_num; ++i)
            repeat_times[i] = 0;
        for (int i = 0; i < capture_num; ++i)
            captures[i].reset();
    }

    // Inherit data from thread t except state_index
    void inherit_from(const thread *t)
    {
        predict = t->predict;
        reverse_predict = t->reverse_predict;

        for (int i = 0; i < repeat_num; ++i)
            repeat_times[i] = t->repeat_times[i];
        for (int i = 0; i < capture_num; ++i)
            captures[i] = t->captures[i];
    }

    // Split the thread from thread list
    void split()
    {
        if (prev)
            prev->next = next;
        if (next)
            next->prev = prev;
        prev = nullptr;
        next = nullptr;
    }

    thread()
        : repeat_num(0),
          capture_num(0),
          repeat_times(nullptr),
          captures(nullptr)
    {
    }

    thread(int repeat_count, int capture_count)
        : repeat_num(repeat_count),
          capture_num(capture_count),
          repeat_times(new int[repeat_count]()),
          captures(new capture[capture_count])
    {
    }

    thread(const thread &) = delete;
    void operator = (const thread &) = delete;

    ~thread()
    {
        delete [] repeat_times;
        delete [] captures;
    }
};

class thread_list
{
public:
    thread_list() { }
    thread_list(const thread_list &) = delete;
    void operator = (const thread_list &) = delete;

    ~thread_list()
    {
        auto t = head_->next;
        while (t)
        {
            auto tmp = t;
            t = t->next;
            delete tmp;
        }
    }

    // List is empty or not
    inline bool empty() const
    {
        return head_->next == nullptr;
    }

    // Return the first thread in the list
    inline thread * front() const
    {
        return head_->next;
    }

    // Push thread t into the front of list, then the list
    // own thread t
    inline void push_front(thread *t)
    {
        t->prev = head_;
        t->next = head_->next;
        if (head_->next)
            head_->next->prev = t;
        head_->next = t;
    }

    // Pop the first thread of list, then the list does not
    // own thread t yet
    inline thread * pop_front()
    {
        assert(head_->next);
        auto t = head_->next;
        t->split();
        return t;
    }

    // Slice all threads of thread_list tl into the front of
    // the list
    inline void slice(thread_list &tl)
    {
        if (tl.head_->next)
        {
            // Pointer to the last node of tl
            auto pt = tl.head_->next;
            while (pt->next) pt = pt->next;

            // The last node of tl pointer to the first node
            // of this
            pt->next = head_->next;
            if (head_->next)
                head_->next->prev = pt;

            // Pointer to the first node of tl
            head_->next = tl.head_->next;
            tl.head_->next->prev = head_;

            // Clear tl
            tl.head_->next = nullptr;
        }
    }

    // Swap with thread_list tl
    inline void swap(thread_list &tl)
    {
        std::swap(head_->next, tl.head_->next);
        if (head_->next)
            head_->next->prev = head_;
        if (tl.head_->next)
            tl.head_->next->prev = tl.head_;
    }

    // Create thread
    inline static thread * create(int repeat_count, int capture_count)
    {
        return new thread(repeat_count, capture_count);
    }

private:
    thread sentry_;
    thread *head_ = &sentry_;
};

// Context data for matching
struct context
{
    // Current match threads
    thread_list tlist;
    // Next match threads
    thread_list next_tlist;
    // Pending threads for epsilon_extend
    thread_list pending_tlist;
    // Freed thread list for reuse
    thread_list free_tlist;
    // Current running thread
    thread *current_thread = nullptr;

    // Number of repeat states
    const int repeat_count;
    // Number of capture states
    const int capture_count;

    // Pending states for epsilon_extend
    std::vector<std::pair<int, thread *>> pending_states;

    // Epsilon DFS data for epsilon_extend
    const digraph &epsilon;
    bitmap epsilon_bits;

    // Search range [sbegin, send)
    const char *sbegin;
    const char *send;
    // Current pos
    const char *scur = nullptr;

    // Has accepted or not
    bool accept = false;
    // Captures when accepted
    std::vector<capture> accept_captures;

    void reset(const char *begin, const char *end)
    {
        free_thread_list(tlist);
        free_thread_list(next_tlist);
        free_thread_list(pending_tlist);
        current_thread = nullptr;
        pending_states.clear();

        sbegin = begin;
        send = end;
        scur = nullptr;
        accept = false;
        accept_captures.clear();
    }

    // Alloc a thread
    inline thread * alloc_thread(bool reset = false)
    {
        if (!free_tlist.empty())
        {
            auto t = free_tlist.pop_front();
            if (reset) t->reset();
            return t;
        }

        return thread_list::create(repeat_count, capture_count);
    }

    // Free thread list
    inline void free_thread_list(thread_list &tl)
    {
        free_tlist.slice(tl);
    }

    // Clone thread t and set state index, then put it into pending_threads
    thread *clone_to_pending_threads(const thread *t, int state_index)
    {
        auto tn = alloc_thread(t == nullptr);
        pending_tlist.push_front(tn);
        tn->state_index = state_index;

        if (t) tn->inherit_from(t);
        return tn;
    }

    // DFS epsilon digraph, pending all states into pending_states
    void dfs_to_pending_states(int c, thread *t)
    {
        if (!epsilon_bits.is_set(c))
        {
            epsilon_bits.set(c);
            pending_states.emplace_back(c, t);

            auto it = epsilon.edge_begin(c);
            auto end = epsilon.edge_end(c);
            for (; it != end; ++it)
                dfs_to_pending_states(*it, t);
        }
    }

    context(const states_data &sd,
            const digraph &ep,
            const char *begin,
            const char *end)
        : repeat_count(sd.repeat_count),
          capture_count(sd.capture_count),
          epsilon(ep), epsilon_bits(ep.vertexes()),
          sbegin(begin), send(end)
    {
    }

    context(const context &) = delete;
    void operator = (const context &) = delete;
};

class regex
{
    typedef std::map<std::string, std::vector<char_range>> char_classes;
public:
    explicit regex(const std::string &re)
        : re_(padding_ + re),
          states_(re_.size() + 1, state_none),
          states_data_(re_.size() + 1),
          epsilon_(re_.size() + 1),
          accept_state_(re_.size())
    {
        prepare_padding_states();
        construct_states();
    }

    regex(const regex&) = delete;
    regex& operator = (const regex&) = delete;

    const std::string& pattern() const
    {
        return re_;
    }

    bool match(const std::string &str, match_result *match_res = nullptr) const
    {
        auto begin = str.c_str();
        auto end = begin + str.size();
        return match(begin, end, match_res);
    }

    bool match(const char *begin, const char *end, match_result *match_res = nullptr) const
    {
        return match_search(begin, end, match_res, false);
    }

    bool search(const char *begin, const char *end, match_result *match_res) const
    {
        return match_search(begin, end, match_res, true);
    }

private:
    bool match_search(const char *begin, const char *end,
            match_result *match_res, bool search) const
    {
        if (context_)
            context_->reset(begin, end);
        else
            context_.reset(new context(states_data_, epsilon_, begin, end));
        context &ctx = *context_;

        // Init threads
        if (search)
        {
            epsilon_extend(state_of_search_begin_0, ctx, ctx.tlist, nullptr);
            epsilon_extend(state_of_search_begin_1, ctx, ctx.tlist, nullptr);
        }
        else
        {
            epsilon_extend(state_of_match_begin_, ctx, ctx.tlist, nullptr);
        }

        // Iterate input
        for (ctx.scur = ctx.sbegin;
                search ? !ctx.accept && ctx.scur != ctx.send : ctx.scur != ctx.send;
                ++ctx.scur)
        {
            ctx.accept = false;

            for (auto t = ctx.tlist.front(); t;)
            {
                ctx.current_thread = t;
                auto c = t->state_index;
                // Move to next first, because move_to_next may split
                // the t from the tlist
                t = t->next;
                switch (states_[c])
                {
                    case state_any:
                        move_to_next(c, ctx);
                        break;
                    case state_char:
                        if (static_cast<unsigned char>(*ctx.scur) == states_data_.data[c].c)
                            move_to_next(c, ctx);
                        break;
                    case state_dot:
                        if (*ctx.scur != '\n')
                            move_to_next(c, ctx);
                        break;
                    case state_match_range:
                        if (match_range(*ctx.scur, c))
                            move_to_next(c, ctx);
                        break;
                    case state_exclude_range:
                        if (!match_range(*ctx.scur, c))
                            move_to_next(c, ctx);
                        break;
                    default:
                        break;
                }
            }

            ctx.tlist.swap(ctx.next_tlist);
            ctx.free_thread_list(ctx.next_tlist);
        }

        if (match_res)
        {
            if (ctx.accept)
                match_res->captures_.swap(ctx.accept_captures);
            else
                match_res->captures_.resize(ctx.capture_count);
        }

        return ctx.accept;
    }

    bool match_range(unsigned char c, int s) const
    {
        int begin = states_data_.data[s].char_range_begin;
        int end = states_data_.data[s].char_range_end;

        for (int i = begin; i < end; ++i)
        {
            const auto &range = states_data_.char_ranges[i];
            if (c >= range.first && c <= range.last)
                return true;
        }

        return false;
    }

    void move_to_next(int v, context &ctx) const
    {
        auto out_it = states_data_.data[v].out_begin;
        auto out_end = states_data_.data[v].out_end;

        while (out_it < out_end)
        {
            auto n = states_data_.out_states[out_it];
            auto tn = ctx.clone_to_pending_threads(ctx.current_thread, v);
            epsilon_extend(n, ctx, ctx.next_tlist, tn);
            ++out_it;
        }

        epsilon_extend(v + 1, ctx, ctx.next_tlist, ctx.current_thread);
    }

    void epsilon_extend(int v, context &ctx, thread_list &tlist, thread *from) const
    {
        auto it = ctx.pending_states.size();
        ctx.epsilon_bits.clear();
        ctx.dfs_to_pending_states(v, from);
        auto end = ctx.pending_states.size();

        while (it < end)
        {
            for (; it < end; ++it)
            {
                auto c = ctx.pending_states[it].first;
                auto t = ctx.pending_states[it].second;
                auto s = states_[c];
                switch (s)
                {
                    case state_none:
                        break;

                    case state_branch:
                    {
                        if (!t)
                        {
                            t = ctx.alloc_thread(true);
                            ctx.pending_tlist.push_front(t);
                        }
                        t->state_index = c;
                        branch_to_next(ctx, c, t);
                        break;
                    }

                    case state_repeat:
                    {
                        assert(t);
                        // Increase repeat times
                        auto index = states_data_.data[c].index;
                        auto times = t->repeat_times[index] + 1;
                        auto need_repeat = times < states_data_.data[c].max;
                        auto need_move_next = times >= states_data_.data[c].min &&
                            times <= states_data_.data[c].max;

                        // Less than max, repeat it again
                        if (need_repeat)
                        {
                            thread *tb = nullptr;
                            if (need_move_next)
                                tb = ctx.clone_to_pending_threads(t, c);
                            else
                            {
                                t->state_index = c;
                                tb = t;
                            }
                            tb->repeat_times[index] = times;

                            // Clear all repeat states counter in range [b, c)
                            auto b = states_data_.data[c].repeat_begin;
                            for (auto i = b; i < c; ++i)
                            {
                                if (states_[i] == state_repeat)
                                    tb->repeat_times[states_data_.data[i].index] = 0;
                            }

                            ctx.dfs_to_pending_states(b, tb);
                        }

                        // Move to next state when repeat times in range [min, max]
                        if (need_move_next)
                        {
                            t->state_index = c;
                            t->repeat_times[index] = times;
                            branch_to_next(ctx, c, t);
                        }
                        break;
                    }

                    case state_capture_begin:
                    case state_predict_begin:
                    case state_reverse_predict_begin:
                    {
                        if (!t)
                        {
                            t = ctx.alloc_thread(true);
                            ctx.pending_tlist.push_front(t);
                        }
                        t->state_index = c;
                        if (s == state_capture_begin)
                        {
                            // Update capture data
                            auto index = states_data_.data[c].capture_num;
                            t->captures[index].begin =
                                ctx.scur ? ctx.scur + 1 : ctx.sbegin;
                        }
                        else if (s == state_predict_begin)
                        {
                            // Update predict
                            t->predict.begin =
                                ctx.scur ? ctx.scur + 1 : ctx.sbegin;
                        }
                        else
                        {
                            // Update reverse predict
                            t->reverse_predict.begin =
                                ctx.scur ? ctx.scur + 1 : ctx.sbegin;
                        }

                        branch_to_next(ctx, c, t);
                        break;
                    }

                    case state_capture_end:
                    {
                        assert(t);
                        auto from_state = states_[t->state_index];
                        t->state_index = c;

                        // Update capture data
                        auto index = states_data_.data[c].capture_num;
                        if (from_state == state_capture_begin)
                            t->captures[index].end = t->captures[index].begin;
                        else
                            t->captures[index].end = ctx.scur + 1;

                        // The accept_state_ is last state_capture_end.
                        // If c is accept_state_, then do not move to
                        // next state
                        if (c != accept_state_)
                        {
                            branch_to_next(ctx, c, t);
                        }
                        else if (!ctx.accept)
                        {
                            ctx.accept = true;
                            ctx.accept_captures.assign(t->captures, t->captures + t->capture_num);
                            // Set predict begin to accept_captures[0].end
                            // when predict success
                            if (t->predict.begin)
                                ctx.accept_captures[0].end = t->predict.begin;
                            // Set reverse predict end to accept_captures[0].begin
                            // when reverse predict success
                            if (t->reverse_predict.end)
                                ctx.accept_captures[0].begin = t->reverse_predict.end;
                        }
                        break;
                    }

                    case state_predict_end:
                    {
                        assert(t);
                        auto from_state = states_[t->state_index];
                        t->state_index = c;

                        // Update predict data
                        if (from_state == state_predict_begin)
                            t->predict.end = t->predict.begin;
                        else
                            t->predict.end = ctx.scur + 1;

                        branch_to_next(ctx, c, t);
                        break;
                    }

                    case state_reverse_predict_end:
                    {
                        assert(t);
                        auto from_state = states_[t->state_index];
                        t->state_index = c;

                        // Update reverse predict data
                        if (from_state == state_reverse_predict_begin)
                            t->reverse_predict.end = t->reverse_predict.begin;
                        else
                            t->reverse_predict.end = ctx.scur + 1;

                        branch_to_next(ctx, c, t);
                        break;
                    }

                    case state_line_start:
                    {
                        if (!ctx.scur || *ctx.scur == '\n')
                            branch_to_next(ctx, c, t);
                        break;
                    }

                    case state_line_end:
                    {
                        if ((!ctx.scur && ctx.sbegin == ctx.send) ||
                            (ctx.scur && (ctx.scur + 1 == ctx.send || ctx.scur[1] == '\n')))
                            branch_to_next(ctx, c, t);
                        break;
                    }

                    case state_word_boundary:
                    {
                        auto pc = ctx.scur;
                        if (pc && (pc + 1 == ctx.send ||
                                   isspace(static_cast<unsigned char>(pc[1]))))
                            branch_to_next(ctx, c, t);
                        break;
                    }

                    case state_not_word_boundary:
                    {
                        auto pc = ctx.scur;
                        if (pc && !(pc + 1 == ctx.send ||
                                    isspace(static_cast<unsigned char>(pc[1]))))
                            branch_to_next(ctx, c, t);
                        break;
                    }

                    default:
                    {
                        if (t)
                            t->split();
                        else
                            t = ctx.alloc_thread(true);
                        t->state_index = c;
                        tlist.push_front(t);
                        break;
                    }
                }
            }

            end = ctx.pending_states.size();
        }

        ctx.pending_states.clear();
        ctx.free_thread_list(ctx.pending_tlist);
    }

    void branch_to_next(context &ctx, int c, thread *t) const
    {
        // Branch all out states
        ctx.dfs_to_pending_states(c + 1, t);
        for (auto i = states_data_.data[c].out_begin;
                i < states_data_.data[c].out_end; ++i)
        {
            auto tn = ctx.clone_to_pending_threads(t, c);
            ctx.dfs_to_pending_states(states_data_.out_states[i], tn);
        }
    }

    void prepare_padding_states()
    {
        // Add padding states
        // |-----|--------|---------------|-----|-------------|
        // | any | repeat | capture begin | ... | capture end |
        // |-----|--------|---------------|-----|-------------|
        states_[0] = state_any;
        add_repeat_state(1, 0, 0);
        add_capture_begin(2);
        add_capture_end(states_.size() - 1, 2);
    }

    void construct_states()
    {
        std::stack<int> meta;
        std::stack<int> lps;
        std::vector<std::pair<int, int>> out_edges;
        std::vector<std::pair<int, int>> repeat0_edges;

        int size = re_.size();
        int i = padding_size_;
        while (i < size)
        {
            int lp = i;
            switch (re_[i])
            {
                case '(':
                    i = add_left_parentheses(i, lps);
                    break;
                case ')':
                    add_right_parentheses(i, lp, lps, meta);
                    break;
                case '|':
                    meta.push(i);

                    // Add '|' start edge, from '(' or beginning
                    if (i < size - 1)
                    {
                        if (!lps.empty())
                            out_edges.push_back({ lps.top(), i + 1 });
                        else
                            out_edges.push_back({ 2, i + 1 });
                    }
                    break;
                case '.':
                    states_[i] = state_dot;
                    break;
                case '[':
                    i = add_range_state(i);
                    break;
                case '^':
                    states_[i] = state_line_start;
                    break;
                case '$':
                    states_[i] = state_line_end;
                    break;
                case '\\':
                    i = add_escape_char(i);
                    break;
                default:
                    add_char_state(i, static_cast<unsigned char>(re_[i]));
                    break;
            }

            if (i < size - 1)
                i = add_repeat_state(i, lp, out_edges, repeat0_edges);

            ++i;
        }

        if (!lps.empty())
            throw regex_error("parentheses () not blanced");

        // Process remain meta characters
        while (!meta.empty())
        {
            auto top = meta.top();
            if (re_[top] == '|')
                epsilon_.add_edge(top, i);
            meta.pop();
        }

        // Sort it, then all state_capture_begin, state_predict_begin,
        // state_reverse_predict_begin states grouped
        std::sort(out_edges.begin(), out_edges.end());

        // Prepare out states
        int s = -1;
        for (const auto &p : out_edges)
        {
            if (p.first != s)
            {
                if (s != -1)
                    states_data_.data[s].out_end = states_data_.out_states.size();
                s = p.first;
                states_data_.data[s].out_begin = states_data_.out_states.size();

                // If state_none has out edges, change the state to state_branch
                if (states_[s] == state_none)
                {
                    states_[s] = state_branch;
                    // state_branch will move to next, so remove the epsilon edge
                    epsilon_.remove_edge(s, s + 1);
                }
            }

            states_data_.out_states.push_back(p.second);
        }
        if (s != -1)
            states_data_.data[s].out_end = states_data_.out_states.size();
    }

    int add_left_parentheses(int i, std::stack<int> &lps)
    {
        int size = re_.size();
        if (i + 2 < size && re_[i + 1] == '?')
        {
            if (re_[i + 2] == ':')
            {
                lps.push(i);
                states_[i] = state_branch;
                // No epsilon edge from '(' to '?'
                epsilon_.add_edge(i + 1, i + 2);
                epsilon_.add_edge(i + 2, i + 3);
                return i + 2;
            }
            else if (re_[i + 2] == '=')
            {
                lps.push(i);
                states_[i] = state_predict_begin;
                // No epsilon edge from '(' to '?'
                epsilon_.add_edge(i + 1, i + 2);
                epsilon_.add_edge(i + 2, i + 3);
                return i + 2;
            }
            else if (i + 3 < size && re_[i + 2] == '<' && re_[i + 3] == '=')
            {
                lps.push(i);
                states_[i] = state_reverse_predict_begin;
                // No epsilon edge from '(' to '?'
                epsilon_.add_edge(i + 1, i + 2);
                epsilon_.add_edge(i + 2, i + 3);
                epsilon_.add_edge(i + 3, i + 4);
                return i + 3;
            }
        }

        add_capture_begin(i);
        lps.push(i);
        return i;
    }

    void add_right_parentheses(int i, int &lp,
            std::stack<int> &lps, std::stack<int> &meta)
    {
        // Process matched pair '('
        if (!lps.empty())
        {
            lp = lps.top();
            lps.pop();

            auto s = states_[lp];
            switch (s)
            {
                case state_capture_begin:
                    add_capture_end(i, lp);
                    break;

                case state_predict_begin:
                    states_[i] = state_predict_end;
                    break;

                case state_reverse_predict_begin:
                    states_[i] = state_reverse_predict_end;
                    break;

                default:
                    epsilon_.add_edge(i, i + 1);
                    break;
            }

            // Complete '|' edge, form '|' to ')'
            while (!meta.empty() && re_[meta.top()] == '|' && meta.top() > lp)
            {
                epsilon_.add_edge(meta.top(), i);
                meta.pop();
            }
        }
        else throw regex_error("parentheses () not blanced");
    }

    void add_repeat0_out_edges(int begin, int repeat_end,
            std::vector<std::pair<int, int>> &out_edges,
            std::vector<std::pair<int, int>> &repeat0_edges)
    {
        // Extend out edges when there have more than one zero-min repeat
        // states are adjacent. e.g.
        // "(.*.*.*)"
        //  ^  ^ ^ ^
        //  |  | | |
        //  0  3 5 7
        // state 0 out edges: 0->3 0->5 0->7
        // state 2 out edges: 2->5 2->7
        // state 4 out edges: 4->7
        auto from_edges = std::equal_range(repeat0_edges.begin(),
                repeat0_edges.end(), std::make_pair(0, begin),
                [](const std::pair<int, int> &l,
                   const std::pair<int, int> &r) {
                    return l.second < r.second;
                });

        auto it = out_edges.size();

        for (; from_edges.first != from_edges.second; ++from_edges.first)
            out_edges.push_back({ from_edges.first->first, repeat_end });
        out_edges.push_back({ begin - 1, repeat_end });

        auto end = out_edges.size();

        for (; it < end; ++it)
            repeat0_edges.push_back(out_edges[it]);
    }

    void add_repeat_state(int i, int begin,
            int min, int max = std::numeric_limits<int>::max())
    {
        states_[i] = state_repeat;
        states_data_.data[i].min = min;
        states_data_.data[i].max = max;
        states_data_.data[i].index = states_data_.repeat_count++;
        states_data_.data[i].repeat_begin = begin;
    }

    int add_repeat_state(int i, int begin,
            std::vector<std::pair<int, int>> &out_edges,
            std::vector<std::pair<int, int>> &repeat0_edges)
    {
        // If next char is '*', '+', '?', '{',
        // add repeat state and epsilon edges
        assert(begin > 0);
        if (re_[i + 1] == '*')
        {
            add_repeat_state(i + 1, begin, 0);
            add_repeat0_out_edges(begin, i + 2, out_edges, repeat0_edges);
            ++i;
        }
        else if (re_[i + 1] == '+')
        {
            add_repeat_state(i + 1, begin, 1);
            ++i;
        }
        else if (re_[i + 1] == '?')
        {
            add_repeat_state(i + 1, begin, 0, 1);
            add_repeat0_out_edges(begin, i + 2, out_edges, repeat0_edges);
            ++i;
        }
        else if (re_[i + 1] == '{')
        {
            int lb = i + 1;
            int rb = lb + 1;
            int size = re_.size();

            // Find '}'
            while (rb < size && re_[rb] != '}') ++rb;

            // Not find '}'
            if (rb >= size)
                return i;

            // Find ','
            int comma = lb + 1;
            while (comma < rb && re_[comma] != ',') ++comma;

            int min = atoi(re_.substr(lb + 1, comma - lb - 1).c_str());

            if (comma == rb)
            {
                // If ',' do not exist, such as "{1}"
                add_repeat_state(lb, begin, min, min);
            }
            else if (comma + 1 == rb)
            {
                // No character after ',', such as "{1,}"
                add_repeat_state(lb, begin, min);
            }
            else
            {
                // Max repeat value exist
                int max = atoi(re_.substr(comma + 1, rb - comma - 1).c_str());
                add_repeat_state(lb, begin, min, max);
            }

            // Add out edge, skip repeat state when min <= 0
            if (min <= 0)
                add_repeat0_out_edges(begin, rb + 1, out_edges, repeat0_edges);

            // Add epsilon edges, from lb + 1 to rb
            ++lb;
            while (lb <= rb) { epsilon_.add_edge(lb, lb + 1); ++lb; }

            i = rb;
        }

        return i;
    }

    int add_range_state(int i)
    {
        // Find ']' from i + 2, assume "[]" to be normal characters.
        int size = re_.size();
        int lb = i;
        int rb = i + 2;

        // Find ']'
        while (rb < size && re_[rb] != ']') ++rb;

        // Can not find ']'
        if (rb >= size)
        {
            add_char_state(i, static_cast<unsigned char>(re_[i]));
            return i;
        }

        state s = state_match_range;
        if (re_[++i] == '^')
        {
            s = state_exclude_range;
            ++i;
            // "[^]]" is legal
            if (i == rb)
            {
                // Find next ']'
                ++rb;
                while (rb < size && re_[rb] != ']') ++rb;
                if (rb == size) throw regex_error("brackets [ ] not balanced");
            }
        }
        states_[lb] = s;
        states_data_.data[lb].char_range_begin = states_data_.char_ranges.size();

        while (i < rb)
        {
            // Process character classes
            if (re_[i] == '[')
            {
                if (i + 1 < rb && re_[i + 1] == ':')
                {
                    int ii = i + 2;
                    auto it = parse_char_class(ii, rb);
                    if (it != char_classes_.end())
                    {
                        states_data_.char_ranges.insert(
                                states_data_.char_ranges.end(),
                                it->second.begin(), it->second.end());
                        i = ii + 1;
                        continue;
                    }
                }
            }

            int f = static_cast<unsigned char>(re_[i++]);
            int l = 0;

            // Char range
            if (i + 1 < rb && re_[i] == '-')
            {
                ++i;    // Skip '-'
                l = static_cast<unsigned char>(re_[i++]);
            }
            else
            {
                l = f;
            }

            states_data_.char_ranges.push_back(char_range(f, l));
        }
        states_data_.data[lb].char_range_end = states_data_.char_ranges.size();

        // Add epsilon edges, from lb + 1 to rb
        ++lb;
        while (lb <= rb) { epsilon_.add_edge(lb, lb + 1); ++lb; }
        return rb;
    }

    char_classes::const_iterator parse_char_class(int &i, int &rb) const
    {
        int e = i;
        // Find ":]"
        while (e < rb && re_[e] != ':') ++e;
        if (e == rb || re_[e + 1] != ']')
            return char_classes_.end();

        auto str = re_.substr(i, e - i);
        auto it = char_classes_.find(str);
        if (it == char_classes_.end())
            throw regex_error("invalid character class " + str);

        i = e + 1;
        // If re_[rb] is the right bracket of character class,
        // then search new ']' position as rb
        if (i >= rb)
        {
            int size = re_.size();
            ++rb;
            while (rb < size && re_[rb] != ']') ++rb;
            if (rb == size) throw regex_error("brackets [ ] not balanced");
        }

        return it;
    }

    int add_escape_char(int i)
    {
        int size = re_.size();
        if (i + 1 < size)
        {
            int c = static_cast<unsigned char>(re_[i + 1]);
            switch (c)
            {
                case 'f': add_char_state(i, '\f'); break;
                case 'n': add_char_state(i, '\n'); break;
                case 'r': add_char_state(i, '\r'); break;
                case 't': add_char_state(i, '\t'); break;
                case 'v': add_char_state(i, '\v'); break;
                case 's': add_spaces_state(i, state_match_range); break;
                case 'S': add_spaces_state(i, state_exclude_range); break;
                case 'w': add_word_state(i, state_match_range); break;
                case 'W': add_word_state(i, state_exclude_range); break;
                case 'd': add_digit_state(i, state_match_range); break;
                case 'D': add_digit_state(i, state_exclude_range); break;
                case 'b': states_[i] = state_word_boundary; break;
                case 'B': states_[i] = state_not_word_boundary; break;
                default: add_char_state(i, c); break;
            }

            ++i;
        }
        else throw regex_error("trailing backslash (\\)");

        epsilon_.add_edge(i, i + 1);
        return i;
    }

    void add_spaces_state(int i, state s)
    {
        states_[i] = s;
        states_data_.data[i].char_range_begin = states_data_.char_ranges.size();
        states_data_.char_ranges.push_back(char_range('\f'));
        states_data_.char_ranges.push_back(char_range('\n'));
        states_data_.char_ranges.push_back(char_range('\r'));
        states_data_.char_ranges.push_back(char_range('\t'));
        states_data_.char_ranges.push_back(char_range('\v'));
        states_data_.data[i].char_range_end = states_data_.char_ranges.size();
    }

    void add_word_state(int i, state s)
    {
        states_[i] = s;
        states_data_.data[i].char_range_begin = states_data_.char_ranges.size();
        states_data_.char_ranges.push_back(char_range('a', 'z'));
        states_data_.char_ranges.push_back(char_range('A', 'Z'));
        states_data_.char_ranges.push_back(char_range('0', '9'));
        states_data_.char_ranges.push_back(char_range('_'));
        states_data_.data[i].char_range_end = states_data_.char_ranges.size();
    }

    void add_digit_state(int i, state s)
    {
        states_[i] = s;
        states_data_.data[i].char_range_begin = states_data_.char_ranges.size();
        states_data_.char_ranges.push_back(char_range('0', '9'));
        states_data_.data[i].char_range_end = states_data_.char_ranges.size();
    }

    void add_char_state(int i, int c)
    {
        states_[i] = state_char;
        states_data_.data[i].c = c;
    }

    void add_capture_begin(int i)
    {
        states_[i] = state_capture_begin;
        states_data_.data[i].capture_num = states_data_.capture_count++;
    }

    void add_capture_end(int i, int lp)
    {
        states_[i] = state_capture_end;
        states_data_.data[i].capture_num = states_data_.data[lp].capture_num;
    }

    typedef std::vector<state> states;

    static char_classes char_classes_;

    // Padding size = 3
    static constexpr const char *padding_ = "   ";
    static const int padding_size_ = 3;
    static const int state_of_match_begin_ = 2;
    static const int state_of_search_begin_0 = 0;
    static const int state_of_search_begin_1 = 2;

    // Regex string
    std::string re_;
    // States, format:
    // |-----|--------|---------------|-----|-------------|
    // | any | repeat | capture begin | ... | capture end |
    // |-----|--------|---------------|-----|-------------|
    states states_;
    // Data of states
    states_data states_data_;
    // Epsilon digraph
    digraph epsilon_;
    // Accept state
    const int accept_state_;
    // Context data
    mutable std::unique_ptr<context> context_;
};

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

// Helper functions
inline bool regex_match(const regex &re,
        const std::string &str, match_result *match_res = nullptr)
{
    return re.match(str, match_res);
}

inline bool regex_match(const regex &re,
        const char *begin, const char *end, match_result *match_res = nullptr)
{
    return re.match(begin, end, match_res);
}

inline bool regex_match(const std::string &re,
        const std::string &str, match_result *match_res = nullptr)
{
    regex r(re);
    return regex_match(r, str, match_res);
}

inline bool regex_match(const std::string &re,
        const char *begin, const char *end, match_result *match_res = nullptr)
{
    regex r(re);
    return regex_match(r, begin, end, match_res);
}

inline bool regex_search(const regex &re,
        const char *begin, const char *end, match_result *match_res)
{
    return re.search(begin, end, match_res);
}

inline bool regex_search(const std::string &re,
        const char *begin, const char *end, match_result *match_res)
{
    regex r(re);
    return regex_search(r, begin, end, match_res);
}

inline auto regex_search_all(const regex &re,
        const char *begin, const char *end) -> std::vector<match_result>
{
    std::vector<match_result> results;

    match_result match_res;
    if (regex_search(re, begin, end, &match_res))
    {
        results.push_back(match_res);
        while (regex_search(re, match_res.end_pos(), end, &match_res))
            results.push_back(match_res);
    }

    return results;
}

inline auto regex_search_all(const std::string &re,
        const char *begin, const char *end) -> std::vector<match_result>
{
    regex r(re);
    return regex_search_all(r, begin, end);
}

} // namespace skl

#endif // REGEX_H
