#include "RegexFAElement.h"
#include "RegexNFA.h"

namespace regex
{
    namespace automata
    {
        std::string Node::TypeDesc() const
        {
            std::string desc;
            if (type_ & Type_Start)
            {
                desc += "Type_Start";
            }
            if (type_ & Type_Accept)
            {
                if (!desc.empty())
                    desc += " ";
                desc += "Type_Accept";
            }
            if (type_ & Type_Normal)
            {
                if (!desc.empty())
                    desc += " ";
                desc += "Type_Normal";
            }
            if (type_ & Type_Repeat)
            {
                if (!desc.empty())
                    desc += " ";
                desc += "Type_Repeat";
            }
            return desc;
        }

        void EdgeSet::Insert(const Edge &range)
        {
            int current = range.first_;
            int last = range.last_;

            while (current <= last)
            {
                Edge cur(current, current);
                Iterator it = edges_.lower_bound(cur);
                if (it == edges_.end())
                {
                    edges_.insert(Edge(current, last));
                    current = last + 1;
                }
                else
                {
                    if (current < it->first_)
                    {
                        if (last < it->first_)
                        {
                            edges_.insert(Edge(current, last));
                            current = last + 1;
                        }
                        else
                        {
                            edges_.insert(Edge(current, it->first_ - 1));
                            current = it->first_;
                        }
                    }
                    else if (current == it->first_)
                    {
                        if (last < it->last_)
                        {
                            int back_first = last + 1;
                            int back_last = it->last_;
                            edges_.erase(it);
                            edges_.insert(Edge(current, last));
                            edges_.insert(Edge(back_first, back_last));
                            current = last + 1;
                        }
                        else
                        {
                            current = it->last_ + 1;
                        }
                    }
                    else
                    {
                        if (last < it->last_)
                        {
                            int front_first = it->first_;
                            int front_last = current - 1;
                            int middle_first = current;
                            int middle_last = last;
                            int back_first = last + 1;
                            int back_last = it->last_;
                            edges_.erase(it);
                            edges_.insert(Edge(front_first, front_last));
                            edges_.insert(Edge(middle_first, middle_last));
                            edges_.insert(Edge(back_first, back_last));
                            current = last + 1;
                        }
                        else
                        {
                            int front_first = it->first_;
                            int front_last = current - 1;
                            int back_first = current;
                            int back_last = it->last_;
                            edges_.erase(it);
                            edges_.insert(Edge(front_first, front_last));
                            edges_.insert(Edge(back_first, back_last));
                            current = back_last + 1;
                        }
                    }
                }
            }
        }
    } // namespace automata
} // namespace regex
