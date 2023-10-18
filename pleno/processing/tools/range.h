#pragma once //from charlib

#include <cassert>

struct RangeIndex
{
    int begin_index;
    int end_index;

    bool empty() const { return begin_index == end_index; }
    size_t size() const { return end_index - begin_index; }

    struct RangeIndexIterator
    {
        int index;
        bool operator!=(const RangeIndexIterator& rg_it) const { return rg_it.index != index; }
        void operator++() { ++index; }
        auto& operator*() { return index; }
    };

    RangeIndexIterator begin() { return RangeIndexIterator{begin_index}; }
    RangeIndexIterator end() { return RangeIndexIterator{end_index}; }
};

template<typename T>
struct Range
{
    T& container;
    RangeIndex indexes;

    auto begin() { return std::next(container.begin(), indexes.begin_index); }
    auto end() { return std::next(container.begin(), indexes.end_index); }

    bool empty() const { return indexes.empty(); }
    size_t size() const { return indexes.size(); }

    auto& operator[](const int i) const
    {
        assert(i >= 0 && i <= indexes.size() - 1 && "Range::operator[]: out of range.");
        return *std::next(container.begin(), indexes.begin_index + i);
    };

