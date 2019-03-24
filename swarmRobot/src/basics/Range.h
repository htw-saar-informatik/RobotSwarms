//
// Created by smanier on 16.01.19.
//

#pragma once

#include "MyAlgorithms.h"

template<typename T>
struct Range {
    struct iterator {
        constexpr iterator(T value)
                : value{value} {

        }

        constexpr T operator*() const {
            return value;
        }

        constexpr T get() const {
            return value;
        }

        iterator &operator++() {
            ++value;
            return *this;
        }

        constexpr bool operator!=(iterator rhs) const {
            return value != rhs.value;
        }

    private:
        T value;
    };

    Range(T from, T to) {
        alg::sort(from, to);
        this->from = from;
        this->to = to;
    }

    constexpr iterator begin() const {
        return iterator(from);
    }

    constexpr iterator end() const {
        return iterator(to);
    }

    constexpr bool valueInRange(T value) const {
        return (value >= from) and (value < to);
    }

private:
    T from;
    T to;
};