//
// Created by smanier on 06.12.18.
//

#pragma once

#include <algorithm>

namespace alg {
    template <typename T> void makeMax(T &max, const T &other) {
        if (max < other){
            max = other;
        }
    }

    template <typename T> void makeMin(T &min, const T &other) {
        if (min < other){
            min = other;
        }
    }
}