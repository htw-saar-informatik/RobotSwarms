//
// Created by smanier on 07.01.19.
//

#pragma once

#include <string>
#include <sstream>

namespace myString {
    template<typename T>
    std::string toString(const T a_value, const int n = 3) {
        std::ostringstream out;
        out.precision(n);
        out << std::fixed << a_value;
        return out.str();
    }
}