//
// Created by smanier on 21.11.18.
//

#pragma once

#include <cmath>
#include <string>

struct Position {
    Position(const float x, const float y) : x(x), y(y) {

    }

    float distance(const Position &other) const {
        const float x_diff = this->x - other.x;
        const float y_diff = this->y - other.y;
        return std::sqrt(x_diff * x_diff + y_diff * y_diff);
    }

    std::string toString() const {
        return std::string("X: " + std::to_string(x) + ", Y: " + std::to_string(y));
    }

    float x, y;
};

struct Size {
    Size(const float x, const float y) : x(x), y(y) {

    }

    float x, y;
};
