//
// Created by smanier on 21.11.18.
//

#pragma once

#include <cmath>
#include <string>
#include "Angle.h"
#include "ToString.h"

struct Position {
    Position(const float x, const float y) : x(x), y(y) {

    }

    float distance(const Position &other) const {
        const float x_diff = this->x - other.x;
        const float y_diff = this->y - other.y;
        return std::sqrt(x_diff * x_diff + y_diff * y_diff);
    }

    Angle angleTo(const Position &destination) const {
        float x_distance = destination.x - x;
        float y_distance = destination.y - y;

        return Angle(x_distance, y_distance);
    }

    Position operator-(const Position &other) const {
        float x = this->x - other.x;
        float y = this->y - other.y;
        return Position(x, y);
    }

    bool operator==(const Position &other) {
        return (std::fabs(x - other.x) < 0.001) and (std::fabs(y - other.y) < 0.001);
    }

    std::string toString() const {
        return std::string("X: " + myString::toString(x) + ", Y: " + myString::toString(y));
    }

    float x, y;
};

struct Size {
    Size(const float x, const float y) : x(x), y(y) {

    }

    float x, y;
};
