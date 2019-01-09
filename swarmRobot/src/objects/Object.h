//
// Created by smanier on 21.11.18.
//

#pragma once

#include <vector>
#include <memory>

#include "Geometry.h"
#include "../logger/Logger.h"
#include "../basics/MyAlgorithms.h"

class Object {
private:
    std::vector<std::unique_ptr<Geometry>> shapes;

public:
    Object() = delete;

    Object(Geometry *movableGeometry) {
        addShape(movableGeometry);
    }

    void move_relative(const Position &pos) {
        for (const auto &i : shapes) {
            i->move_relative(pos);
        }
    }

    void move_absolute(const Position &pos) {
        for (const auto &i : shapes) {
            i->move_absolute(pos);
        }
    }

    void addShape(Geometry *movableGeometry) {
        shapes.push_back(std::unique_ptr<Geometry>(movableGeometry));
    }

    bool inBoundary(const Position &pos) const {
        for (const auto &i : shapes) {
            if (i->inBoundary(pos)) {
                return true;
            }
        }

        return false;
    }

    Position getCentre() const {
        float min_x(99), min_y(99), max_x(0), max_y(0);

        for (const auto &i : shapes) {
            alg::makeMax(max_x, i->max_x());
            alg::makeMax(max_y, i->max_y());
            alg::makeMin(min_x, i->min_x());
            alg::makeMin(min_y, i->min_y());
        }

        return {(min_x + max_x) / 2.0f, (min_y + max_y) / 2.0f};
    }

    Size getSize() const {
        float min_x(99), min_y(99), max_x(0), max_y(0);

        for (const auto &i : shapes) {
            alg::makeMax(max_x, i->max_x());
            alg::makeMax(max_y, i->max_y());
            alg::makeMin(min_x, i->min_x());
            alg::makeMin(min_y, i->min_y());
        }

        return Size(max_x - min_x, max_y - min_y);
    }

    Object &operator=(Object &other) = delete;

    void print() const {
        for (const auto &s : shapes) {
            Logger(ALWAYS, 1, s->toString());
        }
    }

    std::string toString() const {
        std::ostringstream out;
        float min_x(99), min_y(99), max_x(0), max_y(0);
        for (const auto &i : shapes) {
            alg::makeMax(max_x, i->max_x());
            alg::makeMax(max_y, i->max_y());
            alg::makeMin(min_x, i->min_x());
            alg::makeMin(min_y, i->min_y());
        }

        out.precision(3);
        out << "MinX: " << min_x
            << ", MaxX: " << max_x
            << ", MinY: " << min_y
            << ", MaxY: " << max_y;
        return out.str();
    }
};