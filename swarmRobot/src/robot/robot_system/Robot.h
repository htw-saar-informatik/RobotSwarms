#include <cmath>

//
// Created by smanier on 30.09.18.
//

#ifndef PROJECT_TURTLEPOSITION_H
#define PROJECT_TURTLEPOSITION_H

#include <string>
#include <math.h>
#include <iostream>
#include "../../various/Constants.h"
#include "../../basics/Angle.h"
#include "../../basics/BasicStructs.h"

struct Robot : public Circle {
    Angle angle;
    unsigned int index;

    Robot(unsigned int index = 0)
            : Robot({0.0f, 0.0f}, index) {
    }

    Robot(Position pos, unsigned int index = 0)
            : Robot(std::move(pos), 0.0f, index) {
    }

    Robot(Position pos, Angle angle, unsigned int index = 0)
            : Circle(std::move(pos), ROBOT_SIZE), angle(angle), index(index) {
    }

    std::string toString() const {
        return "X: " + std::to_string(position.x) + ", Y: " + std::to_string(position.y) + ", Deg: " +
               std::to_string(angle.get());
    }

    Angle angleTo(const Position &destination) const {
        return position.angleTo(destination);
    }

    Angle getRelativeAngle(const Position &destination) const {
        Angle directAngle = angleTo(destination);
        directAngle.normalizeToTurnAngle();
        return directAngle - this->angle;
    }

    Position simulateMovement(Angle angle, const float speed) const {
        Position newPos = this->position;
        newPos.x += cosf(angle.get() * static_cast<float>(M_PI) / 180.0f) * speed;
        newPos.y += sinf(angle.get() * static_cast<float>(M_PI) / 180.0f) * speed;
        return newPos;
    }

    float distance(const Robot &other) const {
        return this->position.distance(other.position);
    }

    bool operator==(const Robot &other) const {
        return this->index == other.index;
    }

    bool operator!=(const Robot &other) const {
        return this->index != other.index;
    }

    Robot &operator=(const Robot &other) {
        this->index = other.index;
        this->position = other.position;
        this->angle = other.angle;
        return *this;
    }
};

namespace myString {
    std::string toString(const std::vector<Robot> &vec) {
        std::ostringstream out;
        for (const Robot &r : vec) {
            out << r.toString() << ", ";
        }
        return out.str();
    }
}

#endif //PROJECT_TURTLEPOSITION_H
