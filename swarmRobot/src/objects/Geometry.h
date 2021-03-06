//
// Created by smanier on 21.11.18.
//
#pragma once

#include "../basics/BasicStructs.h"
#include "../basics/Position.h"

class Geometry {
public:
    Position position;

    Geometry(Position pos, unsigned long id = 0)
            : position(pos) {

    }

    void move_relative(const Position &pos) {
        this->position.x += pos.x;
        this->position.y += pos.y;
    }

    void move_absolute(const Position &pos) {
        this->position.x = pos.x;
        this->position.y = pos.y;
    }

    virtual float max_y() const = 0;

    virtual float min_y() const = 0;

    virtual float max_x() const = 0;

    virtual float min_x() const = 0;

    virtual bool inBoundary(const Position &pos) const = 0;

    virtual ~Geometry() {

    }

    virtual std::string toString() const {
        return "Min_x: " + myString::toString(min_x()) + ", Max_x: " + myString::toString(max_x())
               + ", Min_y: " + myString::toString(min_y()) + ", Max_y: " + myString::toString(max_y());
    }
};

class Rectangle : public Geometry {
private:
    const Size recSize;
    const bool inverted;

public:
    Rectangle(Position pos, Size size, bool inverted = false)
            : Geometry(pos), recSize(size), inverted(inverted) {
    }

    virtual bool inBoundary(const Position &pos) const override {
        const bool inside = (pos.x > min_x()) and (pos.x < max_x())
                            and (pos.y > min_y()) and (pos.y < max_y());

        return (!inverted) == inside;
    }

    virtual float max_x() const override {
        return this->position.x + recSize.x / 2.0f;
    }

    virtual float min_x() const override {
        return this->position.x - recSize.x / 2.0f;
    }

    virtual float max_y() const override {
        return this->position.y + recSize.y / 2.0f;
    }

    virtual float min_y() const override {
        return this->position.y - recSize.y / 2.0f;
    }
};

class Circle : public Geometry {
private:
    const float radius;

public:
    Circle(Position pos, float radius)
            : Geometry(pos), radius(radius) {
    }

    virtual bool inBoundary(const Position &otherPosition) const override {
        return this->position.distance(otherPosition) < radius;
    }

    virtual float max_y() const override {
        return this->position.y + radius;
    }

    virtual float min_y() const override {
        return this->position.y - radius;
    }

    virtual float max_x() const override {
        return this->position.x + radius;
    }

    virtual float min_x() const override {
        return this->position.x - radius;
    }
};