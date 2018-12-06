//
// Created by smanier on 21.11.18.
//
#pragma once

#include "../basics/BasicStructs.h"

class Geometry {
public:
    Position position;

    Geometry(Position pos) : position(pos) {

    }

    void move_relative(const float x, const float y) {
        this->position.x += x;
        this->position.y += y;
    }

    virtual float max_y() const = 0;

    virtual float min_y() const = 0;

    virtual float max_x() const = 0;

    virtual float min_x() const = 0;

    virtual bool inBoundary(const Position &pos) const = 0;

    virtual ~Geometry() {

    }
};

class Rectangle : public Geometry {
private:
    const Size recSize;

public:
    Rectangle(Position pos, Size size)
            : Geometry(pos), recSize(size) {
    }

    virtual bool inBoundary(const Position &pos) const override {
        return (pos.x > min_x()) && (pos.x < max_x())
               && (pos.y > min_y()) && (pos.y < max_y());
    }

    virtual float max_y() const override {
        return this->position.y + recSize.y / 2.0f;
    }

    virtual float min_y() const override {
        return this->position.y - recSize.y / 2.0f;
    }

    virtual float max_x() const override {
        return this->position.x + recSize.x / 2.0f;
    }

    virtual float min_x() const override {
        return this->position.x - recSize.x / 2.0f;
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