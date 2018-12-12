#include <utility>

//
// Created by smanier on 20.10.18.
//

#ifndef PROJECT_ANGLE_H
#define PROJECT_ANGLE_H


#include <cmath>
#include <algorithm>

class Angle {
private:
    float value;

    void normalize() {
        value = fmodf(value, 360.0f);
    }

public:
    Angle(float value) : value(value) { // NOLINT(google-explicit-constructor)
        normalize();
    }

    Angle(float vec_x, float vec_y) {
        value = std::atan2(vec_y, vec_x) * 180.0f / SMALL_PI;
        normalize();
    }

    Angle operator+(const Angle &other) const {
        Angle created = this->value + other.value;
        created.normalize();
        return created;
    }

    Angle &operator+=(const Angle &other) {
        this->value += other.value;
        normalize();
        return *this;
    }

    Angle operator-(const Angle &other) const {
        Angle created = this->value - other.value;
        created.normalize();
        return created;
    }

    Angle &operator-=(const Angle &other) {
        this->value -= other.value;
        normalize();
        return *this;
    }

    Angle operator*(float other) const {
        Angle created = this->value * other;
        created.normalize();
        return created;
    }

    Angle &operator*=(float other) {
        this->value *= other;
        normalize();
        return *this;
    }

    Angle operator/(float other) const {
        Angle created = this->value / other;
        created.normalize();
        return created;
    }

    Angle &operator/=(float other) {
        this->value /= other;
        normalize();
        return *this;
    }

    bool operator<(Angle other) const {
        return this->value < other.get();
    }

    bool operator>(Angle other) const {
        return this->value > other.get();
    }

    float get() const {
        return value;
    }

    float toRad() const {
        float rad = value / 180.0f * SMALL_PI;

        if (rad > M_PI) {
            rad = (2.0f * SMALL_PI) - rad;
            rad *= -1.0f;
        }

        return rad;
    }

    void normalizeToTurnAngle() {
        normalize();

        if (this->value < -180.0f) this->value += 360.0f;
        if (this->value > 180.0f) this->value -= 360.0f;
    }

    void normalizeToCircularAngle() {
        normalize();

        if (this->value < 0) this->value = 360.0f - this->value;
    }

    bool isBetween(float min, float max) const {
        return alg::inRange(this->value, min, max);
    }
};

Angle meanAngle(const std::vector<Angle> &angles) {
    double y_part = 0, x_part = 0;

    for (const Angle &angle : angles) {
        x_part += std::cos(angle.get() * M_PI / 180.0f);
        y_part += std::sin(angle.get() * M_PI / 180.0f);
    }

    return Angle(static_cast<float>(x_part / angles.size()), static_cast<float>(y_part / angles.size()));
}

Angle radToDegree(float rad) {
    float degree = rad * 180.0f / static_cast<float>(M_PI);
    if (degree < 0.0) degree += 360.0f;
    return degree;
}


#endif //PROJECT_ANGLE_H
