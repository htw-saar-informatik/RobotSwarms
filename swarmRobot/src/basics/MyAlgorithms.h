//
// Created by smanier on 06.12.18.
//

#pragma once

#include <algorithm>
#include "../logger/Logger.h"

namespace alg {
    /**
     * Sets "max" to the maximum of both values
     * @param max This variable will be the bigger one
     * @param other This one may be copied to "max" but remains untouched
     */
    template<typename T>
    void makeMax(T &max, const T &other) {
        if (max < other) {
            max = other;
        }
    }

    /**
     * Sets "min" to the minimum of both values
     * @param min This variable will be the smaller one
     * @param other This one may be copied to "min" but remains untouched
     */
    template<typename T>
    void makeMin(T &min, const T &other) {
        if (min > other) {
            min = other;
        }
    }

    /**
     * Sorts both variables so "min" will be the smaller one and "max" the bigger one
     * @param min
     * @param max
     */
    template<typename T>
    void sort(T &min, T &max) {
        if (min > max) std::swap(min, max);
    }

    /**
     * Tests if number is in range of the given range
     * @param number The value to test
     * @param range The range to test
     * @return True if the number is in range, else false
     */
    template<typename T>
    bool inRange(const T number, const std::pair<T, T> range) {
        return inRange(number, range.first, range.second);
    }

    /**
     * Tests if number is in range of the given numbers
     * @param number The value to test
     * @param from The buttom of the range
     * @param to The top of the range
     * @return True if the number is in range, else false
     */
    template<typename T>
    bool inRange(const T number, T from, T to) {
        sort(from, to);
        return (number >= from) and (number < to);
    }

    /**
     * Tests if the given object is null
     * @param object The object to test
     * @param name The name of the object
     * @param shutdown If set, the programm will shutdown if the test evaluates to true
     * @return True if the object is null, else false
     */
    template<typename T>
    bool assertNotNull(const T &object, const std::string name, bool shutdown = false) {
        if (object == nullptr) {
            Logger(CRITICAL, "Given object is null: %s", name);
            if (shutdown) exit(-1);
            return true;
        }

        return false;
    }

    /**
     * Tests if the given condition is false
     * @param object The condition to test
     * @param name The message to display
     * @param shutdown If set, the programm will shutdown if the test evaluates to true
     * @return True if the condition is false, else false
     */
    bool assertTrue(const bool condition, const std::string msg, bool shutdown = false) {
        if (!condition) {
            Logger(CRITICAL, msg);
            if (shutdown) exit(-1);
            return true;
        }

        return false;
    }
}