//
// Created by smanier on 31.10.18.
//

#ifndef PROJECT_TIME_H
#define PROJECT_TIME_H

#include <cstdint>
#include <ctime>

class Time {
private:
    uint8_t getAndReduce(time_t &time, uint8_t reduction) const {
        const uint8_t value = time % reduction;
        time /= reduction;
        return value;
    }

public:
    uint8_t sec, min, h;

    Time(time_t millis) {
        sec = getAndReduce(millis, 60);
        min = getAndReduce(millis, 60);
        h = getAndReduce(millis, 24);
    }
};

#endif //PROJECT_TIME_H
