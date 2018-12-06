//
// Created by smanier on 24.10.18.
//

#ifndef PROJECT_LOGGER_H
#define PROJECT_LOGGER_H

#include <vector>
#include <ostream>
#include <iostream>
#include <stdarg.h>
#include "Time.h"

std::string RobotName(int index) {
    return "FlockRobot[" + std::to_string(index) + "]";
}

enum LOG_LEVEL {
    NEVER = -1, DEEP_DEBUG = 0, DEBUG = 1, INFO = 2, WARNING = 3, ERROR = 4, CRITICAL = 5
};

std::string LOGGER_CAPTION;
const time_t START_TIME = time(nullptr);

class Logger {
private:
    const LOG_LEVEL LEVEL_OF_LOGGING = INFO;

    static const int BUFFER_SIZE = 200;
    const LOG_LEVEL logLevel;
    bool restricted = false;

    std::string getName(LOG_LEVEL level) const {
        switch (level) {
            case NEVER: return "NEVER";
            case DEEP_DEBUG: return "DEEP_DEBUG";
            case DEBUG: return "DEBUG";
            case INFO: return "INFO";
            case WARNING: return "WARNING";
            case ERROR: return "ERROR";
            case CRITICAL: return "CRITICAL";
            default: throw "Logger name not assigned!";
        }
    }

    void print(const char *msg, ...) const {
        if ((!restricted) && (logLevel >= LEVEL_OF_LOGGING)) {
            const Time currenTime(time(nullptr) - START_TIME);
            printf("%02d:%02d:%02d %s: [%-8s] %s\n", currenTime.h, currenTime.min, currenTime.sec,
                   LOGGER_CAPTION.c_str(), getName(logLevel).c_str(), msg);
        }
    }

public:
    Logger() = delete;

    Logger(LOG_LEVEL logLevel, int restricted) : logLevel(std::move(logLevel)) {
        this->restricted = LOGGER_CAPTION.compare(RobotName(restricted)) != 0;
    }

    Logger(LOG_LEVEL logLevel, const char *msg, ...) : logLevel(std::move(logLevel)) {
        va_list arglist;
        va_start(arglist, msg);
        char buffer[BUFFER_SIZE];
        vsprintf(buffer, msg, arglist);
        print(buffer);
    }

    Logger(LOG_LEVEL logLevel, const std::string msg) : logLevel(std::move(logLevel)) {
        print(msg.c_str());
    }

    Logger(LOG_LEVEL logLevel, int restricted, const char *msg, ...) : logLevel(std::move(logLevel)) {
        this->restricted = LOGGER_CAPTION.compare(RobotName(restricted)) != 0;

        va_list arglist;
        va_start(arglist, msg);
        char buffer[BUFFER_SIZE];
        vsprintf(buffer, msg, arglist);
        print(buffer);
    }

    Logger(LOG_LEVEL logLevel, int restricted, const std::string msg) : logLevel(std::move(logLevel)) {
        this->restricted = LOGGER_CAPTION.compare(RobotName(restricted)) != 0;
        print(msg.c_str());
    }

    Logger &newLine(const char *msg, ...) {
        va_list arglist;
        va_start(arglist, msg);
        char buffer[BUFFER_SIZE];
        vsprintf(buffer, msg, arglist);
        print(buffer);

        return *this;
    }

    Logger &newLine(const std::string msg) {
        print(msg.c_str());
        return *this;
    }

    Logger &bar() {
        print("===== ===== ===== ===== ===== ===== ===== =====");
        return *this;
    }
};

#endif //PROJECT_LOGGER_H
