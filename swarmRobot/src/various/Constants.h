//
// Created by smanier on 06.10.18.
//

#pragma once

#include "../objects/Object.h"

// Math
static const float SMALL_PI = static_cast<float>(M_PI);

// Flock constants
static const int FLOCK_SIZE = 0 + 5;
static const float FLOCK_TURN_TO_FLOCK_PERCENT = 5.0f/*Percent*/ / 100.0f;
static const float FLOCK_RANDOM_TURN_DEGREE = 45.0f;
static const float FLOCK_LOCALITY_RANGE = 2.0f;

// Robot constants
static const float ROBOT_SPEED_PERCENT = 0.1f;
static const float ROBOT_SPEED = FLOCK_LOCALITY_RANGE * ROBOT_SPEED_PERCENT;
static const float ROBOT_SIZE = 0.25f;

// ROS constants
static const float TURTLESIM_SIZE = 11.088889f;
static const float TURTLESIM_BORDER_THICKNESS = 0.1f;
static const Object BORDER(new Rectangle(
        Position(TURTLESIM_SIZE / 2.0f, TURTLESIM_SIZE / 2.0f),
        Size(TURTLESIM_SIZE - TURTLESIM_BORDER_THICKNESS, TURTLESIM_SIZE - TURTLESIM_BORDER_THICKNESS)
));

// ROS TOPICS
static const std::string TOPIC_MISSION_NEW = "flock/mission/new";
static const std::string TOPIC_MISSION_IN_POSITION = "flock/mission/inPosition";
static const std::string TOPIC_ROBOT_MISSION_FINISHED = "flock/mission/finished";
static const std::string TOPIC_ROBOT_READY = "flock/ready";