//
// Created by smanier on 06.10.18.
//

#pragma once

#include "../objects/Object.h"

// Flock constants
static const int FLOCK_SIZE = 0 + 10;
static const float FLOCK_TURN_TO_FLOCK_PERCENT = 5.0f/*Percent*/ / 100.0f;
static const float FLOCK_RANDOM_TURN_DEGREE = 45.0f;
static const float FLOCK_LOCALITY_RANGE = 4.0f;

// Robot constants
static const float ROBOT_SPEED_PERCENT = 0.1f;
static const float ROBOT_SPEED = 4.0f * ROBOT_SPEED_PERCENT;
static const float ROBOT_SIZE = 0.25f;

// ROS constants
static const float TURTLESIM_SIZE = 11.088889f;
static const float TURTLESIM_BORDER_THICKNESS = 0.01f;
static Object BORDER(new Rectangle(
        Position(TURTLESIM_SIZE / 2.0f, TURTLESIM_SIZE / 2.0f),
        Size(TURTLESIM_SIZE - TURTLESIM_BORDER_THICKNESS, TURTLESIM_SIZE - TURTLESIM_BORDER_THICKNESS)
), 0);

// ROS TOPICS
static const unsigned int ROS_QUEUE_SIZE = 0;
static const std::string TOPIC_MISSION_NEW = "flock/mission/new";
static const std::string TOPIC_MISSION_IN_POSITION = "flock/mission/inPosition";
static const std::string TOPIC_ROBOT_MISSION_FINISHED = "flock/mission/finished";
static const std::string TOPIC_ROBOT_READY = "flock/ready";