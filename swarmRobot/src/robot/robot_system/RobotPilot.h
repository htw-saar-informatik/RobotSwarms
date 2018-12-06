#pragma once

#include "../flock/Statistics.h"
#include "../../logger/Logger.h"
#include "RobotCore.h"

using namespace std;

class RobotPilot : public RobotCore {
public:
    explicit RobotPilot(int index, bool disablePen = true)
            : RobotCore(index, disablePen) {
        refreshPosition();
    }

    RobotPilot(int index, const Position &spawnPosition, bool disablePen = true)
            : RobotCore(index, spawnPosition, 0, disablePen) {
        refreshPosition();
    }

    RobotPilot(int index, const Position &spawnPosition, const Angle &spawnAngle, bool disablePen = true)
            : RobotCore(index, spawnPosition, spawnAngle, disablePen) {
        refreshPosition();
    }

    void move(Angle &turnAngle, bool withNoise = false, const float speed = ROBOT_SPEED) {
        Logger(DEEP_DEBUG, "Current position: %s", this->position().toString().c_str());
        if (withNoise) {
            addNoise(turnAngle);
        }

        move_internal_relative(turnAngle, speed);
        antiBump();
    }

    void move(float speed = ROBOT_SPEED) {
        Angle turnAngle = nextStep_withNoise();
        move(turnAngle, false, speed);
    }

    void moveTo(const Position &pos, bool withNoise = false, float speed = ROBOT_SPEED) {
        Angle turnAngle = self().angleTo(pos);
        move(turnAngle, withNoise, speed);
    }

    void hold() {
        geometry_msgs::Twist msg;
        publisher.publish(msg);

        sleeper.sleep();
        ros::spinOnce();
    }

    Angle nextStep_withNoise() {
        const vector<Robot> &flock = findFlock(swarm[static_cast<unsigned long>(myIndex)]);
        Angle turnAngle = averageDegree(flock);
        addNoise(turnAngle);

        if (flock.size() > 1) {
            const Angle &angleToFlockCentre = self().angleTo(flockCentre(flock));
            Angle angleDiff = angleToFlockCentre - turnAngle;
            angleDiff.normalizeToTurnAngle();
            turnAngle += angleDiff * FLOCK_TURN_TO_FLOCK_PERCENT;
        }

        makeMoveLegal(turnAngle, ROBOT_SPEED, BORDER);
        return turnAngle;
    }

    bool predictRobotCollision(const Position &destination) const {
        for (const Robot &robot : swarm) {
            if ((robot != self()) && robot.inBoundary(destination)) {
                Logger(WARNING, 4, "Robot collision");
                return true;
            }
        }

        return false;
    }

    bool predictLegalMove(const Position &destination, const Object &stayIn) {
        return (!predictRobotCollision(destination)) && stayIn.inBoundary(destination);
    }

    bool makeMoveLegal(Angle &turnAngle, const float speed, const Object &stayIn) {
        for (int i = 0; i < 181; i++) {
            if (predictLegalMove(self().simulateMovement(turnAngle + i, speed), stayIn)) {
                turnAngle += i;
                return true;
            }

            if (predictLegalMove(self().simulateMovement(turnAngle - i, speed), stayIn)) {
                turnAngle -= i;
                return true;
            }
        }

        return false;
    }

    void waitForFlockReady() {
        while ((!flockIsReady()) and ros::ok()) {
            hold();
        }
    }

    ~RobotPilot() = default;

private:
    void addNoise(Angle &turnAngle) {
        turnAngle += rndDistributor(rndGenerator);
    }

    void antiBump() {
        const Robot &mySelf = self();

        for (const Robot &other : swarm) {
            if ((other != mySelf) and other.inBoundary(mySelf.position)) {
                const Angle &turnAngle = mySelf.angleTo(other.position) + 180.0f;

                float speed = 0;
                while (other.inBoundary(mySelf.simulateMovement(turnAngle, speed))) {
                    speed += 0.01f;
                }

                teleport_absolute(mySelf.simulateMovement(turnAngle, speed), mySelf.angle);
            }
        }
    }

    // ========== ========== ========= ========== ==========
    // ========== ========== INTERNALS ========== ==========
    // ========== ========== ========= ========== ==========

    void move_internal_relative(const Angle &turnAngle, float speed) {
        const Angle &relativeTurnAngle = turnAngle - self().angle;
        this->turn_teleport_relative(relativeTurnAngle);

        geometry_msgs::Twist msg;
        msg.linear.x = speed;
        publisher.publish(msg);

        Logger(DEEP_DEBUG, "RobotPilot::move()", "Moving to: %s",
               self().simulateMovement(relativeTurnAngle, ROBOT_SPEED).toString().c_str());

        sleeper.sleep();
        ros::spinOnce();
    }
};