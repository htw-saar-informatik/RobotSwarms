#include "../robot_system/RobotPilot.h"
#include <cstdlib>
#include <ctime>

#include "../flock/FlockControl.h"
#include "../../objects/Object.h"

void loop(RobotPilot &pilot, const Object &object);

int main(int argc, char **argv) {
    // ===== Main init =====
    const int index = (int) std::strtol(argv[1], nullptr, 10);
    const string &robotName = RobotName(index);
    ros::init(argc, argv, robotName);
    LOGGER_CAPTION = robotName;

    const long seed = std::chrono::system_clock::now().time_since_epoch().count();
    auto rndGenerator = std::default_random_engine(static_cast<unsigned long>(seed * index));
    const Size &borderSize = BORDER.getSize();
    auto rndDistributor = std::uniform_real_distribution<float>(borderSize.x + 1, borderSize.y - 1);

    // ===== Robot init =====
    ros::spinOnce();
    Position spawn{rndDistributor(rndGenerator), rndDistributor(rndGenerator)};
    RobotPilot pilot(index, spawn);
    subscribeToFlock();

    // ===== Movable init =====
    const Object object(new Rectangle({5, 5}, {4, 4}));

    try {
        loop(pilot, object);
    } catch (const char *message) {
        Logger(CRITICAL, "Catched error message: %s", message);
    } catch (const string &message) {
        Logger(CRITICAL, "Catched error message: %s", message.c_str());
    } catch (const int errorCode) {
        Logger(CRITICAL, "Catched errorCode: %d", errorCode);
    } catch (const exception &e) {
        Logger(CRITICAL, "Catched exception: %s", e.what());
    } catch (...) {
        Logger(CRITICAL, "Catched unknown object-exception");
    }

    Logger(CRITICAL, "Robot %d shutting down", index);
}

void loop(RobotPilot &pilot, const Object &object) {
    pilot.hold();

    while (ros::ok()) {
        if (!object.inBoundary(pilot.position())) {
            pilot.moveTo(object.getCentre());
        } else {
            Angle turnAngle = pilot.nextStep_withNoise();
            float currentSpeed = ROBOT_SPEED;

            while ((!pilot.makeMoveLegal(turnAngle, currentSpeed * 1.5f, object)) && ros::ok()) {
                Logger(WARNING, 4, "Simulated move not in boundary");
                currentSpeed *= 0.99f;
                usleep(1);
                ros::spinOnce();
            }

            pilot.move(turnAngle);
        }
    }
}