#pragma once

#include <functional>

namespace lemlib::motion_handler {
/**
 * @brief run a motion algorithm
 *
 * @param f the motion function
 *
 * @b Example:
 * @code {.cpp}
 * // a simple motion algorithm, as an example
 * void simpleMotion() {
 *   std::uint32_t prevTime = pros::millis();
 *   // run until the task has been notified
 *   while (!pros::Task::notify_take(true, 0)) {
 *     // motion algorithm stuff would go here
 *     // ...
 *     // wait 10 ms to free up CPU time for other tasks
 *     pros::Task::delay_until(&prevTime, 10);
 *   }
 * }
 *
 * // runs during the autonomous period
 * void autonomous() {
 *   // pass the motion to the motion handler
 *   lemlib::motion_handler::move([&] { simpleMotion(); });
 *   // "Hello World!" is printed immediately after the motion starts
 *   std::cout << "Hello World!" << std::endl;
 *   // but if we try to run another motion while one is still running,
 *   // it will wait until the last one stops running
 *   lemlib::motion_handler::move([&] { simpleMotion(); });
 *   std::cout << "Last motion ended, new motion started!" << std::endl;
 * }
 * @endcode
 */
void move(std::function<void(void)> f);
/**
 * @brief cancel the currently running motion, if it exists
 *
 * @b Example:
 * @code {.cpp}
 * // a simple motion algorithm, as an example
 * void simpleMotion() {
 *   std::uint32_t prevTime = pros::millis();
 *   // run until the task has been notified
 *   while (!pros::Task::notify_take(true, 0)) {
 *     // motion algorithm stuff would go here
 *     // ...
 *     // wait 10 ms to free up CPU time for other tasks
 *     pros::Task::delay_until(&prevTime, 10);
 *   }
 * }
 *
 * // runs during the autonomous period
 * void autonomous() {
 *   // pass the motion to the motion handler
 *   lemlib::motion_handler::move([&] { simpleMotion(); });
 *   lemlib::motion_handler::isMoving(); // returns true
 *   // cancel the motion
 *   lemlib::motion_handler::cancel();
 *   pros::delay(10); // give the task time to stop
 *   lemlib::motion_handler::isMoving(); // returns false
 * }
 * @endcode
 */
bool isMoving();
/**
 * @brief cancel the currently running motion, if it exists
 *
 * @b Example:
 * @code {.cpp}
 * // a simple motion algorithm, as an example
 * void simpleMotion() {
 *   std::uint32_t prevTime = pros::millis();
 *   // run until the task has been notified
 *   while (!pros::Task::notify_take(true, 0)) {
 *     // motion algorithm stuff would go here
 *     // ...
 *     // wait 10 ms to free up CPU time for other tasks
 *     pros::Task::delay_until(&prevTime, 10);
 *   }
 * }
 *
 * // runs during the autonomous period
 * void autonomous() {
 *   // pass the motion to the motion handler
 *   lemlib::motion_handler::move([]{ simpleMotion(); });
 *   lemlib::motion_handler::isMoving(); // returns true
 *   // cancel the motion
 *   lemlib::motion_handler::cancel();
 *   pros::delay(10); // give the task time to stop
 *   lemlib::motion_handler::isMoving(); // returns false
 * }
 * @endcode
 */
void cancel();
/**
 * @brief wait until the robot reaches a certain radius within a designated point
 *
 * @b Example:
 * @code {.cpp}
 * 
 * // runs during the autonomous period
 * void autonomous() {
 *   // move the robot to a (60, 40).
 *   lemlib::moveToPoint({60_in, 40_in}, 5_sec, params, settings);
 *   // wait until the robot is in an 8-inch radius from (50, 30).
 *   lemlib::motion_handler::waitUntilRadius(50_in, 30_in, 8_in, [] -> units::Pose { return odom.getPose(); });
 *   // cancel the motion
 *   lemlib::motion_handler::cancel();
 *   pros::delay(10); // give the task time to stop
 * }
 * @endcode
 */
void waitUntilRadius(Length x, Length y, Length radius, std::function<units::Pose()> poseGetter);
/**
 * @brief wait until the robot reaches a certain radius within a designated point
 *
 * @b Example:
 * @code {.cpp}
 * 
 * // runs during the autonomous period
 * void autonomous() {
 *   // move the robot to a (60, 40).
 *   lemlib::moveToPoint({60_in, 40_in}, 5_sec, params, settings);
 *   // wait until the robot has traveled 12 inches since the function has been called.
 *   lemlib::motion_handler::waitUntilProgress(12_in, [] { return odom.getPose(); });
 *   // cancel the motion
 *   lemlib::motion_handler::cancel();
 *   pros::delay(10); // give the task time to stop
 * }
 * @endcode
 */
void waitUntilProgress(Length dist, std::function<units::Pose()> poseGetter);
} // namespace lemlib::motion_handler