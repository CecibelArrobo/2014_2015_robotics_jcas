#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   Robot detected white walls
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @date    2014-11
 */
#include <iostream>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED           10

class MyRobot : public DifferentialWheels {

public:
    // You may need to define your private methods or variables, like
    //  Constructors, helper functions, etc.

    /**
     * @brief Empty constructor of the class.
     */
    MyRobot();

    /**
     * @brief Destructor of the class.
     */
    ~MyRobot();

    /**
     * @brief Function with the logic of the controller.
     */
    void run();

    /**
     * @brief Movements todo by the robot
     * @param mode movement selector
     */
    void movement(int mode);
    /**
     * @brief Calculate percentage of white pixels
     * @return white percentage
     */
    double white_detector();

private:

    int _time_step;

    // velocities
    double _left_speed, _right_speed;

    // sensors
    Camera *_forward_camera;
};

#endif

