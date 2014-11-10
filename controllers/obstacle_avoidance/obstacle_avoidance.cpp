/**
 * @file    obstacle_avoidance.cpp
 * @brief   The robot go across the world avoid obstacle
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @date    2014-10
 */

#include "MyRobot.h"

// This is the main program of our controller.
// It creates an instance of your Robot subclass, launches its
// function(s) and destroys it at the end of the execution.


/**
 * @brief Main program.
 * @param argc
 * @param **argv
 * @return 0
 */
int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
