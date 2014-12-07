/**
 * @file    life_prueba.cpp
 * @brief   The robot go across the world, search two people trapped and
 *          go back to start line.
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @author  Marcos Paulo Nascimento Gouveia <1100329047@alumnos.uc3m.es>
 * @date    2014-12
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
