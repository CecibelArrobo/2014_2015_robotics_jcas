/**
 * @file    MyRobot.h
 * @brief   The robot go across the world avoid obstacle
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @date    2014-10
 */

#include <iostream>

#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 4
#define DISTANCE_LIMIT      100
#define MAX_SPEED           30

class MyRobot : public DifferentialWheels {
    private:
        int _time_step;

        DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];
        double _left_speed, _right_speed;

        enum Mode {
            STOP,
            FORWARD,
            TURN_LEFT,
            TURN_RIGHT,
            OBSTACLE_AVOID_LEFT,
            OBSTACLE_AVOID_RIGHT,
            BACK,
        };

        Mode _mode;

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
         * @brief User defined function for initializing and running the MyRobot class.
         */
        void run();

        /**
          * @brief Allot mode depending of measured distance by sensors
          * @param sensor_val[] sensor values array
          */
        void Control(double sensor_val[]);

        /**
          * @brief Send actuators commands according to the mode
          */
        void mode_selection();
};
