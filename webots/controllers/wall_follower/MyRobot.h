/**
 * @file    MyRobot.h
 * @brief   Robot follows walls
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @date    2014-10
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED       100
#define NUM_DISTANCE_SENSOR 2

class MyRobot : public DifferentialWheels {
    private:
        int _time_step;
        double _desired_angle;
        Compass * _my_compass;
        double _left_speed, _right_speed;
        DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];

    public:
        /**
         * @brief Empty constructor of the class.
         */
        MyRobot();

        /**
         * @brief Destructor of the class.
         */
        ~MyRobot();

        /**
         * @brief User defined function for initializing and running the template class.
         */
        void run();

        /**
          * @brief An example for converting bearing vector from compass to angle (in degrees).
          */
        double convert_bearing_to_degrees(const double* in_vector);
};
