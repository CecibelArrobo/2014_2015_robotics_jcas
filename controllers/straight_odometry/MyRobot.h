/**
 * @file    MyRobot.h
 * @brief   Robot go across the world in a straight line using odometry
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @date    2014-11
 */

#include <iostream>
#include <cmath>
#include <vector>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED       100

class MyRobot : public DifferentialWheels {

private:
    int _time_step;

    Compass * _my_compass;

    double _left_speed, _right_speed, _left_encoder, _right_encoder;

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
          * @brief Converting bearing vector from compass to angle (in degrees).
          */
    double convert_bearing_to_degrees(const double* in_vector);

    /**
          * @brief Kind of movement to do
          */
    int move();

    /**
          * @brief Send actuators commands according to the mode
          */
    void mode_selection(int mode);

    /**
          * @brief Calculate actual position and crossed distance
          */
    vector <double> calc_distance(vector <double> prev_position);

};
