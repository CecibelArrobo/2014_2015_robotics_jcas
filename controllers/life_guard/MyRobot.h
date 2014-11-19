#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   The robot go across the world avoid obstacle
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @date    2014-10
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 8
#define DISTANCE_LIMIT      100
#define MAX_SPEED           50

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

    double convert_bearing_to_degrees(const double* in_vector);

    double normal_angle(double angle);

    double color_detector(int camera, int color);

    bool person_detector();

private:
    int _time_step;



    // Velocoties
    double _left_speed, _right_speed;

    // Encoders
    double  _right_encoder, _left_encoder;

    // Sensors
    DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];
    Camera *_forward_camera;
    Camera *_spherical_camera;
    Compass * _my_compass;

    enum Mode {
        STOP,
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        OBSTACLE_AVOID_LEFT,
        OBSTACLE_AVOID_RIGHT,
        BACK,
        TURN_AROUND,
    };

    Mode _mode;
};
#endif
