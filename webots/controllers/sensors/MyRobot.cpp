/**
 * @file    MyRobot.cpp
 * @brief   While the robot moves, print frontal distance sensor values to console
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @date    2014-10
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    //Get and enable distance sensors
    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds2");
    _distance_sensor[2]->enable(_time_step);
    _distance_sensor[3] = getDistanceSensor("ds3");
    _distance_sensor[3]->enable(_time_step);
    _distance_sensor[4] = getDistanceSensor("ds12");
    _distance_sensor[4]->enable(_time_step);
    _distance_sensor[5] = getDistanceSensor("ds13");
    _distance_sensor[5]->enable(_time_step);
    _distance_sensor[6] = getDistanceSensor("ds14");
    _distance_sensor[6]->enable(_time_step);
    _distance_sensor[7] = getDistanceSensor("ds15");
    _distance_sensor[7]->enable(_time_step);
}


//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // Disabled all sensors
    _my_compass->disable();
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double compass_angle;

    while (step(_time_step) != -1) {

        // Read the sensors
        const double *compass_val = _my_compass->getValues();
        const double distanceSensor_val_0 = _distance_sensor[0]->getValue();
        const double distanceSensor_val_1 = _distance_sensor[1]->getValue();
        const double distanceSensor_val_2 = _distance_sensor[2]->getValue();
        const double distanceSensor_val_3 = _distance_sensor[3]->getValue();
        const double distanceSensor_val_12 = _distance_sensor[4]->getValue();
        const double distanceSensor_val_13 = _distance_sensor[5]->getValue();
        const double distanceSensor_val_14 = _distance_sensor[6]->getValue();
        const double distanceSensor_val_15 = _distance_sensor[7]->getValue();


        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;
        cout << "Distance sensor 0: " << distanceSensor_val_0 << endl;
        cout << "Distance sensor 1: " << distanceSensor_val_1 << endl;
        cout << "Distance sensor 2: " << distanceSensor_val_2 << endl;
        cout << "Distance sensor 3: " << distanceSensor_val_3 << endl;
        cout << "Distance sensor 12: " << distanceSensor_val_12 << endl;
        cout << "Distance sensor 13: " << distanceSensor_val_13 << endl;
        cout << "Distance sensor 14: " << distanceSensor_val_14 << endl;
        cout << "Distance sensor 15: " << distanceSensor_val_15 << endl;


        // Robot moves in a straight line
        if (compass_angle < (DESIRED_ANGLE - 2)) {
            // Turn right
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED - 15;
        }
        else {
            if (compass_angle > (DESIRED_ANGLE + 2)) {
                // Turn left
                _left_speed = MAX_SPEED - 15;
                _right_speed = MAX_SPEED;
            }
            else {
                // Move straight forward
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;
            }
        }

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////
