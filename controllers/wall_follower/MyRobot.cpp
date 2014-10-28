/**
 * @file    MyRobot.cpp
 * @brief   Robot follows walls
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @date    2014-10
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _desired_angle = -45;

    _left_speed = 0;
    _right_speed = 0;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
    //Get and enable distance sensors
    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);   
    _distance_sensor[1] = getDistanceSensor("ds15");
    _distance_sensor[1]->enable(_time_step);
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
    int control=0;

    while (step(_time_step) != -1) {
        // Read the sensors
        const double *compass_val = _my_compass->getValues();
        const double distanceSensor_val_0 = _distance_sensor[0]->getValue();
        const double distanceSensor_val_15 = _distance_sensor[1]->getValue();

        // Convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);        

        // Print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;
        cout << "Distance sensor 0: " << distanceSensor_val_0 << endl;
        cout << "Distance sensor 15: " << distanceSensor_val_15 << endl;

        // If distance to an object is less than 15, robot moves straight
        if((distanceSensor_val_0 < 15) && (distanceSensor_val_15 < 15)){

            // Change control value when ds0 and ds15 don't detect anything
            if((distanceSensor_val_0 == 0) && (distanceSensor_val_15 == 0)){

                control = 0;
            }

            if (compass_angle < (_desired_angle - 2)) {

                // Turn right
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED/1.25;
                cout << "Turning right" << endl;
            }
            else {
                if (compass_angle > (_desired_angle + 2)){

                    // Turn left
                    _left_speed = MAX_SPEED / 1.25;
                    _right_speed = MAX_SPEED;
                    cout << "Turning left" << endl;
                }
                else{

                    // Move straight forward
                    _left_speed = MAX_SPEED;
                    _right_speed = MAX_SPEED;
                    cout << "Moving forward" << endl;
                }
            }
        }

        // Else, the distance is higher than 15
        else{

            _left_speed = -MAX_SPEED / 20.0;
            _right_speed = -MAX_SPEED / 3.0;

            // Increase angle only once
            if  (control ==0){

                _desired_angle += 90.0;

                // Convert the angle in the opossite
                if (_desired_angle > 180){

                    _desired_angle -= 360;
                }

               control +=1;
            }
            cout << "Backing up and turning right" << endl;
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
