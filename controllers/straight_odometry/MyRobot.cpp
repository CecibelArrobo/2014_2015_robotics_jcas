/**
 * @file    MyRobot.cpp
 * @brief   Robot go across the world in a straight line using odometry
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @date    2014-11
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
    _my_compass -> enable(_time_step);

    // Eneable and get encoders
    enableEncoders(_time_step);
    _left_encoder = getLeftEncoder();
    _right_encoder = getRightEncoder();
}


//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // Disabled compass sensor
    _my_compass -> disable();

    // Disabled enconders
    disableEncoders();
}

//////////////////////////////////////////////

void MyRobot::run(){

    double compass_angle;
    double x = 0, z = 0;
    double distance = 0;

    while (step(_time_step) != -1) {

        // Vector to initial position and actual position
        vector <double> position, actual_position;

        // Read the compass sensor
        const double *compass_val = _my_compass->getValues();
        compass_angle = convert_bearing_to_degrees(compass_val);

        cout << "Compass angle: " << compass_angle << endl;

        // Initialization for initial position
        position.push_back(x);
        position.push_back(z);
        position.push_back(compass_angle*M_PI/180);

        // If crossed distance (distance) is smaller than desired distance (18.3)
        // robot moves  depend of encoders position
        // gets actual position
        if (distance < 18.3 ){ // Estimate error 1.3 m, yellow line distance is 17 m

            mode_selection(move());

            actual_position = calc_distance(position);
            x = actual_position.at(0);
            z = actual_position.at(1);
            distance = actual_position.at(2);

            cout << "x: " << x << ", z: " << z << ", theta: " << compass_angle*M_PI/180 << endl;
            cout << "distance: " << distance << endl;
        }
        // Else, desired distance achivied and robot stops
        else {

            mode_selection(3);
        }
    }
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[2], in_vector[0]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////

int MyRobot::move()
{

    // Get encoders position
    double l_encoder = getLeftEncoder();
    double r_encoder = getRightEncoder();
    int mode = -1; // return variable, mode to robot movements.

    // If both sensors measure the same, the speed of both wheels is the same
    if(l_encoder == r_encoder){

        mode = 0;
    }
    else{

        // If left encoder measure is greater than right encoder measure
        // decrease left speed
        if(l_encoder > r_encoder){

            mode = 1;
        }
        // Else, right encoder measure is greater than left encoder measure
        // decrease right speed
        else{

            mode = 2;
        }
    }

    return mode;
}

//////////////////////////////////////////////

void MyRobot::mode_selection(int mode)
{

    switch (mode){

    case 0:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED;
        break;

    case 1:
        _left_speed = MAX_SPEED-10;
        _right_speed = MAX_SPEED;
        break;

    case 2:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED-10;
        break;

    case 3: //Stop
        _left_speed = 0;
        _right_speed = 0;

    default:
        break;
    }

    // Set the motor speeds
    setSpeed(_left_speed, _right_speed);
}

//////////////////////////////////////////////

vector <double> MyRobot::calc_distance(vector <double> prev_position){

    double x, z, theta;
    double distance = 0;
    vector <double> pos;
    // Odometry variables, to know robot position
    double d_l = 0.0528, d_r =  d_l , d_z, d_x, D_c;

    // Get previous position
    x = prev_position.at(0);
    z = prev_position.at(1);
    theta = prev_position.at(2);

    // Calculate odometry parameters
    D_c = (d_l+ d_r)/2;
    d_x = D_c * cos(theta);
    d_z = D_c * sin(theta);

    // Calculate actual position and crossed distance
    x = x + d_x;
    z = z + d_z;
    distance = sqrt((x*x)+(z*z));

    // set actual position (pos)
    pos.push_back(x);
    pos.push_back(z);;
    pos.push_back(distance);

    return pos;
}



