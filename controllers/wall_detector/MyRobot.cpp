/**
 * @file    MyRobot.cpp
 * @brief   Robot detected white walls
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @date    2014-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    // init default values
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // Get and enable forward camera
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable devices
    _forward_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double percentage_white = 0.0; // percentage of white pixels

    while (step(_time_step) != -1) {

        percentage_white = white_detector();

        // Robot moves to find a wall
        if(percentage_white > 88.1){

            movement(0);

            cout << "Detected wall" << endl;
        }
        else {

            // forward slowly
            movement(1);
        }

        // set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

void MyRobot::movement(int mode)
{

    switch (mode){

    case 0: // Stop
        _left_speed = 0;
        _right_speed = 0;
        break;

    case 1: // Forward
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED;
        break;

    default:
        break;
    }
}

//////////////////////////////////////////////

double MyRobot:: white_detector()
{

    int sum = 0; // White pixels number
    unsigned char green = 0, red = 0, blue = 0; // Green, red, blue pixel value
    double percentage_white = 0.0; // percentage of white pixels

    // get size of images for forward camera
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = _forward_camera->getHeight();

    // get current image from forward camera
    const unsigned char *image_f = _forward_camera->getImage();

    // count number of pixels that are white
    // (here assumed to have pixel value > 245 out of 255 for all color components)
    for (int x = 0; x < image_width_f; x++) {
        for (int y = 0; y < image_height_f; y++) {
            green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
            red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
            blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

            if ((green > 245) && (red > 245) && (blue > 245)) {
                sum = sum + 1;
            }
        }
    }

    percentage_white = (sum / (float) (image_width_f * image_height_f)) * 100;

    return percentage_white;
}
