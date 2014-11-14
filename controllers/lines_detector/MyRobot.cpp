/**
 * @file    MyRobot.cpp
 * @brief   Robot detected yellow lines
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @date    2014-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    // Init default values
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // Get and enable spherical camera
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable devices
    _spherical_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double perthousand_yellow = 0.0; // Use per thousand because the width line is very small

    while (step(_time_step) != -1) {

        perthousand_yellow = yellow_detector();

        if((perthousand_yellow > 14.0) && (perthousand_yellow < 16.2)){

            cout << "Detected yellow line" << endl;
        }

        movement(1);

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

double MyRobot:: yellow_detector()
{

    int sum = 0; // Yellow pixels number
    unsigned char green = 0, red = 0, blue = 0; // Green, red, blue pixel value
    double perthousand_yellow = 0.0; // Use per thousand because the width line is very small

    // get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();

    // get current image from spherical camera
    const unsigned char *image_s = _spherical_camera->getImage();

    // count number of pixels that are yellow
    // (here assumed to have pixel green, red value > 206 and blue value == 0 out of 255)
    for (int x = 0; x < image_width_s; x++) {

        for (int y = 0; y < image_height_s; y++) {

            green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
            red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
            blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);

            if ((green > 206) && (red > 206) && (blue == 0)) {

                sum = sum + 1;
            }
        }
    }

    perthousand_yellow = (sum / (float) (image_width_s * image_height_s)) * 1000;

    return perthousand_yellow;
}

