/**
 * @file    MyRobot.cpp
 * @brief   The robot go across the world avoid obstacle
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

    _mode = FORWARD;

    _distance_sensor[0] = getDistanceSensor("ds1");
    _distance_sensor[1] = getDistanceSensor("ds14");
    _distance_sensor[2] = getDistanceSensor("ds12");
    _distance_sensor[3] = getDistanceSensor("ds13");

    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->enable(_time_step);
    }

    // Get and enable forward camera
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }

    // Disable devices
    _forward_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double ir_val[NUM_DISTANCE_SENSOR];

    while (step(_time_step) != -1) {

        // Read the sensors
        for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {

            ir_val[i] = _distance_sensor[i]->getValue();
        }

        // Show sensor value
        // cout << "ds1: " << ir_val[0] << " ds14:" << ir_val[1] << endl;
        // cout << " ds12:" << ir_val[2]<< " ds13:" << ir_val[3] << endl;

        // Call to Control function
        //Control(ir_val);





        cout << "Green Percentage: " << person_detector() << endl;

        // Call to mode_selection function
        mode_selection();

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

void MyRobot::Control(double sensor_val[]) {

    // If distance measure by distance sensor 1 and distance sensor 14 is greater
    // than DISTANCE_LIMIT = 100, there are an obstacle, robot avoid obstacle
    if ((sensor_val[0] > DISTANCE_LIMIT) || (sensor_val[1] > DISTANCE_LIMIT)) {

        _mode = OBSTACLE_AVOID_LEFT;
        cout << "Backing up and turning left." << endl;

    }
    else {

        // If distance measure by distance sensor 12 is equal to cero, don't detect
        // any obstacle on the right lateral, and distance measure by distance
        // sensor 13 is smaller than 150, on right back there are an obstacle but
        // distance is enough to turn right without crash
        if((sensor_val[2] == 0.0) && (sensor_val[3] < (DISTANCE_LIMIT + 50))){
            _mode = TURN_RIGHT;
            cout << "Turning right." << endl;

        }
        /// Else, moving forward
        else{

            _mode = FORWARD;
            cout << "Moving forward." << endl;
        }
    }
}

//////////////////////////////////////////////

void MyRobot::mode_selection(){

    switch (_mode){

    case FORWARD:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED;
        break;
    case TURN_RIGHT:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED / 2;
        break;
    case OBSTACLE_AVOID_LEFT:
        _left_speed = -MAX_SPEED / 3.0;
        _right_speed = -MAX_SPEED / 20.0;
        break;
    case STOP:
        _left_speed = 0;
        _right_speed = 0;
        break;
    case TURN_AROUND:
        _left_speed = -MAX_SPEED / 5;
        _right_speed = MAX_SPEED / 5;
        break;
    default:
        break;
    }
}

//////////////////////////////////////////////

double MyRobot:: person_detector()
{

    int sum = 0; // White pixels number
    unsigned char green = 0, red = 0, blue = 0; // Green, red, blue pixel value
    double percentage_green = 0.0; // percentage of white pixels
    int contador = 0;

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

            //cout << "Verde: " << (int)green << ", rojo:" << (int)red << ", azul: " << (int) blue << endl;

            if ((green > 190) && (red < 60) && (blue < 20)) { // Green == person
                sum = sum + 1;

                //cout << "Suma:" << sum << endl;
            }
        }
    }

    percentage_green = (sum / (float) (image_width_f * image_height_f)) * 100;

    if (percentage_green > 50.0){

        cout << "Detected person" << endl;
        _mode = STOP;

        while(contador<3000){
            _mode = TURN_AROUND;

            contador++;
        }

    }
    else {

        _mode = FORWARD;
    }

    return percentage_green;
}


