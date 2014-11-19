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
    _distance_sensor[1] = getDistanceSensor("ds3");
    _distance_sensor[2] = getDistanceSensor("ds4");
    _distance_sensor[3] = getDistanceSensor("ds6");
    _distance_sensor[4] = getDistanceSensor("ds9");
    _distance_sensor[5] = getDistanceSensor("ds11");
    _distance_sensor[6] = getDistanceSensor("ds12");
    _distance_sensor[7] = getDistanceSensor("ds14");


    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->enable(_time_step);
    }

    // Get and enable forward camera
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);

    // Get and enable spherical camera
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);

    // get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    // Eneable and get encoders
    enableEncoders(_time_step);
    _left_encoder = getLeftEncoder();
    _right_encoder = getRightEncoder();
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }

    // Disable devices
    _forward_camera->disable();
    _spherical_camera->disable();
    _my_compass->disable();

    // Disabled enconders
    disableEncoders();
}

//////////////////////////////////////////////

void MyRobot::run()
{

    double ir_val[NUM_DISTANCE_SENSOR];
    int control=0;
    double compass_angle, angle;

    while (step(_time_step) != -1) {


        // Read the sensors
        for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {

            ir_val[i] = _distance_sensor[i]->getValue();
        }

        const double *compass_val = _my_compass->getValues();

        // convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Read right encoder
        _right_encoder = getRightEncoder();


        if ( person_detector() == true && control == 0){

            cout << "Detected person" << endl;
            _mode = STOP;
            angle = compass_angle;
            cout << "Angle: "<< angle<< endl;
            control=1;
        }
        else{

            _mode = FORWARD;

            // Call to Control function
            //Control(ir_val);

        }

        if(control == 1 && compass_angle < angle){


            cout << "Encoder: " << _right_encoder << endl;
            _mode = TURN_AROUND;


        }

        // Call to mode_selection function
        mode_selection();


    }
}

//////////////////////////////////////////////

void MyRobot::Control(double sensor_val[]) {

    if(sensor_val[0]== 0 && sensor_val[1] == 0 &&sensor_val[2]== 0 &&sensor_val[3]== 0 &&sensor_val[4]== 0 && sensor_val[5]== 0 &&sensor_val[6]== 0 &&sensor_val[7]== 0)
    {
        _mode = FORWARD;
    }
    if(sensor_val[0]== 0 && sensor_val[1] >100 && sensor_val[2]> 100 &&sensor_val[3]== 0 &&sensor_val[4]== 0 && sensor_val[5]== 0 &&sensor_val[6]== 0 &&sensor_val[7]== 0)
    {
        _mode = FORWARD;
    }
    if(sensor_val[0]== 0 && sensor_val[1] == 0 &&sensor_val[2]== 0 &&sensor_val[3]== 0 &&sensor_val[4]== 0 && sensor_val[5] > 100 &&sensor_val[6]> 100 &&sensor_val[7]== 0)
    {
        _mode = FORWARD;
    }
    if(sensor_val[0]== 0 && sensor_val[1] == 0 &&sensor_val[2]== 0 &&sensor_val[3]== 0 &&sensor_val[4]== 0 && sensor_val[5]== 0 &&sensor_val[6]== 0 &&sensor_val[7]> 100)
    {
        _mode = TURN_LEFT;
    }
    if(sensor_val[0]> 100 && sensor_val[1] == 0 &&sensor_val[2]== 0 &&sensor_val[3]== 0 &&sensor_val[4]== 0 && sensor_val[5]== 0 &&sensor_val[6]== 0 &&sensor_val[7]== 0)
    {
        _mode = TURN_RIGHT;
    }
    if(sensor_val[0]== 0 && sensor_val[1] == 0 &&sensor_val[2]== 0 &&sensor_val[3]== 0 &&sensor_val[4]== 0 && sensor_val[5]> 100 &&sensor_val[6]== 0 &&sensor_val[7]== 0)
    {
        _mode = TURN_RIGHT;
    }
    if(sensor_val[0]== 0 && sensor_val[1] == 0 &&sensor_val[2] > 100 &&sensor_val[3]== 0 &&sensor_val[4]== 0 && sensor_val[5]== 0 &&sensor_val[6]== 0 &&sensor_val[7]== 0)
    {
        _mode = TURN_LEFT;
    }
    if(sensor_val[0]> 100 && sensor_val[1] == 0 &&sensor_val[2]== 0 &&sensor_val[3]== 0 &&sensor_val[4]== 0 && sensor_val[5]== 0 &&sensor_val[6]== 0 &&sensor_val[7]> 100)
    {
        _mode = OBSTACLE_AVOID_LEFT;
    }
    if(sensor_val[0]== 0 && sensor_val[1] == 0 &&sensor_val[2]== 0 &&sensor_val[3]== 0 &&sensor_val[4]== 0 && sensor_val[5]== 0 &&sensor_val[6]== 0 &&sensor_val[7]== 0)
    {
        _mode = FORWARD;
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

    case TURN_LEFT:
        _left_speed = MAX_SPEED / 2;
        _right_speed = MAX_SPEED;
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

    // Set the motor speeds
    setSpeed(_left_speed, _right_speed);
}

//////////////////////////////////////////////


double MyRobot:: color_detector(int camera, int color)
{

    int sum = 0; // Color pixels number
    unsigned char green = 0, red = 0, blue = 0; // Green, red, blue pixel value
    double percentage = 0.0; // percentage of color pixels
    const unsigned char *image;
    int image_width, image_height;

    switch (camera){
    // get current image from forward camera and get size of image
    case 0:
        image = _forward_camera->getImage();
        image_width = _forward_camera->getWidth();
        image_height = _forward_camera->getHeight();
        break;
        // get current image from spherical camera and get size of images for spherical camera
    case 1:
        image = _spherical_camera->getImage();
        image_width = _spherical_camera->getWidth();
        image_height = _spherical_camera->getHeight();
        break;
    default:
        break;
    }

    // count number of pixels that are white
    // (here assumed to have pixel value > 245 out of 255 for all color components)
    for (int x = 0; x < image_width; x++) {
        for (int y = 0; y < image_height; y++) {
            green = _spherical_camera->imageGetGreen(image, image_width, x, y);
            red = _spherical_camera->imageGetRed(image, image_width, x, y);
            blue = _spherical_camera->imageGetBlue(image, image_width, x, y);

            switch(color){
            case 0:
                if ((green > 245) && (red > 245) && (blue > 245)) { // White == wall

                    sum = sum + 1;
                }
                break;
            case 1:
                if ((green > 190) && (red < 60) && (blue < 20)) { // Green == person

                    sum = sum + 1;
                }
                break;
            default:
                break;
            }
        }
    }

    percentage = (sum / (float) (image_width * image_height)) * 100;

    cout <<"Porcentaje: " << percentage << endl;

    return percentage;
}

//////////////////////////////////////////////

bool MyRobot::person_detector(){

    bool detected = false; // Variable to know if robots detecs a person

    if(color_detector(0,1) > 50.0){

        detected = true;
    }

    return detected;
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = normal_angle(rad * (180.0 / M_PI));
    return deg;
}
//////////////////////////////////////////////

double MyRobot::normal_angle(double angle){

    if(angle < 0 ){

        angle += 360;
    }
    else if(angle>360){

        angle -= 360;
    }

    return angle;
}
