/**
 * @file    MyRobot.cpp
 * @brief   The robot go across the world, search two people trapped and
 *          go back to start line.
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @author  Marcos Paulo Nascimento Gouveia <1100329047@alumnos.uc3m.es>
 * @date    2014-12
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{

    _time_step = 64;
    _left_speed = 0;
    _right_speed = 0;
    _person_found = 0;
    _finish_line = false;
    _start_line = false;
    _person_detected = false;
    _different_person = true;
    _corner = false;
    _init_pos = false;
    _final_pos = false;
    _time = 0;

    _mode = FORWARD;

    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[2] = getDistanceSensor("ds2");
    _distance_sensor[3] = getDistanceSensor("ds3");

    _distance_sensor[4] = getDistanceSensor("ds4");
    _distance_sensor[5] = getDistanceSensor("ds5");
    _distance_sensor[6] = getDistanceSensor("ds6");
    _distance_sensor[7] = getDistanceSensor("ds7");

    _distance_sensor[8] = getDistanceSensor("ds8");
    _distance_sensor[9] = getDistanceSensor("ds9");
    _distance_sensor[10] = getDistanceSensor("ds10");
    _distance_sensor[11] = getDistanceSensor("ds11");

    _distance_sensor[12] = getDistanceSensor("ds12");
    _distance_sensor[13] = getDistanceSensor("ds13");
    _distance_sensor[14] = getDistanceSensor("ds14");
    _distance_sensor[15] = getDistanceSensor("ds15");

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
}

//////////////////////////////////////////////

void MyRobot::run()
{

    double ir_val[NUM_DISTANCE_SENSOR];

    double compass_angle;
    int status = 0, turn = 0 ;
    int angle;
    double x = 0, z = 0;


    while (step(_time_step) != -1) {

        vector <double> position, actual_position;

        // Read the sensors
        for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {

            ir_val[i] = _distance_sensor[i]->getValue();
        }


        const double *compass_val = _my_compass->getValues();

        // convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // Actual position
        position.push_back(x);
        position.push_back(z);
        position.push_back(compass_angle*M_PI/180);

        // Lines detector
        cross_line(position);

        // Main control
        // If robot is in initial position
        if(_init_pos == true ){

            // If the person found number is two and do the rotation, go to home
            if(_person_found == 2 && status == 2){

                cout << "PERSONAS ENCONTRADAS" << endl;
                go_home(ir_val, position);
            }
            // Else, look for people
            else{

                switch(status){

                // Search person
                case 0 :
                    person_find(ir_val, position);

                    // If detected one perso, catch the angle an change status to turn
                    if (_person_detected == true) {

                        _time = 0;
                        angle = (int)normal_angle(_person_angle+180);
                        status =1;
                    }
                    break;

                // Turn 360 degrees
                case 1:
                    _person_detected = false;

                    // If time is more than 2 seconds, turns
                    if(tempo() == true){

                        // Turn 180+180 degrees
                        if(turn < 2){

                            if (compass_angle < (angle-2) || compass_angle > (angle+2)){

                                _mode = TURN_AROUND;
                            }
                            else{

                                _mode = STOP;
                                angle =(int) normal_angle(compass_angle+180);
                                turn +=1;
                            }
                        }
                        // Finish, the 360 degrees round, change to statutus 2
                        else{

                            status = 2;
                        }
                    }
                    //Else, wait two seconds
                    else{

                        _mode = STOP;
                    }

                    break;

                // Turn 180 degrees, to put on bottom the person find, then look for person again
                case 2:

                    turn = 0;
                    if (compass_angle < (angle-2) || compass_angle > (angle+2)){

                        _mode = TURN_AROUND;
                    }
                    else{

                        _mode = STOP;
                        status = 0;
                    }
                    break;

                default: break;
                }
            }
        }
        // Else, place the robot in the initial position
        else{
            initial_position(ir_val, compass_angle);
        }

        // If the robot is in a corner, the robot turn around to right for two seconds
        if(_corner == true){
            if(tempo() == false){

                _mode = TURN_AROUND_RIGHT;
            }
            else{
                _corner= false;
                _time = 0;
            }
        }

        //Updated the robot position
        actual_position = odometry(position);
        x = actual_position.at(0);
        z = actual_position.at(1);

        // Call to mode_selection function
        mode_selection();

        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////

void MyRobot::goAhead(double sensor_val[], vector<double> position)
{

    double x, z, theta;

    x = position.at(0);
    z = position.at(1);
    theta = position.at(2)*180/M_PI;

    //Avoid a frontal ostacle
    if((sensor_val[1] > DISTANCE_LIMIT) || (sensor_val[14] > DISTANCE_LIMIT)){

        if(abs(sensor_val[1]-sensor_val[14])>50){

            // If left frontal sensor is more close, avoid obstacle turning right, and back sensor are away from the wall
            if((sensor_val[1]>sensor_val[14]) && (sensor_val[7]< 1000 && sensor_val[8]<1000)){

                    _mode = OBSTACLE_AVOID_RIGHT;

            }
            else{

                // If right frontal sensor is more close, avoid obstacle turning left, and back sensor are away from the wall
                if(sensor_val[7]< 1000 && sensor_val[8]<1000){

                        _mode = OBSTACLE_AVOID_LEFT;
                }
                else{

                    // Back sensor are close to wall, move forward slowly
                    _mode = FORWARD_SLOW;
                }
            }
        }
        else{

            // If the subtraction is less than 50 an the lateral sensor are greater than 400,it's a corner
            if(sensor_val[12]>400 || sensor_val[3]>400){

                _corner = true;
            }
        }
    }
    else{

        // If there aren't any obstacle near to the robot, try to keep 45 degrees orientation
        if((sensor_val[0]+sensor_val[1]+sensor_val[2]+sensor_val[3]+sensor_val[4]+sensor_val[11]+sensor_val[12]+sensor_val[13]+sensor_val[14]+sensor_val[15])==0 && (sqrt(x*x+z*z) < 12) ){

            if (theta > 40 && theta < 50){

                _mode = FORWARD;
            }
            else{

                if(theta > 50 || theta < -135){

                    _mode = TURN_LEFT_SOFT;
                }
                else{

                    _mode = TURN_RIGHT_SOFT;
                }
            }
        }
        // Else, depend of the sensors measure ...
        else {

            // There is a hole on the left
            if(sensor_val[3]==0 && (sensor_val[4] > (DISTANCE_LIMIT + 50)) && (sensor_val[1]+sensor_val[2] < 100)){

                _mode = TURN_AROUND;
            }
            else{

                // There is a hole on the right
                if(sensor_val[12]==0 && (sensor_val[11] > (DISTANCE_LIMIT + 50)) && (sensor_val[14]+sensor_val[13] < 100)){

                    _mode = TURN_AROUND_RIGHT;
                }
                else{

                    // Move forward
                    _mode = FORWARD;

                    // Avoid crash if robot is very close to a obstacle
                    avoid_crash(sensor_val);
                }
            }
        }
    }
}

//////////////////////////////////////////////

void MyRobot::goBack(double sensor_val[], vector<double> position)
{

    double theta;

    theta = position.at(2)*180/M_PI;

    //Avoid a frontal ostacle
    if((sensor_val[1] > DISTANCE_LIMIT) || (sensor_val[14] > DISTANCE_LIMIT)){

        if(abs(sensor_val[1]+sensor_val[14])>50){

             // If left frontal sensor is more close, avoid obstacle turning right, and back sensor are away from the wall
            if((sensor_val[1]>sensor_val[14]) && (sensor_val[7]< 1000 && sensor_val[8]<1000)){

                    _mode = OBSTACLE_AVOID_RIGHT;

            }
            else{

                // If right frontal sensor is more close, avoid obstacle turning left, and back sensor are away from the wall
                if(sensor_val[7]< 1000 && sensor_val[8]<1000){

                        _mode = OBSTACLE_AVOID_LEFT;
                }
                else{

                    // Back sensor are close to wall, move forward slowly
                    _mode = FORWARD_SLOW;
                }
            }
        }
        else{

            // If the subtraction is less than 50 an the lateral sensor are greater than 400,it's a corner
            if(sensor_val[12]>400 || sensor_val[3]>400){
                _corner = true;
            }
        }
    }
    else {

        // If there aren't any obstacle near to the robot, try to keep -135 degrees orientation
        if((sensor_val[0]+sensor_val[1]+sensor_val[2]+sensor_val[3]+sensor_val[4]+sensor_val[11]+sensor_val[12]+sensor_val[13]+sensor_val[14]+sensor_val[15])==0){

            if (theta > -140 && theta < -130){

                _mode = FORWARD;
            }
            else{

                if(theta > -130 && theta < 45){

                    _mode = TURN_LEFT_SOFT;
                }
                else{

                    _mode = TURN_RIGHT_SOFT;
                }
            }
        }
        else {

            // There is a hole on the left
            if(sensor_val[3]==0 && (sensor_val[4] > (DISTANCE_LIMIT + 50)) && (sensor_val[1]+sensor_val[2] < 100)){

                _mode = TURN_AROUND;
            }
            else{

                // There is a hole on the right
                if(sensor_val[12]==0 && (sensor_val[11] > (DISTANCE_LIMIT + 50))&& (sensor_val[14]+sensor_val[13] < 100)){

                    _mode = TURN_AROUND_RIGHT;
                }
                else{

                    // Move forward
                    _mode = FORWARD;

                    // Avoid crash if robot is very close to a obstacle
                    avoid_crash(sensor_val);
                }
            }
        }        
    }
}

//////////////////////////////////////////////

void MyRobot::mode_selection()
{

    switch (_mode){

    case FORWARD:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED;
        break;

    case FORWARD_SLOW:
        _left_speed = MAX_SPEED/2;
        _right_speed = MAX_SPEED/2;
        break;

    case TURN_RIGHT:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED-10;
        break;

    case TURN_RIGHT_SOFT:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED/2;
        break;

    case TURN_LEFT:
        _left_speed = MAX_SPEED-10;
        _right_speed = MAX_SPEED;
        break;

    case TURN_LEFT_SOFT:
        _left_speed = MAX_SPEED/2 ;
        _right_speed = MAX_SPEED;
        break;

    case OBSTACLE_AVOID_RIGHT:
        _left_speed = (-MAX_SPEED)/ 20;
        _right_speed = (-MAX_SPEED) /5 ;
        break;

    case OBSTACLE_AVOID_LEFT:
        _left_speed = (-MAX_SPEED) / 5;
        _right_speed = (-MAX_SPEED) / 20;
        break;

    case STOP:
        _left_speed = 0;
        _right_speed = 0;
        break;

    case TURN_AROUND:
        _left_speed = -MAX_SPEED/5;
        _right_speed = MAX_SPEED/5;
        break;

    case TURN_AROUND_RIGHT:
        _left_speed =  MAX_SPEED/5;
        _right_speed = -MAX_SPEED/5;
        break;

    case BACK:
        _left_speed =  -MAX_SPEED/1.25 ;
        _right_speed = -MAX_SPEED/1.25;
        break;

    default:
        break;
    }
}

//////////////////////////////////////////////

double MyRobot:: color_detector(int camera, int color)
{

    int sum = 0; // Color pixels number
    unsigned char green = 0, red = 0, blue = 0; // Green, red, blue pixel value
    double percentage = 0.0; // percentage of color pixels
    const unsigned char *image;
    int image_width =0 , image_height= 0;

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

    // count number of pixels that are each color
    for (int x = 0; x < image_width; x++) {

        for (int y = 0; y < image_height; y++) {

            green = _spherical_camera->imageGetGreen(image, image_width, x, y);
            red = _spherical_camera->imageGetRed(image, image_width, x, y);
            blue = _spherical_camera->imageGetBlue(image, image_width, x, y);

            switch(color){

            case 0:
                if ((green > 240) && (red > 240) && (blue > 240)) { // White == wall

                    sum = sum + 1;
                }
                break;

            case 1:
                if ((green > 30) && (red < 65) && (blue < 32) && ((green - red) > 20) && ((green - blue) > 15)) { // Green == person

                    sum = sum + 1;
                }
                break;

            case 2:
                if ((green > 70) && (red > 70) && (blue < 100) && (green==red) && ((red-blue)>50)) { // Yellow == line
                    sum = sum + 1;
                }
                break;

            default:
                break;
            }
        }
    }

    percentage = (sum / (float) (image_width * image_height)) * 100;

    return percentage;
}

//////////////////////////////////////////////


double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    // Angle in radians
    double rad = atan2(in_vector[0], in_vector[2]);
    // Angle to degrees
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////

double MyRobot::normal_angle(double angle)
{
    // If angle is smaller than -180 degrees
    if(angle < -180 ){

        angle += 360;
    }
    // If angle is greater than 180 degrees
    else if(angle >180){

        angle -= 360;
    }
    return angle;
}

//////////////////////////////////////////////

vector <double> MyRobot::odometry(vector <double> prev_position)
{
    double x, z, theta;

    vector <double> pos;

    // Odometry variables, to know robot position
    double d_l, d_r, d_z, d_x, D_c;

    d_l = _left_speed*0.085*0.064/10;
    d_r = _right_speed*0.085*0.064/10;

    // Get previous position
    x = prev_position.at(0);
    z = prev_position.at(1);
    theta = prev_position.at(2);
    // Calculate odometry parameters
    theta += (d_r-d_l)/0.38;
    D_c = (d_l+ d_r)/2;
    d_x = D_c * cos(theta);
    d_z = D_c * sin(theta);
    // Calculate actual position and crossed distance
    x = x + d_x;
    z = z + d_z;

    // set actual position (pos)
    pos.push_back(x);
    pos.push_back(z);;
    pos.push_back(theta);
    return pos;

}

//////////////////////////////////////////////

int MyRobot::local_person()
{
    int sum = 0; // Color pixels number
    unsigned char green = 0, red = 0, blue = 0; // Green, red, blue pixel value
    double percentage_left, percentage_right, percentage_front; // percentage of color pixels
    const unsigned char *image;
    int image_width =0 , image_height= 0;

    image = _spherical_camera->getImage();
    image_width = _spherical_camera->getWidth();
    image_height = _spherical_camera->getHeight();

    // Izquierda
    for (int x = 0; x < 40; x++) {
        for (int y = 40; y < 120; y++) {

            green = _spherical_camera->imageGetGreen(image, image_width, x, y);
            red = _spherical_camera->imageGetRed(image, image_width, x, y);
            blue = _spherical_camera->imageGetBlue(image, image_width, x, y);

            if ((green > 30) && (red < 65) && (blue < 32) && ((green - red) > 20) && ((green - blue) > 15)) { // Green == person

                sum = sum + 1;
            }
        }
    }
    percentage_left = (sum / (float) (image_width * image_height)) * 100;

    // Derecha
    sum = 0;

    for (int x = 120; x < image_width; x++) {
        for (int y = 40; y < 120; y++) {

            green = _spherical_camera->imageGetGreen(image, image_width, x, y);
            red = _spherical_camera->imageGetRed(image, image_width, x, y);
            blue = _spherical_camera->imageGetBlue(image, image_width, x, y);

            if ((green > 30) && (red < 65) && (blue < 32) && ((green - red) > 20) && ((green - blue) > 15)) { // Green == person

                sum = sum + 1;
            }
        }
    }
    percentage_right = (sum / (float) (image_width * image_height)) * 100;

    // Frente
    sum = 0;
    for (int x = 0; x < image_width; x++) {
        for (int y = 0; y < 40; y++) {

            green = _spherical_camera->imageGetGreen(image, image_width, x, y);
            red = _spherical_camera->imageGetRed(image, image_width, x, y);
            blue = _spherical_camera->imageGetBlue(image, image_width, x, y);

            if ((green > 30) && (red < 65) && (blue < 32) && ((green - red) > 20) && ((green - blue) > 15)) { // Green == person

                sum = sum + 1;
            }
        }
    }
    percentage_front = (sum / (float) (image_width * image_height)) * 100;

    if(percentage_left> percentage_front && percentage_left > percentage_right){

        // Turn left
        return 1;
    }
    else {

        if(percentage_front > percentage_left && percentage_front > percentage_right){

            // Forward
            return 2;
        }
        else{
            if(percentage_right > percentage_left && percentage_right > percentage_front){

                // Turn right
                return 3;
            }
            else {

                // goAhead
                return 0;
            }
        }
    }
}

//////////////////////////////////////////////

void MyRobot::turn_to_desired_angle(int desired_angle, double theta){

    // Decided the side to turn depend of the less route
    if(theta > (desired_angle+2) || theta < normal_angle(desired_angle-180)){
        // Turn left
        _mode = TURN_AROUND;
    }
    else{
        // Turn right
        _mode = TURN_AROUND_RIGHT;
    }

}

//////////////////////////////////////////////

void MyRobot::initial_position(double sensor_val[],double compass_angle)
{
    // If the robot is close to a wall, go back
    if( color_detector(0,0)> 88.1 && (sensor_val[1] > 100|| sensor_val[14]>100)){

        _mode = BACK;
    }
    // Else, robot is away from the wall
    else{

        // If robot angle is smaller than 43 degrees or is greater than 47 degrees, rotate to 45 degrees
        if (compass_angle < 43 || compass_angle > 47){

            turn_to_desired_angle(45, compass_angle);
        }
        // Else, robot is in the correct position
        else{

            _init_pos = true;
        }
    }
}

//////////////////////////////////////////////

void MyRobot::person_find(double sensor_val[],vector <double> position)
{
    double x, z, theta;
    int direction;

    // Get position
    x = position.at(0);
    z = position.at(1);
    theta = position.at(2);

    // Get direction
    direction = local_person();

    // Is the person is on the left, turn the left and avoid crash
    if (direction == 1){

        _mode = TURN_LEFT_SOFT;
        avoid_crash(sensor_val);
    }
    // Is the person is in front, forward and avoid crash
    if (direction == 2){

        _mode = FORWARD;
        avoid_crash(sensor_val);
    }
    // Is the person is to the right, turn the right and avoid crash
    if (direction == 3) {

        _mode = TURN_RIGHT_SOFT;
        avoid_crash(sensor_val);
    }
    // If don't see a person on spherical image or the person is the same person
    if (direction == 0 || _different_person == false) {

        // Is don't cross the finish line goAhead
        if (_finish_line == false){

            goAhead(sensor_val,position);
        }
        // Else, cross the finish line, only avoid crash while search people
        else{
            avoid_crash(sensor_val);
        }
    }

    // If frontal camera see the person, move forward and avoid crash
    if (color_detector(0,1) > 0.1 && color_detector(0,1) < 6){

        _mode = FORWARD;
        avoid_crash(sensor_val);
    }

    // If robot is close the person, detect the person
    if(color_detector(0,1) > 50.0){

        _person_detected = true;
    }

    // If detect a person, save his position
    if (_person_detected == true) {

        // Check if is other person
        if(different_person(position) == true){

            _person_angle = (int)(theta*180/M_PI);
            _person_found +=1;
            _P.setId(_person_found);
            _P.setX(x+sin(theta));
            _P.setZ(z+cos(theta));
            _persons.push_back(_P);
        }
        // If the person is the same, hange the angle to turn and search other person
        else{
            _person_angle = (int) normal_angle((theta*180/M_PI)+90);
        }
    }
}

//////////////////////////////////////////////

void MyRobot::cross_line(vector<double> position)
{
    // Get robot position
    double z = position.at(1);
    double x = position.at(0);

    // Finish line
    if((color_detector(1,2) > 1.65) && (color_detector(1,2) < 2.32) && (sqrt(x*x+z*z) > 7)){
        _finish_line = true;
    }
    // Start line
    if((color_detector(1,2) > 1.65) && (color_detector(1,2) < 2.32) && (sqrt(x*x+z*z) < 7) && _finish_line==true) {
        _start_line = true;
    }
}

//////////////////////////////////////////////

bool MyRobot::different_person(vector<double> position)
{
    if(_persons.size()>0){

        // Check if the new detected person is different of save person
        for( int i = 0; i <_persons.size(); i++){

            if(((position.at(0)+sin(position.at(2))) == _persons[i].getX()) &&((position.at(1)+cos(position.at(2))) == _persons[i].getX())){

                _different_person = false;
                return false;
            }
            else{

                _different_person = true;
                return true;
            }
        }
    }
    else{

        _different_person = true;
        return true;
    }
}

//////////////////////////////////////////////

bool MyRobot::tempo()
{
    _time += 1;

    // is time is smaller than 33, two seconds don't over. t = 64 ms * 32 = 2048 ms = 2 s 48 ms
    if(_time < 33 ){

        return false;
    }
    else{

        return true;
    }
}

//////////////////////////////////////////////

void MyRobot::avoid_crash(double sensor_val[])
{
    //Avoid a frontal ostacle
    if((sensor_val[1] > DISTANCE_LIMIT) || (sensor_val[14] > DISTANCE_LIMIT)){

        if(abs(sensor_val[1]+sensor_val[14])>50){

            if((sensor_val[1]>sensor_val[14]) && (sensor_val[7]< 1000 && sensor_val[8]<1000)){

                    _mode = OBSTACLE_AVOID_RIGHT;

            }
            else{

                if(sensor_val[7]< 1000 && sensor_val[8]<1000){

                        _mode = OBSTACLE_AVOID_LEFT;
                }
                else{
                    _mode = FORWARD_SLOW;
                }
            }

        }
        else{

            if(sensor_val[12]>400 || sensor_val[3]>400){
                _corner = true;
            }
        }

    }

    // Avoid crash with fine walls
    if(sensor_val[0] > 900 && sensor_val[15] > 900 && (sensor_val[13]+sensor_val[14]+sensor_val[1]+sensor_val[0])==0){

        _mode = BACK;
    }
    // Avoid crash with left lateral obstacle
    if(sensor_val[3] > 900 || sensor_val[4] > 900){

        _mode = TURN_RIGHT;
    }
    // Avoid crash with right lateral obstacle
    if(sensor_val[11] > 900 || sensor_val[12] > 900){
        _mode = TURN_LEFT;
    }
    // Avoid crash with left frontal obstacle
    if(sensor_val[2] > 900){
        _mode = TURN_RIGHT_SOFT;
    }
    // Avoid crash with right frontal obstacle
    if(sensor_val[13] > 900){
        _mode = TURN_LEFT_SOFT;
    }
    // Avoid crash with bottom obstacle
    if(sensor_val[6] > 900 || sensor_val[9] > 900){
        _mode = FORWARD_SLOW;
    }
    // Kind of corner: when only right and left lateral senson have measured
    if(sensor_val[12]>270 && sensor_val[3]>270){
        _corner = true;
    }
    // Kind of corner: when frontal sensors have a little measured and frontal right and left sensor have a measure greater than 50
    if(sensor_val[2]>50 && sensor_val[14]>50 && sensor_val[0] < 60 && sensor_val [15] < 60){
        _corner = true;
    }

}

//////////////////////////////////////////////

void MyRobot::go_home(double sensor_val[], vector<double> position){

    // If line star is over,  stop if pass two seconds
    if(_start_line == true){

        if(tempo() == true){

            _mode = STOP;
            for(int i = 0; i<_persons.size(); i++){

                cout << "Persona " << i+1 << " encontrada en x: " << _persons[i].getX() << ", z: " << _persons[i].getZ() << endl;
            }
        }
        else{
            _mode = FORWARD;
        }
    }
    // Else goBack to find start line
    else{

        // If final position is correct
        if(_final_pos == true){

            goBack(sensor_val, position);

        }
        // Else, put on correct position
        else{

            final_position(sensor_val,(position.at(2)*180/M_PI));
        }
    }
}

//////////////////////////////////////////////

void MyRobot::final_position(double sensor_val[],double compass_angle)
{
    // If the robot is close one obstacle, back
    if((sensor_val[1] > 100 || sensor_val[14]>100)){

        _mode = BACK;
    }
    // Else, the robot don't have obstacle in front and turn to -135 angle
    else{
        if (compass_angle> -140 && compass_angle < -130){

            _final_pos = true;
        }
        else{

            if(compass_angle> -130 && compass_angle< 45){

                _mode = TURN_AROUND;
            }
            else{

                _mode = TURN_AROUND_RIGHT;
            }
        }
    }
}

