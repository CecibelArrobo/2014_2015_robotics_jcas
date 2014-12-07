#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   The robot go across the world, search two people trapped and
 *          go back to start line.
 *
 * @author  Jessica Cecibel Arrobo Sarango <100283869@alumnos.uc3m.es>
 * @author  Marcos Paulo Nascimento Gouveia <1100329047@alumnos.uc3m.es>
 * @date    2014-12
 */

#include <iostream>
#include <cmath>
#include <vector>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 16
#define DISTANCE_LIMIT      100
#define MAX_SPEED           70

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
      * @brief Go ahead avoid crash with distance measured by sensors
      * @param sensor_val[] sensor values array
      * @param position actual robot position
      */
    void goAhead(double sensor_val[], vector<double> position);

    /**
      * @brief Go back avoid crash with distance measured by sensors
      * @param sensor_val[] sensor values array
      * @param position actual robot position
      */
    void goBack(double sensor_val[], vector<double> position);

    /**
      * @brief Send actuators commands according to the mode
      */
    void mode_selection();

    /**
      * @brief Convert the compass angle to degrees
      * @param in_vector measured provide by compass
      * @return angle in degrees
      */
    double convert_bearing_to_degrees(const double* in_vector);

    /**
      * @brief Convert a angle into the range [0,180] and [-180,0]
      * @param angle angle to convert
      * @return converted angle
      */
    double normal_angle(double angle);

    /**
      * @brief Detected some colors in forward or spherical camera
      * @param camera allot select the camera image to detect color
      * 0:forward, 1: spherical
      * @param color the color that wanted dectect
      * 0: white, 1: green, 2: yellow
      * @return color_percentage the color percentage in the camera image
      */
    double color_detector(int camera, int color);

    /**
      * @brief Know the robot differential position
      * @param prev_position previous robot position
      * @return actual_position the new calculate position
      */
    vector <double> odometry(vector <double> prev_position);

    /**
      * @brief Localizate a person with spherical camera
      * @return direction to move
      */
    int local_person();

    /**
      * @brief Turn to a specific angle depend of actual angle
      * @param desired_angle angle objetive
      * @param theta actual angle
      */
    void turn_to_desired_angle(int desired_angle, double theta);

    /**
      * @brief Move the robot to initial position and active the flag _init_pos
      * @param sensor_val[] sensor values array
      * @param compass_angle actual angle
      */
    void initial_position(double sensor_val[], double compass_angle);

    /**
      * @brief Move the robot to final position and active the flag _final_pos
      * @param sensor_val[] sensor values array
      * @param compass_angle actual angle
      */
    void final_position(double sensor_val[], double compass_angle);

    /**
      * @brief Search person using local_person()
      * @param sensor_val[] sensor values array
      * @param position actual robot position
      */
    void person_find(double sensor_val[],vector<double> position);

    /**
      * @brief Detected if cross finish line and start line
      * @param position actual robot position
      */
    void cross_line(vector <double> position);

    /**
      * @brief Identifie if person detecte is a diferent person
      * @param position actual robot position
      * @return true: different person, false: same person
      */
    bool different_person(vector <double> position);

    /**
      * @brief Two second timer
      * @return true: when time is over
      */
    bool tempo();

    /**
      * @brief Avoid catastrofic crash
      * @param sensor_val[] sensor values array
      */
    void avoid_crash( double sensor_val[]);

    /**
      * @brief Return to initial zone
      * @param sensor_val[] sensor values array
      * @param position actual robot position
      */
    void go_home(double sensor_val[],vector<double> position);

    
private:
    int _time_step;



    // Velocoties
    double _left_speed, _right_speed;

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
        FORWARD_SLOW,
        TURN_LEFT_SOFT,
        TURN_RIGHT_SOFT,
        TURN_AROUND_RIGHT
    };

    Mode _mode;

    struct Person{

        int id;
        double P_x, P_z;

        Person(){ id = 0; P_x= 0; P_z = 0;}

        int getId(){ return id;}
        double getX(){ return P_x;}
        double getZ(){return P_z;}
        void setId(int n){ id = n;}
        void setX(double x){P_x = x;}
        void setZ(double z){P_z = z;}

    };

    vector <Person> _persons;
    int _person_found;
    bool _finish_line;
    bool _start_line;
    bool _person_detected;
    bool _different_person;
    int _time;
    bool _corner;
    bool _init_pos;
    bool _final_pos;
    int _person_angle;
    Person _P;

};
#endif
