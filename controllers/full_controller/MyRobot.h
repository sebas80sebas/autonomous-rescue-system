#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   Controller for the whole project.
 *
 * @author  Álvaro Cabrera Nieto        <100472152@alumnos.uc3m.es>
 * @author  Iván Sebastián Loor Weir    <100448737@alumnos.uc3m.es>
 * @date    2025
*/

// include dependencies
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>

#include <map>
#include <chrono>
#include <limits>
#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>


using namespace webots;

#define WHEELS_DISTANCE 0.3606 //[=] meters
#define WHEEL_RADIUS 0.0825    //[=] meters

// Structure to represent a movement sequence
struct MovementStep {
    enum Type {
        WAIT,           // Wait for a determined time
        TURN_LEFT,      // Turn left a determined angle (180 - x)º
        TURN_RIGHT,     // Turn right a determined angle (180 - x)º
        MOVE_FORWARD,   // Move forward for a determined time
        STOP            // Stop the robot
    };
    
    Type type;          // Movement type
    double value;       // Associated value (time in seconds or angle in degrees)
};

class MyRobot : public Robot {
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
         * @brief Function with the logic of the controller.
         * @param
         * @return
         */
        void run();

        /**
         * Turns the robot towards a specific orientation
         * @param robot Pointer to Robot object
         * @param compass Pointer to compass sensor
         * @param leftMotor Pointer to left motor
         * @param rightMotor Pointer to right motor
         * @param targetOrientation Target orientation vector [x, y, z]
         * @param tolerance Allowed tolerance to consider orientation reached
         * @param timeStep Time step for simulation
         * @return true when desired orientation has been reached
         */
        bool turnToOrientation(Robot* robot, Compass* compass, Motor* leftMotor, Motor* rightMotor, const double targetOrientation[3], double tolerance, int timeStep);
        
        // Function to detect collisions using distance sensors
        bool isCollisionDetected(DistanceSensor* distance_sensors[]);

        // Function to move the robot forward
        void moveForward(Motor* leftMotor, Motor* rightMotor, double speed);

        // Function to stop the robot
        void stopRobot(Motor* leftMotor, Motor* rightMotor);

        // Function to execute a movement sequence
        bool executeMovementSequence(Robot* robot, Motor* leftMotor, Motor* rightMotor, Compass* compass,
            std::vector<MovementStep>& sequence, int& currentStepIndex, 
            double& stepStartTime, int timeStep);





    private:
        int timeStep; // control time step
        
        // Camera sensors
        Camera *_front_cam;
        Camera *_spher_cam;

        // Motors
        Motor *_leftMotor;
        Motor *_rightMotor;
        /*
        Motor* _left_wheel_motor;
        Motor* _right_wheel_motor;
        */

        // Positioning
        PositionSensor * _left_wheel_sensor;
        PositionSensor * _right_wheel_sensor;

        // Distance
        DistanceSensor*_distance_sensor[16];

        // Compass
        Compass *_compass;

        // GPS
        GPS * _gps;

        // Function to detect scenario based on colors and pixel counts
        std::string detect_scenario(const std::vector<std::tuple<int, int, int>>& colors, 
            const std::vector<int>& pixel_counts);

        // Function to convert degrees to radians    
        inline double degToRad(double degrees);

        // Variables for total distance traveled
        double previous_left_position;  // Previous position of left encoder
        double previous_right_position; // Previous position of right encoder
        double total_distance;          // Total distance traveled in meters
};

#endif
