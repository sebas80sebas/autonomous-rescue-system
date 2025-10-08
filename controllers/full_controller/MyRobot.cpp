/**
 * @file    MyRobot.cpp
 * @brief   Controller for the whole project.
 *
 * @author  Álvaro Cabrera Nieto        <100472152@alumnos.uc3m.es>
 * @author  Iván Sebastián Loor Weir    <100448737@alumnos.uc3m.es>
 * @date    2025
*/

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{
    // get the time step of the current world.
	timeStep = (int)getBasicTimeStep();

	// You should insert a getDevice-like function in order to get the
	// instance of a device of the robot. Something like:
	//  Motor *motor = robot->getMotor("motorname");
	//  DistanceSensor *ds = robot->getDistanceSensor("dsname");
	//  ds->enable(timeStep);

	// initializing and testing motors
	_leftMotor = getMotor("left wheel motor");
	_rightMotor = getMotor("right wheel motor");

	_leftMotor->setPosition(INFINITY);
	_rightMotor->setPosition(INFINITY);

	_leftMotor->setVelocity(3);
	_rightMotor->setVelocity(5);

	// initializing encoders
	_left_wheel_sensor = getPositionSensor("left wheel sensor");
	_right_wheel_sensor = getPositionSensor("right wheel sensor");
	_left_wheel_sensor->enable(timeStep);
	_right_wheel_sensor->enable(timeStep);

	// Initialization of total distance traveled variables
    previous_left_position = 0.0;
    previous_right_position = 0.0;
    total_distance = 0.0;

	
	// Viewed from above, ds0 is frontal, ds1 is to the left of ds0, ds2 is to the left of ds1, ..., ds15 is to the right of ds0
	const char *ds_name[16] = {"ds0", "ds1" ,"ds2", "ds3", "ds4", "ds5", "ds6", "ds7", "ds8", "ds9", "ds10", "ds11", "ds12","ds13", "ds14", "ds15"};
	for (int ind = 0; ind < 16; ind++){
		_distance_sensor[ind] = getDistanceSensor(ds_name[ind]); 
		_distance_sensor[ind]->enable(timeStep);
	}

	// initializing compass
	_compass = getCompass("compass");
	_compass->enable(timeStep);

	// initializing GPS
	_gps = getGPS("gps");
	_gps->enable(timeStep);

	// initializing and testing cameras
	_front_cam = getCamera("camera_f");
	_front_cam->enable(timeStep);
	std::cout << "Front images are " << _front_cam->getWidth() << " x " << _front_cam->getHeight() << std::endl;
	_spher_cam = getCamera("camera_s");
	_spher_cam->enable(timeStep);
	std::cout << "Spherical images are " << _spher_cam->getWidth() << " x " << _spher_cam->getHeight() << std::endl;  

}




//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _leftMotor->setVelocity(0);
	_rightMotor->setVelocity(0);
	_left_wheel_sensor->disable();
	_right_wheel_sensor->disable();  
	for (int ind = 0; ind < 16; ind++) {
		_distance_sensor[ind]->disable();
	}
	_compass->disable();
	_gps->disable();
	_front_cam->disable();
	_spher_cam->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    // //////////////////////////////////////// INITIALIZATION ////////////////////////////////////////

    const double orientationTolerance = 0.05;       // Tolerance to allow some flexibility

    // Define possible robot states (initial behavior)
    enum RobotState {
        TO_NEG_Z,       // Turn towards negative Z axis
        MOVE_TO_CRASH_Z, // Move forward until crash (after orienting towards -Z)
        TO_POS_X,       // Turn towards positive X axis
        MOVE_TO_CRASH_X, // Move forward until crash (after orienting towards +X)
        TO_NEG_X,       // Turn towards negative X axis
        FINISHED        // Final state
    };

    // Global variable to store current state
    RobotState currentState = TO_NEG_Z;

    // Target orientations for each direction
    const double orientationNegZ[3] = {0, 0, -1};  // Towards negative Z axis
    const double orientationPosX[3] = {1, 0, 0};   // Towards positive X axis
    const double orientationNegX[3] = {-1, 0, 0};  // Towards negative X axis

    // Variable to store turn start time
    double turn_start_time = 0;

    // New enum to include scenario detection state
    enum ScenarioState {
        DEFAULT_NAVIGATION,		// Original navigation sequence
        SCENARIO_01,			// Specific handling for scenario 01
        SCENARIO_02,			// Specific handling for scenario 02
        SCENARIO_03,			// Specific handling for scenario 03
        SCENARIO_04,			// Specific handling for scenario 04
        SCENARIO_05,			// Specific handling for scenario 05 (case 0)
        SCENARIO_05_1,			// Specific handling for scenario 05 (case 1)
        SCENARIO_06,			// Specific handling for scenario 06
        SCENARIO_07,			// Specific handling for scenario 07
        SCENARIO_08,			// Specific handling for scenario 08
        SCENARIO_09,			// Specific handling for scenario 09
        SCENARIO_10,			// Specific handling for scenario 10
        DET_01_08_10,			// Detection of Scenarios 01, 08 and 10
        SCENARIO_01_08_10,		// Control of Scenarios 01, 08 and 10
        DET_04_05_06_07,		// Detection of Scenarios 04, 05, 06 and 07
        SCENARIO_04_05_06_07,	// Control of Scenarios 04, 05, 06 and 07
        DET_SCENARIO_05			// Detection of Scenario 05_1
    };

    // Global scenario state variable
    ScenarioState scenarioState = DEFAULT_NAVIGATION;

    /// Declare scenario detection flag outside the loop

    bool isScenario02Detected = false; 
    bool isScenario05Detected = false;
    bool isScenario01y08y10Detected = false; 
    bool isScenario04y05y06y07Detected = false;
    bool isScenario09Detected = false; 

    // Color Scenario 09 
    const std::tuple<int, int, int> SCENARIO_09_COLOR = {210, 250, 250};	
    // Color Scenario 01, 08, 10
    const std::tuple<int, int, int> Color01_08_10 = {40, 110, 110};
    // Color Scenario 04, 05, 06, 07
    const std::tuple<int, int, int> Color04_05_06_07 = {40, 100, 100};
    // Color Scenario 06
    const std::tuple<int, int, int> Color06 = {90, 190, 190};

	// //////////////////////////////////////// MAIN LOOP ////////////////////////////////////////

	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	while (step(timeStep) != -1) {

		// Read the sensors:
		// Enter here functions to read sensor data, like:
		//  double val = ds->getValue();

		// Calculate total distance traveled
		// Calculate distance traveled in this step
        double current_left_position = _left_wheel_sensor->getValue();
        double current_right_position = _right_wheel_sensor->getValue();

		// Calculate angle traveled by each wheel (in radians)
        double delta_left = current_left_position - previous_left_position;
        double delta_right = current_right_position - previous_right_position;

		// Calculate distance traveled by each wheel (radius * angle)
        double distance_left = delta_left * WHEEL_RADIUS;
        double distance_right = delta_right * WHEEL_RADIUS;

		// Average distance traveled in this step
        double distance_step = (distance_left + distance_right) / 2.0;

		// Accumulate in total distance
        total_distance += distance_step;

		// Update previous positions
        previous_left_position = current_left_position;
        previous_right_position = current_right_position;

		std::cout << "Total distance traveled: " << total_distance << " meters" << std::endl;

		// DEBUG: print currentState
		std::cout << "Current state: " << currentState << std::endl;

		// testing compass
		const double * compass_vals = _compass->getValues();
		if (compass_vals != NULL) {
			std::cout << "Compass values: " << compass_vals [0] << "," << compass_vals[1] << "," << compass_vals[2] << std::endl;
		}
		else {
			std::cout << "Compass values are NULL" << std::endl;
		}

		// Check distance sensors for debugging
		for (int ind = 0; ind < 16; ind++) {
			if (_distance_sensor[ind]->getValue() > 80.0) {  // Show only significant readings
				std::cout << "DS" << ind << ": " << _distance_sensor[ind]->getValue() << std::endl;
			}
		}

		// Color tracking variables
		std::map<std::tuple<int, int, int>, int> colorCount;

		// Get the camera image
		const unsigned char* image = _front_cam->getImage();
		int width = _front_cam->getWidth();
		int height = _front_cam->getHeight();

		// Reset color count
		colorCount.clear();

		// Count color occurrences
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				int r = _front_cam->imageGetRed(image, width, x, y);
				int g = _front_cam->imageGetGreen(image, width, x, y);
				int b = _front_cam->imageGetBlue(image, width, x, y);
				
				// Group similar colors (reduce precision to group close colors)
				r = r / 10 * 10;
				g = g / 10 * 10;
				b = b / 10 * 10;
				
				colorCount[{r, g, b}]++;
			}
		}

        // Only detect scenario 09 if it hasn't been detected before
        if (!isScenario09Detected) {
            for (const auto& pair : colorCount) {
                if (pair.first == SCENARIO_09_COLOR) {
                    isScenario09Detected = true;
                    break;
                }
            }
        }
        
        
		// Sort colors by occurrence
		std::vector<std::pair<std::tuple<int, int, int>, int>> sortedColors;
		for (const auto& pair : colorCount) {
			sortedColors.push_back(pair);
		}
		std::sort(sortedColors.begin(), sortedColors.end(), 
			[](const auto& a, const auto& b) { return a.second > b.second; });

		// Print top 5 colors
		std::cout << "Top 5 Colors (R,G,B) [Pixel Count]:" << std::endl;
		for (int i = 0; i < std::min(5, (int)sortedColors.size()); i++) {
			auto color = sortedColors[i].first;
			std::cout << "Color " << i+1 << ": (" 
					<< std::get<0>(color) << "," 
					<< std::get<1>(color) << "," 
					<< std::get<2>(color) << ") [" 
					<< sortedColors[i].second << " pixels]" << std::endl;
		}

        // After sorting colors by occurrence and before the state switch:
  
        // Vectors to store colors and counts for detect_scenario
        std::vector<std::tuple<int, int, int>> dominant_colors;
        std::vector<int> pixel_counts;
        
        // Fill vectors with dominant color data
        for (int i = 0; i < std::min(5, (int)sortedColors.size()); i++) {
            dominant_colors.push_back(sortedColors[i].first);
            pixel_counts.push_back(sortedColors[i].second);
        }
        
        // Detect the scenario
        std::string scenario = detect_scenario(dominant_colors, pixel_counts);
        
        // Indicate corresponding scenario mode if one is detected
        if (scenario != "unknown") {
           
            if (scenario == "04y05y06y07") {
                std::cout << "It's a scenario between 04, 05, 06 and 07" << std::endl;
                
                isScenario04y05y06y07Detected = true;
                
            }
            else if (scenario == "01y08y10") {
                
                std::cout << "It's a scenario between 01, 08 and 10" << std::endl;
                
                isScenario01y08y10Detected = true;
                
            }
            
            else if (scenario == "02") {
                
                std::cout << "It's scenario 2" << std::endl;
                
                isScenario02Detected = true;
                
            }
            
            else if (scenario == "05") {
                
                std::cout << "It's scenario 5" << std::endl;
                
                isScenario05Detected = true;
                
            }

			else if (scenario == "03") {
				scenarioState = SCENARIO_03;
				
				
			}

        }

        // State machine with scenario detection
        switch (scenarioState) {

            case DEFAULT_NAVIGATION:

				// //////////////////////////////////////// PROGRAM START ////////////////////////////////////////
				// At the beginning, the robot must turn to orient towards the negative Z axis
				// Then, move forward quickly until collision
				// After that, the robot must turn to orient towards the positive X axis
				// Next, move forward quickly until collision
				// Finally, the robot must turn to orient towards the negative X axis
				//
				// This is done to begin distinguishing between different scenarios (colors, walls, gaps, etc.)

				// State machine
				switch (currentState) {
					case TO_NEG_Z:
						// Turn towards negative Z axis
						if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationNegZ, orientationTolerance, timeStep)) {
							std::cout << "Orientation towards -Z reached. Moving forward until collision." << std::endl;
							// Change to next state
							currentState = MOVE_TO_CRASH_Z;
							// Stop
							moveForward(_leftMotor, _rightMotor, 0.0);

							// Store turn end time
							turn_start_time = getTime();
						}
						break;
						
					case MOVE_TO_CRASH_Z:

						// Wait 1 second before moving forward - give time for motors and wheels to stabilize
						if (getTime() - turn_start_time < 1) {
							break;
						}

						// Maximum speed for collision
						moveForward(_leftMotor, _rightMotor, 10.0);
						
						// If cube is found
						if ((isCollisionDetected(_distance_sensor)) && (getTime() - turn_start_time > 1) && (isScenario05Detected)) {
							// Collision detected
							std::cout << "Collision detected with Cube" << std::endl;
							// Change to next state
							scenarioState = DET_SCENARIO_05;
							
							
							// Stop the robot
							stopRobot(_leftMotor, _rightMotor);

							// Store collision end time
							turn_start_time = getTime();
							break;
						}

						// Move forward until collision
						if ((isCollisionDetected(_distance_sensor)) && (getTime() - turn_start_time > 1)) {
							// Collision detected
							std::cout << "Collision detected after orienting towards -Z. Turning towards +X." << std::endl;
							// Change to next state
							currentState = TO_POS_X;
							// Stop the robot
							stopRobot(_leftMotor, _rightMotor);

							// Store collision end time
							turn_start_time = getTime();
						}
						break;
						
					case TO_POS_X:
						// Wait 1 second before moving forward
						if (getTime() - turn_start_time < 1) {
							break;
						}

						// Turn towards positive X axis
						if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationPosX, orientationTolerance, timeStep)) {
							std::cout << "Orientation towards +X reached. Moving forward until collision." << std::endl;
							// Change to next state
							currentState = MOVE_TO_CRASH_X;
							// Stop
							moveForward(_leftMotor, _rightMotor, 0.0);

							// Store turn end time
							turn_start_time = getTime();
						}
						break;
						
					case MOVE_TO_CRASH_X:
						// Wait 1 second before moving forward
						if (getTime() - turn_start_time < 1) {
							break;
						}
						
						// If scenario 09 is detected during second collision, switch to scenario mode
						if (isScenario09Detected) {
							std::cout << "Scenario 09 detected! Switching to scenario mode." << std::endl;
							scenarioState = SCENARIO_09;
							stopRobot(_leftMotor, _rightMotor);
							break;
						}
						

						// Maximum speed for collision
						moveForward(_leftMotor, _rightMotor, 10.0);

						// Move forward until collision
						if (isCollisionDetected(_distance_sensor)) {
							// Collision detected
							std::cout << "Collision detected after orienting towards +X. Turning towards -X." << std::endl;
							// Stop the robot
							stopRobot(_leftMotor, _rightMotor);
							// Change to next state
							currentState = TO_NEG_X;

							// Store collision end time
							turn_start_time = getTime();
						}
						break;
						
					case TO_NEG_X:
						// Wait 1 second before moving forward
						if (getTime() - turn_start_time < 1) {
							break;
						}

						// Check for scenario 03 (pixels with RGB 0,0,0)
						for (const auto& pair : colorCount) {
							if (std::get<0>(pair.first) == 0 && std::get<1>(pair.first) == 0 && std::get<2>(pair.first) == 0) {
								if (pair.second > 1000) { // Minimum threshold of black pixels
									std::cout << "Scenario 03 detected! Switching to scenario mode." << std::endl;
									scenarioState = SCENARIO_03;
									stopRobot(_leftMotor, _rightMotor);
									break;
								}
							}
						}

						// Turn towards negative X axis
						if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationNegX, orientationTolerance, timeStep)) {
							std::cout << "Orientation towards -X reached. Task completed." << std::endl;
							// Change to final state
							currentState = FINISHED;
							// Stop the robot
							stopRobot(_leftMotor, _rightMotor);
						}
						
						// If scenario 01_08_10 is detected during third collision, switch to scenario mode
						if (isScenario01y08y10Detected) {
							scenarioState = SCENARIO_01_08_10;
							break;
						}
						
						// If scenario 02 is detected during third collision, switch to scenario mode
						if (isScenario02Detected) {
							
							if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationNegX, orientationTolerance, timeStep)) {
                                      					std::cout << "Orientation towards +Z reached. Task completed." << std::endl;
                                      					moveForward(_leftMotor, _rightMotor, 0.0);
                                      					scenarioState = SCENARIO_02;                                                                                            
                                      					// Store turn end time					
                                      					turn_start_time = getTime();
                        	}
							
							
							break;
						}
						
						// If scenario 04_05_06_07 is detected during third collision, switch to scenario mode
						if (isScenario04y05y06y07Detected) {
							scenarioState = SCENARIO_04_05_06_07;
							break;
						}
						
						break;
						
					case FINISHED:
						// Keep robot stopped
						stopRobot(_leftMotor, _rightMotor);
						break;
				}
                break;

			//////////// Control of Possible Scenarios 01, 08 and 10 ////////////
			case SCENARIO_01_08_10:
                // Specific handling for scenario 01_08_10
                std::cout << "Detecting which scenario: (01, 08 or 10)" << std::endl;

				// Turn robot around so it can verify which scenario it belongs to in next state
				if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationNegX, orientationTolerance, timeStep)) {
					std::cout << "Orientation towards +Z reached. Task completed." << std::endl;

					moveForward(_leftMotor, _rightMotor, 0.0);

					scenarioState = DET_01_08_10;					
					// Store turn end time
					turn_start_time = getTime();

				}			              
                
                break;
			
			//////////// Detection of Possible Scenarios 01, 08 and 10 ////////////	
			case DET_01_08_10:  
                // Specific handling for scenarios 01, 08 and 10
                
                if (getTime() - turn_start_time < 1) {
					break;
				}

				// Move robot forward to try to detect correct scenario
				moveForward(_leftMotor, _rightMotor, 10.0);
				
				// If color Color01_08_10 = (40, 110, 110) is stable and exceeds 14000 pixels  
				// we are in scenario 08, otherwise we need to detect if we are in 
				// scenario 01 or 10
				if (colorCount[Color01_08_10] > 14000) { 
					std::cout << "Scenario 08" << std::endl;
					
					stopRobot(_leftMotor, _rightMotor);
					
					scenarioState = SCENARIO_08;
					
				} else if ((_distance_sensor[0]->getValue() > 900) && (colorCount[Color01_08_10] < 14000)) {
					
					// If DS6 sensor is activated we are in scenario 1, otherwise in scenario 10
					
					if (_distance_sensor[6]->getValue() > 0) {
					
					std::cout << "Scenario 01" << std::endl;
					
					stopRobot(_leftMotor, _rightMotor);
					
					scenarioState = SCENARIO_01;
					
					} else {
					
					std::cout << "Scenario 10" << std::endl;
					
					stopRobot(_leftMotor, _rightMotor);
					
					scenarioState = SCENARIO_10;
					}
							
				}

                break;


			//////////// Control of Possible Scenarios 04, 05, 06 and 07 ////////////
			case SCENARIO_04_05_06_07:
                // Specific handling for scenario 04_05_06_07
                std::cout << "Detecting which scenario: (04, 05, 06 or 07)" << std::endl;

				// Turn robot around so it can verify which scenario it belongs to in next state
				if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationNegX, orientationTolerance, timeStep)) {
					std::cout << "Orientation towards +Z reached. Task completed." << std::endl;

					moveForward(_leftMotor, _rightMotor, 0.0);

					scenarioState = DET_04_05_06_07;					
					turn_start_time = getTime();

				}			              
                
                break;


			//////////// Detection of Possible Scenarios 04, 05, 06 and 07 ////////////
			case DET_04_05_06_07:  
                // Specific handling for scenarios 04, 05, 06 and 07
                
                if (getTime() - turn_start_time < 1) {
					break;
				}

				// Move robot forward to try to detect correct scenario
	     		moveForward(_leftMotor, _rightMotor, 10.0);
				
				// If Color04_05_06_07 is stable and exceeds 38000 pixels, it could be scenario 04 or 05
				if (colorCount[Color04_05_06_07] > 38000) { 
					std::cout << "Scenario 04 or 05" << std::endl;

					// If distance sensor DS2 detects value greater than 740, it's scenario 05
					if (_distance_sensor[2]->getValue() > 740) {
								
						std::cout << "Scenario 05" << std::endl;
								
						stopRobot(_leftMotor, _rightMotor);
						scenarioState = SCENARIO_05;
					
					// If DS2 detects less than 740, it's scenario 04
					} else if (_distance_sensor[2]->getValue() < 740) {
								
						std::cout << "Scenario 04" << std::endl;
								
						stopRobot(_leftMotor, _rightMotor);
						scenarioState = SCENARIO_04;
					}								
				
				// If Color04_05_06_07 doesn't exceed 38000 pixels and scenario 06 color is detected (>14600), 
				// we are initially in scenario 06 or 07	
	     		} else if ((colorCount[Color04_05_06_07] < 38000) && (colorCount[Color06] > 14600)){
					// If moving forward, DS6 sensor doesn't detect obstacle and scenario 06 color exceeds 15367, 
					// it's scenario 06
                    if ((_distance_sensor[6]->getValue() == 0) && (colorCount[Color06] > 15367)) {
						std::cout << "Scenario 06" << std::endl;
						scenarioState = SCENARIO_06;		
						stopRobot(_leftMotor, _rightMotor);

					// If moving forward, scenario 06 color is less than 15050, then it's scenario 07
					} else if (colorCount[Color06] < 15050) {
						std::cout << "Scenario 07" << std::endl;
						scenarioState = SCENARIO_07;				
						stopRobot(_leftMotor, _rightMotor);
					}
	    		}
	     
                break;

			//////////// Control of Scenario 05_1 ////////////
			case DET_SCENARIO_05:  
				// Specific handling for scenario 05_1
                std::cout << "Scenario 05" << std::endl; 

				// Turn robot around to act under scenario 05_1 
                if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationNegX, orientationTolerance, timeStep)) {
					std::cout << "Orientation towards +Z reached. Task completed." << std::endl;
					moveForward(_leftMotor, _rightMotor, 0.0);

					scenarioState = SCENARIO_05_1;					
					turn_start_time = getTime();
	     		}
	     		
				break;


			// //////////////////////////////////////// SCENARIO 01 ////////////////////////////////////////
			case SCENARIO_01:
                // Specific handling for scenario 1
                static std::vector<MovementStep> scenario01Sequence = {
					
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 2.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 107.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 140.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 130.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 1.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 125.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 115.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 9.0},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found first robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 20.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 105.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 10.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 105.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 105.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 0.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 149.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 11.0},
					{MovementStep::WAIT, 0.15}				
					};
				
				static int currentStepIndex_1 = 0;
				static double stepStartTime_1 = -1;
				
				std::cout << "Executing scenario 01 movement sequence. Step: " << currentStepIndex_1 << std::endl;
				
				// Execute movement sequence
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario01Sequence, currentStepIndex_1, stepStartTime_1, timeStep)) {
					// Sequence completed
					std::cout << "Scenario 01 movement sequence completed." << std::endl;
					// Here you could add more logic or change to another state
					stopRobot(_leftMotor, _rightMotor);
				}
				break;


			// //////////////////////////////////////// SCENARIO 02 ////////////////////////////////////////	
			case SCENARIO_02:
                // Specific handling for scenario 02
				if (getTime() - turn_start_time < 1) {
					break;
				}
                
                static std::vector<MovementStep> scenario02Sequence = {					
					{MovementStep::TURN_RIGHT, 160.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 9.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 150.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 140.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 1.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 130.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 115.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 10.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 6.0},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found first robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15}, 

					{MovementStep::TURN_RIGHT, 90.0},	// Found second robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 20.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 9.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 10.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 110.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.1},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 137.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 11.5},
					{MovementStep::WAIT, 0.15}					
				};
                static int currentStepIndex_2 = 0;
	     		static double stepStartTime_2 = -1;
				
      	     	std::cout << "Executing scenario 02 movement sequence. Step: " << currentStepIndex_2 << std::endl;
				
				// Execute movement sequence
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario02Sequence, currentStepIndex_2, stepStartTime_2, timeStep)) {
					// Sequence completed
					std::cout << "Scenario 2 movement sequence completed." << std::endl;
					// Here you could add more logic or change to another state
					stopRobot(_leftMotor, _rightMotor);
				}	
	     
                break;


			// //////////////////////////////////////// SCENARIO 03 ////////////////////////////////////////
			case SCENARIO_03:
				// Specific handling for scenario 03
				// MovementStep::WAIT - Wait x seconds; helps stabilize wheels and motors
				// MovementStep::MOVE_FORWARD - Move forward x seconds
				// MovementStep::TURN_LEFT - Turn left (180 - x) degrees
				// MovementStep::TURN_RIGHT - Turn right (180 - x) degrees
                static std::vector<MovementStep> scenario03Sequence = {
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 110.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.7},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 120.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.2},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 107.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 2.7},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 96.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 7.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 2.1},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 6.1},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_LEFT, 90.0},		// Encounter first robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_LEFT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_LEFT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_LEFT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 80.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 6.0},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_LEFT, 90.0},		// Encounter second robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_LEFT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_LEFT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_LEFT, 104.0},		// Smaller angle because it slips - by default does more than 360°
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 2.3},		// Return to starting line
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 95.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 10.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 94.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.1},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 123.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 6.2},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 125.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.5},
					{MovementStep::WAIT, 0.15}
				};
				
				static int currentStepIndex_3 = 0;
				static double stepStartTime_3 = -1;
				
				std::cout << "Executing scenario 03 movement sequence. Step: " << currentStepIndex_3 << std::endl;
				
				// Execute movement sequence
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
											scenario03Sequence, currentStepIndex_3, stepStartTime_3, timeStep)) {
					// Sequence completed
					std::cout << "Scenario 03 movement sequence completed." << std::endl;
					// Here you could add more logic or change to another state
					stopRobot(_leftMotor, _rightMotor);
				}
				break;
			
			// //////////////////////////////////////// SCENARIO 04 ////////////////////////////////////////
			case SCENARIO_04:
                // Specific handling for scenario 04
                static std::vector<MovementStep> scenario04Sequence = {
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 2.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 107.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 140.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 130.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 1.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 155.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 7.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 125.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 6.0},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found first robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 30.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 1.5},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found second robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_LEFT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 2.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 125.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 2.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 110.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 8.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 130.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 7.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 160.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::MOVE_FORWARD, 3.5},
					{MovementStep::WAIT, 0.15}				
				};
				
				static int currentStepIndex_4 = 0;
				static double stepStartTime_4 = -1;
				
				std::cout << "Executing scenario 04 movement sequence. Step: " << currentStepIndex_4 << std::endl;
				
				// Execute movement sequence
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario04Sequence, currentStepIndex_4, stepStartTime_4, timeStep)) {
					// Sequence completed
					std::cout << "Scenario 04 movement sequence completed." << std::endl;
					// Here you could add more logic or change to another state
					stopRobot(_leftMotor, _rightMotor);
				}

				break;


			// //////////////////////////////////////// SCENARIO 05 ////////////////////////////////////////
			case SCENARIO_05:
                // Specific handling for scenario 05
                static std::vector<MovementStep> scenario05Sequence = {
					
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 7.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 150.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 95.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 125.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 9.0},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found first robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 85.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 7.5},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found second robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 10.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 8.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 100.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 9.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 78.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 6.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 178.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::MOVE_FORWARD, 5.0},
					{MovementStep::WAIT, 0.15}				
				};
				
				static int currentStepIndex_5 = 0;
				static double stepStartTime_5 = -1;
				
				std::cout << "Executing scenario 05 movement sequence. Step: " << currentStepIndex_5 << std::endl;
				
				// Execute movement sequence
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario05Sequence, currentStepIndex_5, stepStartTime_5, timeStep)) {
					// Sequence completed
					std::cout << "Scenario 05 movement sequence completed." << std::endl;
					// Here you could add more logic or change to another state
					stopRobot(_leftMotor, _rightMotor);
				}
				break;  

			// //////////////////////////////////////// SCENARIO 05_1 ////////////////////////////////////////
			case SCENARIO_05_1:
                // Specific handling for scenario 05_1
                static std::vector<MovementStep> scenario051Sequence = {
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 6.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 160.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 100.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 120.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 9.0},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found first robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 85.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 7.5},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found second robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 10.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 8.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 95.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 9.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 75.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 6.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 175.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::MOVE_FORWARD, 5.0},
					{MovementStep::WAIT, 0.15}				
				};
				
				static int currentStepIndex_51 = 0;
				static double stepStartTime_51 = -1;
				
				std::cout << "Executing scenario 05_1 movement sequence. Step: " << currentStepIndex_51 << std::endl;
				
				// Execute movement sequence
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario051Sequence, currentStepIndex_51, stepStartTime_51, timeStep)) {
					// Sequence completed
					std::cout << "Scenario 05_1 movement sequence completed." << std::endl;
					// Here you could add more logic or change to another state
					stopRobot(_leftMotor, _rightMotor);
				}
				break;


			// //////////////////////////////////////// SCENARIO 06 ////////////////////////////////////////
			case SCENARIO_06:
                // Specific handling for scenario 06                
                static std::vector<MovementStep> scenario06Sequence = {					
					{MovementStep::TURN_RIGHT, 110.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 7.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 110.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 160.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 100.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 120.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 9.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 150.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 1.0},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found first robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 45.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.0},
					{MovementStep::WAIT, 0.15}, 

					{MovementStep::TURN_RIGHT, 90.0},	// Found second robot; make complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 20.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::MOVE_FORWARD, 4.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 112.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 8.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 120.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 105.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 160.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.0},
					{MovementStep::WAIT, 0.15}
				};
				static int currentStepIndex_6 = 0;
				static double stepStartTime_6 = -1;
				
				std::cout << "Executing scenario 06 movement sequence. Step: " << currentStepIndex_6 << std::endl;
					
				// Ejecutar la secuencia de movimientos
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario06Sequence, currentStepIndex_6, stepStartTime_6, timeStep)) {
					// Secuencia completada
					std::cout << "Scenario 06 movement sequence completed." << std::endl;
					// Aquí podrías agregar más lógica o cambiar a otro estado
					stopRobot(_leftMotor, _rightMotor);
				}
			
				break;
				
			// //////////////////////////////////////// ESCENARIO 07 ////////////////////////////////////////
			case SCENARIO_07:
                // Specific handling for scenario 07
                static std::vector<MovementStep> scenario07Sequence = {					
					{MovementStep::MOVE_FORWARD, 12.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 125.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 125.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.5},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found the first robot; make a complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 110.0},	
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 6.0},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found the second robot; make a complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 20.0}, 
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 115.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 100.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 85.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 15.0},
          			{MovementStep::WAIT, 0.15}         
				};
				static int currentStepIndex_7 = 0;
				static double stepStartTime_7 = -1;
					
				std::cout << "Executing scenario 07 movement sequence. Step: " << currentStepIndex_7 << std::endl;
					
				// Execute the movement sequence
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario07Sequence, currentStepIndex_7, stepStartTime_7, timeStep)) {
					// Sequence completed
					std::cout << "Scenario 07 movement sequence completed." << std::endl;
					// Here you could add more logic or change to another state
					stopRobot(_leftMotor, _rightMotor);
				}
			
				break; 


			// //////////////////////////////////////// SCENARIO 08 ////////////////////////////////////////
	 		case SCENARIO_08:
                // Specific handling for scenario 08
				// MovementStep::WAIT - Wait x seconds; helps stabilize wheels and motors
				// MovementStep::MOVE_FORWARD - Move forward x seconds
				// MovementStep::TURN_LEFT - Turn left (180 - x) degrees
				// MovementStep::TURN_RIGHT - Turn right (180 - x) degrees
                static std::vector<MovementStep> scenario08Sequence = {
					{MovementStep::MOVE_FORWARD, 5.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 120.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 6.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 165.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 135.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 9.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 150.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 1.0},
					{MovementStep::WAIT, 0.15},
					
					{MovementStep::TURN_RIGHT, 90.0},	// Found the first robot; make a complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 60.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 1.0},
					{MovementStep::WAIT, 0.15},
					
					{MovementStep::TURN_RIGHT, 90.0},	// Found the second robot; make a complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 110.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 10.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 130.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 7.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 135.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 6.5},
					{MovementStep::WAIT, 0.15}
				};
				
				static int currentStepIndex_8 = 0;
				static double stepStartTime_8 = -1;
				
				std::cout << "Executing scenario 08 movement sequence. Step: " << currentStepIndex_8 << std::endl;
				
				// Execute the movement sequence
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario08Sequence, currentStepIndex_8, stepStartTime_8, timeStep)) {
					// Sequence completed
					std::cout << "Scenario 08 movement sequence completed." << std::endl;
					// Here you could add more logic or change to another state
					stopRobot(_leftMotor, _rightMotor);
				}
				break;





			// //////////////////////////////////////// SCENARIO 09 ////////////////////////////////////////
            case SCENARIO_09:
                // Specific handling for scenario 09
				// MovementStep::WAIT - Wait x seconds; helps stabilize wheels and motors
				// MovementStep::MOVE_FORWARD - Move forward x seconds
				// MovementStep::TURN_LEFT - Turn left (180 - x) degrees
				// MovementStep::TURN_RIGHT - Turn right (180 - x) degrees
                static std::vector<MovementStep> scenario09Sequence = {
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 47.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 155.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 8.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 145.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 117.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 7.5},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found the first robot; make a complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 32.0},	
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 7.5},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found the second robot; make a complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_LEFT, 30.0},	
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 112.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 145.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 140.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 12.0}	
				};
				
				static int currentStepIndex_9 = 0;
				static double stepStartTime_9 = -1;
				
				std::cout << "Executing scenario 09 movement sequence. Step: " << currentStepIndex_9 << std::endl;
				
				// Execute the movement sequence
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
											scenario09Sequence, currentStepIndex_9, stepStartTime_9, timeStep)) {
					// Sequence completed
					std::cout << "Scenario 09 movement sequence completed." << std::endl;
					// Here you could add more logic or change to another state
					stopRobot(_leftMotor, _rightMotor);
				}
				break;

			// //////////////////////////////////////// SCENARIO 10 ////////////////////////////////////////
            case SCENARIO_10:
                // Specific handling for scenario 10
                static std::vector<MovementStep> scenario10Sequence = {
					
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 7.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 60.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 160.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::MOVE_FORWARD, 2.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 70.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 5.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 120.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 10.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 120.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found the first robot; make a complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
  					{MovementStep::TURN_RIGHT, 140.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 2.0},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Found the second robot; make a complete turn
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 170.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 120.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 9.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 95.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.3},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 55.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 6.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 170.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::MOVE_FORWARD, 1.5},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 125.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 1.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_RIGHT, 120.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15}				
					};
				
				static int currentStepIndex_10 = 0;
				static double stepStartTime_10 = -1;
				
				std::cout << "Executing scenario 10 movement sequence. Step: " << currentStepIndex_10 << std::endl;
				
				// Execute the movement sequence
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario10Sequence, currentStepIndex_10, stepStartTime_10, timeStep)) {
					// Sequence completed
					std::cout << "Scenario 10 movement sequence completed." << std::endl;
					// Here you could add more logic or change to another state
					stopRobot(_leftMotor, _rightMotor);
				}
				break;
        }
	};

}

// Function to convert degrees to radians
inline double MyRobot::degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}


// Function to execute a movement sequence
bool MyRobot::executeMovementSequence(Robot* robot, Motor* leftMotor, Motor* rightMotor, Compass* compass,
                             std::vector<MovementStep>& sequence, int& currentStepIndex, 
                             double& stepStartTime, int timeStep) {
    
    // If we have completed the sequence
    if (currentStepIndex >= sequence.size()) {
        return true;
    }
    
    // Get the current step
    MovementStep& currentStep = sequence[currentStepIndex];
    
    // If this is the start of the current step, save the time
    if (stepStartTime < 0) {
        stepStartTime = robot->getTime();
        
        // Configure the movement according to the type
        switch (currentStep.type) {
            case MovementStep::WAIT:
            case MovementStep::STOP:
                // Stop the robot
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(0);
                break;
                
            case MovementStep::MOVE_FORWARD:
                // Move forward
                leftMotor->setVelocity(10.0);  // Adjustable speed
                rightMotor->setVelocity(10.0);
                break;
                
            case MovementStep::TURN_LEFT:
            case MovementStep::TURN_RIGHT:
                // Turn logic is handled in the following case
                break;
        }
    }
    
    // Specific logic for each step type
    switch (currentStep.type) {
        case MovementStep::WAIT:
        case MovementStep::STOP:
        case MovementStep::MOVE_FORWARD:
            // For wait, stop, or forward movement, simply wait for the specified time
            if (robot->getTime() - stepStartTime >= currentStep.value) {
                // Move to the next step
                currentStepIndex++;
                stepStartTime = -1;
            }
            break;
            
        case MovementStep::TURN_LEFT:
        case MovementStep::TURN_RIGHT: {
            // For turns, we need to calculate the target orientation
            
            // Get the current orientation
            const double* compassValues = compass->getValues();
            
            // Calculate the current angle (in the XZ plane)
            double currentAngle = atan2(compassValues[0], compassValues[2]);
            
            // Calculate the target angle
            double targetAngleChange = degToRad(currentStep.value);
            if (currentStep.type == MovementStep::TURN_RIGHT) {
                targetAngleChange = -targetAngleChange;
            }
            
            // If this is the start of the turn, save the initial angle and calculate the target
            static double initialAngle = 0;
            static double targetAngle = 0;
            
            if (robot->getTime() - stepStartTime < timeStep/1000.0) {
                initialAngle = currentAngle;
                targetAngle = initialAngle + targetAngleChange;
                // Normalize the target angle between -PI and PI
                while (targetAngle > M_PI) targetAngle -= 2*M_PI;
                while (targetAngle < -M_PI) targetAngle += 2*M_PI;
            }
            
            // Calculate difference between current angle and target
            double angleDiff = targetAngle - currentAngle;
			double angleDiff2 = targetAngle - currentAngle - M_PI;
			double angleDiff3 = targetAngle - currentAngle + M_PI;
            // Normalize the difference between -PI and PI
            while (angleDiff > M_PI) angleDiff -= 2*M_PI;
            while (angleDiff < -M_PI) angleDiff += 2*M_PI;
			while (angleDiff2 > M_PI) angleDiff2 -= 2*M_PI;
			while (angleDiff2 < -M_PI) angleDiff2 += 2*M_PI;
			while (angleDiff3 > M_PI) angleDiff3 -= 2*M_PI;
			while (angleDiff3 < -M_PI) angleDiff3 += 2*M_PI;

			// DEBUG: print current and target angle
			std::cout << "Current angle: " << currentAngle << ", Target angle: " << targetAngle << std::endl;
			// When the current angle and target are equal, this actually outputs: [full_controller] Current angle: -1.0973, Target angle: 2.09729
			// To fix this, angleDiff should be the subtraction of the two angles or the subtraction of one - PI and the other or the subtraction of one and the other - PI
			
            
            // Turn speed proportional to angle difference
            double turnSpeed = 2.0;  // Adjustable base speed
            
            if ((fabs(angleDiff) < degToRad(2.0)) || (fabs(angleDiff2) < degToRad(2.0)) || (fabs(angleDiff3) < degToRad(2.0))) {
                // If we're close to the target, stop and move to the next step
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(0);
                currentStepIndex++;
                stepStartTime = -1;
            } else if (angleDiff > 0) {
                // Turn left
                leftMotor->setVelocity(-turnSpeed);
                rightMotor->setVelocity(turnSpeed);
            } else {
                // Turn right
                leftMotor->setVelocity(turnSpeed);
                rightMotor->setVelocity(-turnSpeed);
            }
            break;
        }
    }
    
    return false;  // The sequence has not been completed yet
}

// Function to stop the robot
void MyRobot::stopRobot(Motor* leftMotor, Motor* rightMotor) {
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
}

// Function to move the robot forward
void MyRobot::moveForward(Motor* leftMotor, Motor* rightMotor, double speed) {
	// DEBUG: print speed
	std::cout << "Moving forward at " << speed << " m/s" << std::endl;

    leftMotor->setVelocity(speed);
    rightMotor->setVelocity(speed);
}

// Function to detect collisions using distance sensors
bool MyRobot::isCollisionDetected(DistanceSensor* distance_sensors[]) {
	// Threshold to detect collisions (adjust as needed)
	const double COLLISION_THRESHOLD = 100.0;

	// Check the front sensors (ds0, ds15, ds1)
	// Indices may need adjustments depending on the exact configuration
	if (distance_sensors[0]->getValue() > COLLISION_THRESHOLD
		|| distance_sensors[15]->getValue() > COLLISION_THRESHOLD 
		|| distance_sensors[1]->getValue() > COLLISION_THRESHOLD) {
		return true;
	}

	return false;
}

/**
 * Turns the robot to a specific orientation
 * @param robot Pointer to the Robot object
 * @param compass Pointer to the compass sensor
 * @param leftMotor Pointer to the left motor
 * @param rightMotor Pointer to the right motor
 * @param targetOrientation Target orientation vector [x, y, z]
 * @param tolerance Allowed tolerance to consider the orientation has been reached
 * @param timeStep Time step for the simulation
 * @return true when the desired orientation has been reached
 */
bool MyRobot::turnToOrientation(Robot* robot, Compass* compass, Motor* leftMotor, Motor* rightMotor, const double targetOrientation[3], double tolerance, int timeStep) {
	// Turn speeds
	const double TURN_SPEED = 1.5;

	// Get the current compass values
	const double* compassValues = compass->getValues();

	if (compassValues == NULL) {
		std::cout << "Error: Cannot read compass values" << std::endl;
		return false;
	}

	// Calculate the dot product between the current and desired orientation
	double dotProduct = compassValues[0] * targetOrientation[0] + compassValues[1] * targetOrientation[1] + compassValues[2] * targetOrientation[2];

	// Normalize the vectors
	double currentMagnitude = sqrt(compassValues[0]*compassValues[0] + compassValues[1]*compassValues[1] + compassValues[2]*compassValues[2]);

	double targetMagnitude = sqrt(targetOrientation[0]*targetOrientation[0] + targetOrientation[1]*targetOrientation[1] + targetOrientation[2]*targetOrientation[2]);

	// Calculate the cosine of the angle between the vectors
	double cosAngle = dotProduct / (currentMagnitude * targetMagnitude);

	// Ensure cosAngle is in the range [-1, 1]
	if (cosAngle > 1.0) cosAngle = 1.0;
	if (cosAngle < -1.0) cosAngle = -1.0;

	// Calculate the angle in radians
	double angle = acos(cosAngle);

	// Check if we are already in the correct orientation (within tolerance)
	if (angle < tolerance) {
		// Stop the motors
		leftMotor->setVelocity(0);
		rightMotor->setVelocity(0);
		return true;
	}

	// Determine the turn direction (right or left)
	// For this, we use the cross product to determine if we should turn clockwise or counterclockwise
	double crossProduct[3];
	crossProduct[0] = compassValues[1] * targetOrientation[2] - compassValues[2] * targetOrientation[1];
	crossProduct[1] = compassValues[2] * targetOrientation[0] - compassValues[0] * targetOrientation[2];
	crossProduct[2] = compassValues[0] * targetOrientation[1] - compassValues[1] * targetOrientation[0];

	// The sign of the Y component of the cross product indicates the turn direction
	bool turnRight = (crossProduct[1] > 0);

	// Set velocities for the turn
	if (turnRight) {
		// Turn right
		std::cout << "Turning right" << std::endl;
		leftMotor->setVelocity(TURN_SPEED);
		rightMotor->setVelocity(-TURN_SPEED);
	} else {
		// Turn left
		std::cout << "Turning left" << std::endl;
		leftMotor->setVelocity(-TURN_SPEED);
		rightMotor->setVelocity(TURN_SPEED);
	}

	// We have not yet reached the desired orientation
	return false;
}

// Function to detect the scenario based on colors and pixel counts
std::string MyRobot::detect_scenario(const std::vector<std::tuple<int, int, int>>& colors, 
                           const std::vector<int>& pixel_counts) {
    // If there are no colors, we cannot detect anything
    if (colors.empty()) {
        return "unknown";
    }
    
    // Get components of the first dominant color
    int r0 = std::get<0>(colors[0]);
    int g0 = std::get<1>(colors[0]);
    int b0 = std::get<2>(colors[0]);
    
    // Scenarios 1, 8 and 10 have similar colors
    if (r0 == 50 && g0 == 120 && b0 == 120 && colors.size() > 1) {
        int r1 = std::get<0>(colors[1]);
        int g1 = std::get<1>(colors[1]);
        int b1 = std::get<2>(colors[1]);
        
        if (r1 == 40 && g1 == 100 && b1 == 100) {
            return "01y08y10";
        }
    }

	// Scenario 3 (with multiple colors in specific shades and black)
	if ((r0 == 70 && g0 == 160 && b0 == 160) || (r0 == 60 && g0 == 140 && b0 == 140)) {
		// Check if black color exists in the first 5 colors
		for (int i = 0; i < std::min(5, (int)colors.size()); i++) {
			int r = std::get<0>(colors[i]);
			int g = std::get<1>(colors[i]);
			int b = std::get<2>(colors[i]);
			
			if (r == 0 && g == 0 && b == 0) {
				return "03";
			}
		}
	}

    
    // For scenarios 4, 5 (case 0), 6 and 7 with dominant color (80,190,190)
    else if (r0 == 80 && g0 == 190 && b0 == 190) {
        return "04y05y06y07";
    }	

	// For scenario 5 (case 1) with dominant color (70,170,170)
    else if (r0 == 70 && g0 == 170 && b0 == 170) {
        
        return "05";
    }
    
    // For scenario 2 with dominant color (80,180,180)
    else if (r0 == 80 && g0 == 180 && b0 == 180 && colors.size() == 1) {
        return "02";
    }
    
    // If it doesn't match any known pattern
    return "unknown";
}
//////////////////////////////////////////////				
					