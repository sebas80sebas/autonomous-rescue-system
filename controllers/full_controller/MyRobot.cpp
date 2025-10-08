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

	// Inicialización de las variables de distancia total recorrida
    previous_left_position = 0.0;
    previous_right_position = 0.0;
    total_distance = 0.0;

	
	// Visto desde arriba, ds0 es el frontal, ds1 está a la izda. de ds0, ds2 está a la izda. de ds1, ..., ds15 está a la dcha. de ds0
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
    // //////////////////////////////////////// INICIALIZACIÓN ////////////////////////////////////////

    const double orientationTolerance = 0.05;       // Tolerancia para permitir cierta flexibilidad

    // Definir los posibles estados del robot (comportamiento inicial)
    enum RobotState {
        TO_NEG_Z,       // Girar hacia el eje negativo Z
        MOVE_TO_CRASH_Z, // Avanzar hasta chocar (después de orientarse hacia -Z)
        TO_POS_X,       // Girar hacia el eje positivo X
        MOVE_TO_CRASH_X, // Avanzar hasta chocar (después de orientarse hacia +X)
        TO_NEG_X,       // Girar hacia el eje negativo X
        FINISHED        // Estado final
    };

    // Variable global para almacenar el estado actual
    RobotState currentState = TO_NEG_Z;

    // Orientaciones objetivo para cada dirección
    const double orientationNegZ[3] = {0, 0, -1};  // Hacia el eje negativo Z
    const double orientationPosX[3] = {1, 0, 0};   // Hacia el eje positivo X
    const double orientationNegX[3] = {-1, 0, 0};  // Hacia el eje negativo X

    // Variable para almacenar el tiempo de inicio del giro
    double turn_start_time = 0;

    // New enum to include scenario detection state
    enum ScenarioState {
        DEFAULT_NAVIGATION,		// Original navigation sequence
        SCENARIO_01,			// Specific handling for scenario 01
        SCENARIO_02,			// Specific handling for scenario 02
        SCENARIO_03,			// Specific handling for scenario 03
        SCENARIO_04,			// Specific handling for scenario 04
        SCENARIO_05,			// Specific handling for scenario 05 (caso 0)
        SCENARIO_05_1,			// Specific handling for scenario 05 (caso 1)
        SCENARIO_06,			// Specific handling for scenario 06
        SCENARIO_07,			// Specific handling for scenario 07
        SCENARIO_08,			// Specific handling for scenario 08
        SCENARIO_09,			// Specific handling for scenario 09
        SCENARIO_10,			// Specific handling for scenario 10
        DET_01_08_10,			// Deteccion de Escenarios 01, 08 y 10
        SCENARIO_01_08_10,		// Control de Escenarios 01, 08 y 10
        DET_04_05_06_07,		// Deteccion de Escenarios 04, 05, 06 y 07
        SCENARIO_04_05_06_07,	// Control de Escenarios 04, 05, 06 y 07
        DET_SCENARIO_05			// Deteccion de Escenarios 05_1
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
    // Color Scenario 01, 08 , 10
    const std::tuple<int, int, int> Color01_08_10 = {40, 110, 110};
    // Color Scenario 04, 05, 06, 07
    const std::tuple<int, int, int> Color04_05_06_07 = {40, 100, 100};
    // Color Scenario 06
    const std::tuple<int, int, int> Color06 = {90, 190, 190};

	// //////////////////////////////////////// BUCLE PRINCIPAL ////////////////////////////////////////

	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	while (step(timeStep) != -1) {

		// Read the sensors:
		// Enter here functions to read sensor data, like:
		//  double val = ds->getValue();

		// Calcular la distancia total recorrida
		// Calcular distancia recorrida en este paso
        double current_left_position = _left_wheel_sensor->getValue();
        double current_right_position = _right_wheel_sensor->getValue();

		// Calcular el ángulo recorrido por cada rueda (en radianes)
        double delta_left = current_left_position - previous_left_position;
        double delta_right = current_right_position - previous_right_position;

		// Calcular la distancia recorrida por cada rueda (radio * ángulo)
        double distance_left = delta_left * WHEEL_RADIUS;
        double distance_right = delta_right * WHEEL_RADIUS;

		// Distancia promedio recorrida en este paso
        double distance_step = (distance_left + distance_right) / 2.0;

		// Acumular en la distancia total
        total_distance += distance_step;

		// Actualizar posiciones anteriores
        previous_left_position = current_left_position;
        previous_right_position = current_right_position;

		std::cout << "Distancia total recorrida: " << total_distance << " metros" << std::endl;

		// DEBUG: imprimir currentState
		std::cout << "Current state: " << currentState << std::endl;

		// testing compass
		const double * compass_vals = _compass->getValues();
		if (compass_vals != NULL) {
			std::cout << "Compass values: " << compass_vals [0] << "," << compass_vals[1] << "," << compass_vals[2] << std::endl;
		}
		else {
			std::cout << "Compass values are NULL" << std::endl;
		}

		// Comprobar sensores de distancia para depuración
		for (int ind = 0; ind < 16; ind++) {
			if (_distance_sensor[ind]->getValue() > 80.0) {  // Mostrar solo lecturas significativas
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

        // Después de ordenar los colores por ocurrencia y antes del switch de estado:
  
        // Vectores para almacenar los colores y conteos para detect_scenario
        std::vector<std::tuple<int, int, int>> dominant_colors;
        std::vector<int> pixel_counts;
        
        // Llenar los vectores con los datos de los colores dominantes
        for (int i = 0; i < std::min(5, (int)sortedColors.size()); i++) {
            dominant_colors.push_back(sortedColors[i].first);
            pixel_counts.push_back(sortedColors[i].second);
        }
        
        // Detectar el escenario
        std::string scenario = detect_scenario(dominant_colors, pixel_counts);
        
        // Indicar el modo de escenario correspondiente si se detecta uno
        if (scenario != "unknown") {
           
            if (scenario == "04y05y06y07") {
                std::cout << "Es un escenario entre el 04, el 05, el 06 y el 07" << std::endl;
                
                isScenario04y05y06y07Detected = true;
                
            }
            else if (scenario == "01y08y10") {
                
                std::cout << "Es un escenario entre el 01, el 08 y el 10" << std::endl;
                
                isScenario01y08y10Detected = true;
                
            }
            
            else if (scenario == "02") {
                
                std::cout << "Es el escenario 2" << std::endl;
                
                isScenario02Detected = true;
                
            }
            
            else if (scenario == "05") {
                
                std::cout << "Es el escenario 5" << std::endl;
                
                isScenario05Detected = true;
                
            }

			else if (scenario == "03") {
				scenarioState = SCENARIO_03;
				
				
			}

        }

        // State machine with scenario detection
        switch (scenarioState) {

            case DEFAULT_NAVIGATION:

				// //////////////////////////////////////// COMIENZO DEL PROGRAMA ////////////////////////////////////////
				// Al principio, el robot debe girar para orientarse hacia el eje negativo Z
				// Después, avanzar rápidamente hasta chocarse
				// Tras ello, el robot debe girar para orientarse hacia el eje positivo X
				// A continuación, avanzar rápidamente hasta chocarse
				// Finalmente, el robot debe girar para orientarse hacia el eje negativo X
				//
				// Esto se hace para comenzar a distinguir unos escenarios de otros (colores, muros, huecos, etc.)

				// Máquina de estados
				switch (currentState) {
					case TO_NEG_Z:
						// Girar hacia el eje negativo Z
						if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationNegZ, orientationTolerance, timeStep)) {
							std::cout << "Orientación hacia -Z alcanzada. Avanzando hasta colisión." << std::endl;
							// Cambiar al siguiente estado
							currentState = MOVE_TO_CRASH_Z;
							// Parar
							moveForward(_leftMotor, _rightMotor, 0.0);

							// Almacenar el tiempo de final del giro
							turn_start_time = getTime();
						}
						break;
						
					case MOVE_TO_CRASH_Z:

						// Esperar 1 segundo antes de avanzar - damos tiempo a los motores y a las ruedas a estabilizarse
						if (getTime() - turn_start_time < 1) {
							break;
						}

						// Velocidad máxima para colisión
						moveForward(_leftMotor, _rightMotor, 10.0);
						
						// Si encuentra el cubo
						if ((isCollisionDetected(_distance_sensor)) && (getTime() - turn_start_time > 1) && (isScenario05Detected)) {
							// Detectamos colisión
							std::cout << "Colisión detectada con Cubo" << std::endl;
							// Cambiar al siguiente estado
							scenarioState = DET_SCENARIO_05;
							
							
							// Detener el robot
							stopRobot(_leftMotor, _rightMotor);

							// Almacenar el tiempo de final del choque
							turn_start_time = getTime();
							break;
						}

						// Avanzar hasta chocar
						if ((isCollisionDetected(_distance_sensor)) && (getTime() - turn_start_time > 1)) {
							// Detectamos colisión
							std::cout << "Colisión detectada después de orientarse hacia -Z. Girando hacia +X." << std::endl;
							// Cambiar al siguiente estado
							currentState = TO_POS_X;
							// Detener el robot
							stopRobot(_leftMotor, _rightMotor);

							// Almacenar el tiempo de final del choque
							turn_start_time = getTime();
						}
						break;
						
					case TO_POS_X:
						// Esperar 1 segundo antes de avanzar
						if (getTime() - turn_start_time < 1) {
							break;
						}

						// Girar hacia el eje positivo X
						if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationPosX, orientationTolerance, timeStep)) {
							std::cout << "Orientación hacia +X alcanzada. Avanzando hasta colisión." << std::endl;
							// Cambiar al siguiente estado
							currentState = MOVE_TO_CRASH_X;
							// Parar
							moveForward(_leftMotor, _rightMotor, 0.0);

							// Almacenar el tiempo de final del giro
							turn_start_time = getTime();
						}
						break;
						
					case MOVE_TO_CRASH_X:
						// Esperar 1 segundo antes de avanzar
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
						

						// Velocidad máxima para colisión
						moveForward(_leftMotor, _rightMotor, 10.0);

						// Avanzar hasta chocar
						if (isCollisionDetected(_distance_sensor)) {
							// Detectamos colisión
							std::cout << "Colisión detectada después de orientarse hacia +X. Girando hacia -X." << std::endl;
							// Detener el robot
							stopRobot(_leftMotor, _rightMotor);
							// Cambiar al siguiente estado
							currentState = TO_NEG_X;

							// Almacenar el tiempo de final del choque
							turn_start_time = getTime();
						}
						break;
						
					case TO_NEG_X:
						// Esperar 1 segundo antes de avanzar
						if (getTime() - turn_start_time < 1) {
							break;
						}

						// Check for scenario 03 (pixels with RGB 0,0,0)
						for (const auto& pair : colorCount) {
							if (std::get<0>(pair.first) == 0 && std::get<1>(pair.first) == 0 && std::get<2>(pair.first) == 0) {
								if (pair.second > 1000) { // Umbral mínimo de píxeles negros
									std::cout << "Scenario 03 detected! Switching to scenario mode." << std::endl;
									scenarioState = SCENARIO_03;
									stopRobot(_leftMotor, _rightMotor);
									break;
								}
							}
						}

						// Girar hacia el eje negativo X
						if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationNegX, orientationTolerance, timeStep)) {
							std::cout << "Orientación hacia -X alcanzada. Tarea completada." << std::endl;
							// Cambiar al estado final
							currentState = FINISHED;
							// Detener el robot
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
                                      					std::cout << "Orientación hacia +Z alcanzada. Tarea completada." << std::endl;
                                      					moveForward(_leftMotor, _rightMotor, 0.0);
                                      					scenarioState = SCENARIO_02;                                                                                            
                                      					// Almacenar el tiempo de final del giro					
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
						// Mantener al robot detenido
						stopRobot(_leftMotor, _rightMotor);
						break;
				}
                break;

			//////////// Control de Posibles Escenarios 01, 08 y 10 ////////////
			case SCENARIO_01_08_10:
                // Specific handling for scenario 01_08_10
                std::cout << "Detectando que escenario es: (01, 08 o 10)" << std::endl;

				// Damos la vuelta al robot para que pueda verificar a que escenario pertenece en el estado siguiente
				if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationNegX, orientationTolerance, timeStep)) {
					std::cout << "Orientación hacia +Z alcanzada. Tarea completada." << std::endl;

					moveForward(_leftMotor, _rightMotor, 0.0);

					scenarioState = DET_01_08_10;					// Almacenar el tiempo de final del giro
								turn_start_time = getTime();

				}			              
                
                break;
			
			//////////// Detección de Posibles Escenarios 01, 08 y 10 ////////////	
			case DET_01_08_10:  
                // Specific handling for scenarios 01, 08 and 10
                
                if (getTime() - turn_start_time < 1) {
					break;
				}

				// Avanza el robot para tratar de detectar el escenario correcto
				moveForward(_leftMotor, _rightMotor, 10.0);
				
				// Si el color Color01_08_10 = (40, 110, 110)  es estable y supera los 14000 pixeles  
				// estamos en el escenario 08, sino tendriamos que detectar si estamos 
				// escenario 01 o 10
				if (colorCount[Color01_08_10] > 14000) { 
					std::cout << "Escenario 08" << std::endl;
					
					stopRobot(_leftMotor, _rightMotor);
					
					scenarioState = SCENARIO_08;
					
				} else if ((_distance_sensor[0]->getValue() > 900) && (colorCount[Color01_08_10] < 14000)) {
					
					// Si se activa el sensor DS6 estamos en el escenario 1, sino en el escenario 10
					
					if (_distance_sensor[6]->getValue() > 0) {
					
					std::cout << "Escenario 01" << std::endl;
					
					stopRobot(_leftMotor, _rightMotor);
					
					scenarioState = SCENARIO_01;
					
					} else {
					
					std::cout << "Escenario 10" << std::endl;
					
					stopRobot(_leftMotor, _rightMotor);
					
					scenarioState = SCENARIO_10;
					}
							
				}

                break;


			//////////// Control de Posibles Escenarios 04, 05, 06 y 07 ////////////
			case SCENARIO_04_05_06_07:
                // Specific handling for scenario 04_05_06_07
                std::cout << "Detectando que escenario es: (04, 05, 06 o 07)" << std::endl;

				// Damos la vuelta al robot para que pueda verificar a que escenario pertenece en el estado siguiente
				if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationNegX, orientationTolerance, timeStep)) {
					std::cout << "Orientación hacia +Z alcanzada. Tarea completada." << std::endl;

					moveForward(_leftMotor, _rightMotor, 0.0);

					scenarioState = DET_04_05_06_07;					
					turn_start_time = getTime();

				}			              
                
                break;


			//////////// Detección de Posibles Escenarios 04, 05, 06 y 07 ////////////
			case DET_04_05_06_07:  
                // Specific handling for scenarios 04, 05, 06 and 07
                
                if (getTime() - turn_start_time < 1) {
					break;
				}

				// Avanza el robot para tratar de detectar el escenario correcto
	     		moveForward(_leftMotor, _rightMotor, 10.0);
				
				// Si el color Color04_05_06_07 es estable y supera los 38000 píxeles, puede ser el escenario 04 o 05
				if (colorCount[Color04_05_06_07] > 38000) { 
					std::cout << "Escenario 04 o 05" << std::endl;

					// Si el sensor de distancia DS2 detecta un valor mayor a 740, es el escenario 05
					if (_distance_sensor[2]->getValue() > 740) {
								
						std::cout << "Escenario 05" << std::endl;
								
						stopRobot(_leftMotor, _rightMotor);
						scenarioState = SCENARIO_05;
					
					// Si DS2 detecta menos de 740, es el escenario 04
					} else if (_distance_sensor[2]->getValue() < 740) {
								
						std::cout << "Escenario 04" << std::endl;
								
						stopRobot(_leftMotor, _rightMotor);
						scenarioState = SCENARIO_04;
					}								
				
				// Si el Color04_05_06_07 no supera los 38000 píxeles y se detecta el color del escenario 06 (>14600), 
				// estamos en escenario 06 o 07	inicialmente
	     		} else if ((colorCount[Color04_05_06_07] < 38000) && (colorCount[Color06] > 14600)){
					// Si al avanzar, el sensor DS6 no detecta obstáculo y la cantidad de color del escenario 06 supera 15367, 
					// es el escenario 06
                    if ((_distance_sensor[6]->getValue() == 0) && (colorCount[Color06] > 15367)) {
						std::cout << "Escenario 06" << std::endl;
						scenarioState = SCENARIO_06;		
						stopRobot(_leftMotor, _rightMotor);

					// Si al avanzar, la cantidad de color del escenario 06 es menor de 15050, entonces es el escenario 07
					} else if (colorCount[Color06] < 15050) {
						std::cout << "Escenario 07" << std::endl;
						scenarioState = SCENARIO_07;				
						stopRobot(_leftMotor, _rightMotor);
					}
	    		}
	     
                break;

			//////////// Control de Escenario 05_1 ////////////
			case DET_SCENARIO_05:  
				// Specific handling for scenario 05_1
                std::cout << "Escenario 05" << std::endl; 

				// Damos la vuelta al robot para que actue bajo el escenario 05_1 
                if (turnToOrientation(this, _compass, _leftMotor, _rightMotor, orientationNegX, orientationTolerance, timeStep)) {
					std::cout << "Orientación hacia +Z alcanzada. Tarea completada." << std::endl;
					moveForward(_leftMotor, _rightMotor, 0.0);

					scenarioState = SCENARIO_05_1;					
					turn_start_time = getTime();
	     		}
	     		
				break;


			// //////////////////////////////////////// ESCENARIO 01 ////////////////////////////////////////
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el primer robot; dar una vuelta completa
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::TURN_LEFT, 100.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 4.5},
					{MovementStep::WAIT, 0.15},

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el segundo robot; dar una vuelta completa
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
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
				
				// Ejecutar la secuencia de movimientos
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario01Sequence, currentStepIndex_1, stepStartTime_1, timeStep)) {
					// Secuencia completada
					std::cout << "Scenario 01 movement sequence completed." << std::endl;
					// Aquí podrías agregar más lógica o cambiar a otro estado
					stopRobot(_leftMotor, _rightMotor);
				}
				break;


			// //////////////////////////////////////// ESCENARIO 02 ////////////////////////////////////////	
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el primer robot; dar una vuelta completa
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_RIGHT, 90.0},
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 3.0},
					{MovementStep::WAIT, 0.15}, 

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el segundo robot; dar una vuelta completa
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
				
				// Ejecutar la secuencia de movimientos
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario02Sequence, currentStepIndex_2, stepStartTime_2, timeStep)) {
					// Secuencia completada
					std::cout << "Scenario 2 movement sequence completed." << std::endl;
					// Aquí podrías agregar más lógica o cambiar a otro estado
					stopRobot(_leftMotor, _rightMotor);
				}	
	     
                break;


			// //////////////////////////////////////// ESCENARIO 03 ////////////////////////////////////////
			case SCENARIO_03:
				// Specific handling for scenario 03
				// MovementStep::WAIT - Esperar x segundos; ayuda a estabilizar las ruedas y los motores
				// MovementStep::MOVE_FORWARD - Avanzar x segundos
				// MovementStep::TURN_LEFT - Girar a la izquierda (180 - x) grados
				// MovementStep::TURN_RIGHT - Girar a la derecha (180 - x) grados
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

					{MovementStep::TURN_LEFT, 90.0},		// Encontrarse con el primer robot; dar una vuelta completa
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

					{MovementStep::TURN_LEFT, 90.0},		// Encontrarse con el segundo robot; dar una vuelta completa
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_LEFT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_LEFT, 90.0},
					{MovementStep::WAIT, 0.01},
					{MovementStep::TURN_LEFT, 104.0},		// Ángulo menor porque resbala - por defecto hace más de 360º
					{MovementStep::WAIT, 0.15},
					{MovementStep::MOVE_FORWARD, 2.3},		// Volver a la línea de salida
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
				
				// Ejecutar la secuencia de movimientos
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
											scenario03Sequence, currentStepIndex_3, stepStartTime_3, timeStep)) {
					// Secuencia completada
					std::cout << "Scenario 03 movement sequence completed." << std::endl;
					// Aquí podrías agregar más lógica o cambiar a otro estado
					stopRobot(_leftMotor, _rightMotor);
				}
				break;
			
			// //////////////////////////////////////// ESCENARIO 04 ////////////////////////////////////////
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el primer robot; dar una vuelta completa
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el segundo robot; dar una vuelta completa
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
				
				// Ejecutar la secuencia de movimientos
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario04Sequence, currentStepIndex_4, stepStartTime_4, timeStep)) {
					// Secuencia completada
					std::cout << "Scenario 04 movement sequence completed." << std::endl;
					// Aquí podrías agregar más lógica o cambiar a otro estado
					stopRobot(_leftMotor, _rightMotor);
				}

				break;


			// //////////////////////////////////////// ESCENARIO 05 ////////////////////////////////////////
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el primer robot; dar una vuelta completa
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el segundo robot; dar una vuelta completa
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
				
				// Ejecutar la secuencia de movimientos
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario05Sequence, currentStepIndex_5, stepStartTime_5, timeStep)) {
					// Secuencia completada
					std::cout << "Scenario 05 movement sequence completed." << std::endl;
					// Aquí podrías agregar más lógica o cambiar a otro estado
					stopRobot(_leftMotor, _rightMotor);
				}
				break;  

			// //////////////////////////////////////// ESCENARIO 05_1 ////////////////////////////////////////
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el primer robot; dar una vuelta completa
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el segundo robot; dar una vuelta completa
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
				
				// Ejecutar la secuencia de movimientos
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario051Sequence, currentStepIndex_51, stepStartTime_51, timeStep)) {
					// Secuencia completada
					std::cout << "Scenario 05_1 movement sequence completed." << std::endl;
					// Aquí podrías agregar más lógica o cambiar a otro estado
					stopRobot(_leftMotor, _rightMotor);
				}
				break;


			// //////////////////////////////////////// ESCENARIO 06 ////////////////////////////////////////
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el primer robot; dar una vuelta completa
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el segundo robot; dar una vuelta completa
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el primer robot; dar una vuelta completa
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el segundo robot; dar una vuelta completa
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
					
				// Ejecutar la secuencia de movimientos
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario07Sequence, currentStepIndex_7, stepStartTime_7, timeStep)) {
					// Secuencia completada
					std::cout << "Scenario 07 movement sequence completed." << std::endl;
					// Aquí podrías agregar más lógica o cambiar a otro estado
					stopRobot(_leftMotor, _rightMotor);
				}
			
				break; 


			// //////////////////////////////////////// ESCENARIO 08 ////////////////////////////////////////
	 		case SCENARIO_08:
                // Specific handling for scenario 08
				// MovementStep::WAIT - Esperar x segundos; ayuda a estabilizar las ruedas y los motores
				// MovementStep::MOVE_FORWARD - Avanzar x segundos
				// MovementStep::TURN_LEFT - Girar a la izquierda (180 - x) grados
				// MovementStep::TURN_RIGHT - Girar a la derecha (180 - x) grados
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
					
					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el primer robot; dar una vuelta completa
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
					
					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el segundo robot; dar una vuelta completa
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
				
				// Ejecutar la secuencia de movimientos
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario08Sequence, currentStepIndex_8, stepStartTime_8, timeStep)) {
					// Secuencia completada
					std::cout << "Scenario 08 movement sequence completed." << std::endl;
					// Aquí podrías agregar más lógica o cambiar a otro estado
					stopRobot(_leftMotor, _rightMotor);
				}
				break;





			// //////////////////////////////////////// ESCENARIO 09 ////////////////////////////////////////
            case SCENARIO_09:
                // Specific handling for scenario 09
				// MovementStep::WAIT - Esperar x segundos; ayuda a estabilizar las ruedas y los motores
				// MovementStep::MOVE_FORWARD - Avanzar x segundos
				// MovementStep::TURN_LEFT - Girar a la izquierda (180 - x) grados
				// MovementStep::TURN_RIGHT - Girar a la derecha (180 - x) grados
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el primer robot; dar una vuelta completa
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el segundo robot; dar una vuelta completa
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
				
				// Ejecutar la secuencia de movimientos
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
											scenario09Sequence, currentStepIndex_9, stepStartTime_9, timeStep)) {
					// Secuencia completada
					std::cout << "Scenario 09 movement sequence completed." << std::endl;
					// Aquí podrías agregar más lógica o cambiar a otro estado
					stopRobot(_leftMotor, _rightMotor);
				}
				break;

			// //////////////////////////////////////// ESCENARIO 10 ////////////////////////////////////////
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el primer robot; dar una vuelta completa
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

					{MovementStep::TURN_RIGHT, 90.0},	// Ha encontrado el segundo robot; dar una vuelta completa
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
				
				// Ejecutar la secuencia de movimientos
				if (executeMovementSequence(this, _leftMotor, _rightMotor, _compass, 
					scenario10Sequence, currentStepIndex_10, stepStartTime_10, timeStep)) {
					// Secuencia completada
					std::cout << "Scenario 10 movement sequence completed." << std::endl;
					// Aquí podrías agregar más lógica o cambiar a otro estado
					stopRobot(_leftMotor, _rightMotor);
				}
				break;
        }
	};

}

// Función para convertir grados a radianes
inline double MyRobot::degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}


// Función para ejecutar una secuencia de movimientos
bool MyRobot::executeMovementSequence(Robot* robot, Motor* leftMotor, Motor* rightMotor, Compass* compass,
                             std::vector<MovementStep>& sequence, int& currentStepIndex, 
                             double& stepStartTime, int timeStep) {
    
    // Si hemos completado la secuencia
    if (currentStepIndex >= sequence.size()) {
        return true;
    }
    
    // Obtener el paso actual
    MovementStep& currentStep = sequence[currentStepIndex];
    
    // Si es el inicio del paso actual, guardamos el tiempo
    if (stepStartTime < 0) {
        stepStartTime = robot->getTime();
        
        // Configurar el movimiento según el tipo
        switch (currentStep.type) {
            case MovementStep::WAIT:
            case MovementStep::STOP:
                // Detener el robot
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(0);
                break;
                
            case MovementStep::MOVE_FORWARD:
                // Avanzar
                leftMotor->setVelocity(10.0);  // Velocidad ajustable
                rightMotor->setVelocity(10.0);
                break;
                
            case MovementStep::TURN_LEFT:
            case MovementStep::TURN_RIGHT:
                // La lógica de giro se maneja en el caso siguiente
                break;
        }
    }
    
    // Lógica específica para cada tipo de paso
    switch (currentStep.type) {
        case MovementStep::WAIT:
        case MovementStep::STOP:
        case MovementStep::MOVE_FORWARD:
            // Para espera, parada o avance, simplemente esperamos el tiempo especificado
            if (robot->getTime() - stepStartTime >= currentStep.value) {
                // Pasar al siguiente paso
                currentStepIndex++;
                stepStartTime = -1;
            }
            break;
            
        case MovementStep::TURN_LEFT:
        case MovementStep::TURN_RIGHT: {
            // Para giros, necesitamos calcular la orientación objetivo
            
            // Obtener la orientación actual
            const double* compassValues = compass->getValues();
            
            // Calcular el ángulo actual (en el plano XZ)
            double currentAngle = atan2(compassValues[0], compassValues[2]);
            
            // Calcular el ángulo objetivo
            double targetAngleChange = degToRad(currentStep.value);
            if (currentStep.type == MovementStep::TURN_RIGHT) {
                targetAngleChange = -targetAngleChange;
            }
            
            // Si es el inicio del giro, guardamos el ángulo inicial y calculamos el objetivo
            static double initialAngle = 0;
            static double targetAngle = 0;
            
            if (robot->getTime() - stepStartTime < timeStep/1000.0) {
                initialAngle = currentAngle;
                targetAngle = initialAngle + targetAngleChange;
                // Normalizar el ángulo objetivo entre -PI y PI
                while (targetAngle > M_PI) targetAngle -= 2*M_PI;
                while (targetAngle < -M_PI) targetAngle += 2*M_PI;
            }
            
            // Calcular diferencia entre ángulo actual y objetivo
            double angleDiff = targetAngle - currentAngle;
			double angleDiff2 = targetAngle - currentAngle - M_PI;
			double angleDiff3 = targetAngle - currentAngle + M_PI;
            // Normalizar la diferencia entre -PI y PI
            while (angleDiff > M_PI) angleDiff -= 2*M_PI;
            while (angleDiff < -M_PI) angleDiff += 2*M_PI;
			while (angleDiff2 > M_PI) angleDiff2 -= 2*M_PI;
			while (angleDiff2 < -M_PI) angleDiff2 += 2*M_PI;
			while (angleDiff3 > M_PI) angleDiff3 -= 2*M_PI;
			while (angleDiff3 < -M_PI) angleDiff3 += 2*M_PI;

			// DEBUG: imprimir ángulo actual y objetivo
			std::cout << "Ángulo actual: " << currentAngle << ", Ángulo objetivo: " << targetAngle << std::endl;
			// Cuando el ángulo actual y el objetivo son iguales, realmente sale esto: [full_controller] Ángulo actual: -1.0973, Ángulo objetivo: 2.09729
			// Para solucionarlo, angleDiff debería ser la resta de los dos ángulos o la resta de uno - PI y del otro o la resta de uno y del otro - PI
			
            
            // Velocidad de giro proporcional a la diferencia de ángulo
            double turnSpeed = 2.0;  // Velocidad base ajustable
            
            if ((fabs(angleDiff) < degToRad(2.0)) || (fabs(angleDiff2) < degToRad(2.0)) || (fabs(angleDiff3) < degToRad(2.0))) {
                // Si estamos cerca del objetivo, paramos y pasamos al siguiente paso
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(0);
                currentStepIndex++;
                stepStartTime = -1;
            } else if (angleDiff > 0) {
                // Girar a la izquierda
                leftMotor->setVelocity(-turnSpeed);
                rightMotor->setVelocity(turnSpeed);
            } else {
                // Girar a la derecha
                leftMotor->setVelocity(turnSpeed);
                rightMotor->setVelocity(-turnSpeed);
            }
            break;
        }
    }
    
    return false;  // La secuencia no se ha completado todavía
}

// Función para detener el robot
void MyRobot::stopRobot(Motor* leftMotor, Motor* rightMotor) {
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
}

// Función para mover el robot hacia adelante
void MyRobot::moveForward(Motor* leftMotor, Motor* rightMotor, double speed) {
	// DEBUG: imprimir velocidad
	std::cout << "Moviendo hacia adelante a " << speed << " m/s" << std::endl;

    leftMotor->setVelocity(speed);
    rightMotor->setVelocity(speed);
}

// Función para detectar colisiones usando los sensores de distancia
bool MyRobot::isCollisionDetected(DistanceSensor* distance_sensors[]) {
	// Umbral para detectar colisiones (ajustar según sea necesario)
	const double COLLISION_THRESHOLD = 100.0;

	// Verificar los sensores frontales (ds0, ds15, ds1)
	// Los índices pueden necesitar ajustes dependiendo de la configuración exacta
	if (distance_sensors[0]->getValue() > COLLISION_THRESHOLD
		|| distance_sensors[15]->getValue() > COLLISION_THRESHOLD 
		|| distance_sensors[1]->getValue() > COLLISION_THRESHOLD) {
		return true;
	}

	return false;
}

/**
 * Gira el robot hacia una orientación específica
 * @param robot Puntero al objeto Robot
 * @param compass Puntero al sensor brújula
 * @param leftMotor Puntero al motor izquierdo
 * @param rightMotor Puntero al motor derecho
 * @param targetOrientation Vector de orientación objetivo [x, y, z]
 * @param tolerance Tolerancia permitida para considerar que se ha alcanzado la orientación
 * @param timeStep Paso de tiempo para la simulación
 * @return true cuando se ha alcanzado la orientación deseada
 */
bool MyRobot::turnToOrientation(Robot* robot, Compass* compass, Motor* leftMotor, Motor* rightMotor, const double targetOrientation[3], double tolerance, int timeStep) {
	// Velocidades para el giro
	const double TURN_SPEED = 1.5;

	// Obtener los valores actuales de la brújula
	const double* compassValues = compass->getValues();

	if (compassValues == NULL) {
		std::cout << "Error: No se pueden leer los valores de la brújula" << std::endl;
		return false;
	}

	// Calcular el producto escalar entre la orientación actual y la deseada
	double dotProduct = compassValues[0] * targetOrientation[0] + compassValues[1] * targetOrientation[1] + compassValues[2] * targetOrientation[2];

	// Normalizar los vectores
	double currentMagnitude = sqrt(compassValues[0]*compassValues[0] + compassValues[1]*compassValues[1] + compassValues[2]*compassValues[2]);

	double targetMagnitude = sqrt(targetOrientation[0]*targetOrientation[0] + targetOrientation[1]*targetOrientation[1] + targetOrientation[2]*targetOrientation[2]);

	// Calcular el coseno del ángulo entre los vectores
	double cosAngle = dotProduct / (currentMagnitude * targetMagnitude);

	// Asegurar que cosAngle está en el rango [-1, 1]
	if (cosAngle > 1.0) cosAngle = 1.0;
	if (cosAngle < -1.0) cosAngle = -1.0;

	// Calcular el ángulo en radianes
	double angle = acos(cosAngle);

	// Verificar si ya estamos en la orientación correcta (dentro de la tolerancia)
	if (angle < tolerance) {
		// Detener los motores
		leftMotor->setVelocity(0);
		rightMotor->setVelocity(0);
		return true;
	}

	// Determinar la dirección de giro (derecha o izquierda)
	// Para esto, usamos el producto vectorial para determinar si debemos girar en sentido horario o antihorario
	double crossProduct[3];
	crossProduct[0] = compassValues[1] * targetOrientation[2] - compassValues[2] * targetOrientation[1];
	crossProduct[1] = compassValues[2] * targetOrientation[0] - compassValues[0] * targetOrientation[2];
	crossProduct[2] = compassValues[0] * targetOrientation[1] - compassValues[1] * targetOrientation[0];

	// El signo de la componente Y del producto vectorial nos indica la dirección de giro
	bool turnRight = (crossProduct[1] > 0);

	// Establecer velocidades para el giro
	if (turnRight) {
		// Girar a la derecha
		std::cout << "Girando a la derecha" << std::endl;
		leftMotor->setVelocity(TURN_SPEED);
		rightMotor->setVelocity(-TURN_SPEED);
	} else {
		// Girar a la izquierda
		std::cout << "Girando a la izquierda" << std::endl;
		leftMotor->setVelocity(-TURN_SPEED);
		rightMotor->setVelocity(TURN_SPEED);
	}

	// Aún no hemos alcanzado la orientación deseada
	return false;
}

// Función para detectar el escenario basado en colores y conteos de píxeles
std::string MyRobot::detect_scenario(const std::vector<std::tuple<int, int, int>>& colors, 
                           const std::vector<int>& pixel_counts) {
    // Si no hay colores, no podemos detectar nada
    if (colors.empty()) {
        return "unknown";
    }
    
    // Obtener componentes del primer color dominante
    int r0 = std::get<0>(colors[0]);
    int g0 = std::get<1>(colors[0]);
    int b0 = std::get<2>(colors[0]);
    
    // Los escenarios 1, 8 y 10 tienen colores similares
    if (r0 == 50 && g0 == 120 && b0 == 120 && colors.size() > 1) {
        int r1 = std::get<0>(colors[1]);
        int g1 = std::get<1>(colors[1]);
        int b1 = std::get<2>(colors[1]);
        
        if (r1 == 40 && g1 == 100 && b1 == 100) {
            return "01y08y10";
        }
    }

	// Escenario 3 (con múltiples colores en tonos específicos y negro)
	if ((r0 == 70 && g0 == 160 && b0 == 160) || (r0 == 60 && g0 == 140 && b0 == 140)) {
		// Verificar si existe el color negro en los primeros 5 colores
		for (int i = 0; i < std::min(5, (int)colors.size()); i++) {
			int r = std::get<0>(colors[i]);
			int g = std::get<1>(colors[i]);
			int b = std::get<2>(colors[i]);
			
			if (r == 0 && g == 0 && b == 0) {
				return "03";
			}
		}
	}

    
    // Para escenarios 4, 5 (caso 0), 6 y 7 con color dominante (80,190,190)
    else if (r0 == 80 && g0 == 190 && b0 == 190) {
        return "04y05y06y07";
    }	

	// Para escenario 5 (caso 1) con color dominante (70,170,170)
    else if (r0 == 70 && g0 == 170 && b0 == 170) {
        
        return "05";
    }
    // Para escenario 2 con color dominante (80,180,180)
    else if (r0 == 80 && g0 == 180 && b0 == 180 && colors.size() == 1) {
        return "02";
    }
    
    // Si no coincide con ningún patrón conocido
    return "unknown";
}
//////////////////////////////////////////////
