# Autonomous Rescue System - Pioneer II

## 📋 Project Description
Final project for the Robotics course (2024-25) at Universidad Carlos III de Madrid. It consists of developing an intelligent controller for the Pioneer II robot that must explore different scenarios, rescue two victims, and return to the starting point autonomously and optimally.

**Authors:**
- Álvaro Cabrera Nieto (100472152) 
- Iván Sebastián Loor Weir (100448737) 

**Group:** 81 - Fourth Year Computer Engineering

---

## 🛠️ Installation and Setup

### Prerequisites
- **Webots Simulator** (recommended version: 2020a)
- **C++ Compiler** (g++ or equivalent)
- **Operating System:** Linux (tested on Ubuntu/Debian-based distributions)

### 1. Installing Webots

#### On Linux
1. Download Webots from the [official website](https://cyberbotics.com/)
2. Install Webots in the default directory:
   ```bash
   sudo mkdir -p /usr/local/webots
   sudo tar xjf webots-*.tar.bz2 -C /usr/local/
   ```

3. Verify the installation by launching Webots:
   ```bash
   cd /usr/local/webots
   ./webots
   ```

#### Alternative: Using a Terminal
Open a terminal in your desktop:
- Right-click on the desktop background
- Select "Open Terminal Here"
- Navigate to Webots directory:
  ```bash
  cd /usr/local/webots
  ./webots
  ```

### 2. Project Structure Setup

The project must follow the standard Webots directory hierarchy:

```
robotics-project/
├── controllers/
│   └── full_controller/
│       ├── full_controller.cpp
│       ├── Makefile
│       └── [additional controller files]
├── worlds/
│   ├── scenario_01.wbt
│   ├── scenario_02.wbt
│   ├── scenario_03.wbt
│   ├── scenario_04.wbt
│   ├── scenario_05.wbt
│   ├── scenario_06.wbt
│   ├── scenario_07.wbt
│   ├── scenario_08.wbt
│   ├── scenario_09.wbt
│   └── scenario_10.wbt
├── docs/
│   └── Informe_Practica_6.pdf
└── README.md
```

**⚠️ Important:** 
- Worlds (.wbt files) **MUST** be stored directly in a directory named `worlds`
- Controllers **MUST** be stored in subdirectories within the `controllers` folder
- Each controller should have its own folder with the same name as the controller

### 3. Importing the Project

#### Method 1: Clone from Repository (if applicable)
```bash
git clone [repository-url]
cd robotics-project
```

#### Method 2: Manual Import
1. Download or extract the project to your desired location
2. Ensure the directory structure matches the one shown above
3. Open Webots:
   ```bash
   cd /usr/local/webots
   ./webots
   ```

4. In Webots, go to: **File → Open World**
5. Navigate to your project's `worlds` directory
6. Select any scenario file (e.g., `scenario_01.wbt`)

### 4. Configuring the Controller

#### Assigning the Controller to the Robot
1. **Pause the simulation** (click the Pause button in the toolbar)
2. **Reset the simulation** (click the Reset button - time should show 0:00:00:000)
3. In the Scene Tree (left panel), expand the robot node (Pioneer2)
4. Locate the `controller` field
5. Double-click on the controller field
6. Click the **Select...** button
7. Choose `full_controller` from the list
8. **Save the world** (Ctrl+S or File → Save)

#### Compiling the Controller
1. In Webots, go to: **Build → Build**
2. Wait for compilation to complete
3. Check the console for any compilation errors
4. If errors occur, verify that all necessary includes are present:
   ```cpp
   #include <webots/Robot.hpp>
   #include <webots/Motor.hpp>
   #include <webots/DistanceSensor.hpp>
   #include <webots/Compass.hpp>
   #include <webots/GPS.hpp>
   #include <webots/Camera.hpp>
   // Add other necessary headers
   ```

### 5. Running the Simulation

#### Basic Execution
1. **Load a world:** Open any scenario file (scenario_01.wbt to scenario_10.wbt)
2. **Verify the controller** is assigned to the robot
3. **Reset the simulation** (mandatory before each run)
4. **Start the simulation** by clicking the Play button (▶️)

#### Important Notes
- **Always pause and reset** before modifying the world to avoid accumulating errors
- The simulation must start at time 0:00:00:000
- The robot will automatically identify the scenario and execute the optimized route

#### Testing Different Scenarios
To test all scenarios sequentially:
1. Open scenario_01.wbt
2. Run the simulation and observe results
3. Pause and close the world
4. Open scenario_02.wbt
5. Repeat for all 10 scenarios

---

## 🎯 Objectives
- ✅ Rescue 2 people at the opposite end of the scenario 
- ✅ Perform a 360º turn within 1 meter of each victim 
- ✅ Return to the starting line without significant collisions 
- ✅ Complete the mission in less than 4 minutes 
- ✅ Optimize rescue time for each scenario 
- ✅ Maximum 3 collisions allowed 

## 🤖 Pioneer II Robot Features

### Available Sensors
- 16 distance sensors (360º coverage) 
- Encoders on each fixed wheel 
- Compass (precise orientation) 
- GPS (3-meter resolution) 
- Front RGB camera 
- Spherical RGB camera 

### Actuators
- 2 independent fixed wheels (differential drive) 
- 1 support wheel (caster wheel) 

## 🗺️ Scenarios
The project includes 10 different scenarios with the following characteristics:
- Dimensions: 10×20 meters 
- Variable obstacles and walls 
- Different lighting and fog conditions 
- Fixed victim positions (consistent per scenario) 
- Consistent GPS coordinate system and compass 

## 🧠 Solution Strategy

### Phase 1: Scenario Identification
The system implements a systematic recognition process based on:

1. **Initial calibration:**
   - Orientation towards the upper wall 
   - First controlled collision (upper wall) 
   - 90º left turn 
   - Second controlled collision (initial wall) 
   - 180º turn towards the finish line 

2. **Analysis of distinctive features:**
   - RGB color patterns detected by front camera 
   - Distance sensor readings 
   - Compass orientation 

3. **Differentiation through dominant colors:**
   - Each scenario presents unique color combinations 
   - Pixel analysis and specific thresholds 
   - Validation using distance sensors 

### Phase 2: Predetermined Navigation
Once the scenario is identified, the robot executes an optimized and predefined route that includes:
- **MOVE_FORWARD:** Advance for a determined time 
- **TURN_LEFT:** Turn left (specific angle) 
- **TURN_RIGHT:** Turn right (specific angle) 
- **WAIT:** Motor stabilization 

### Special Cases
- **Scenario 05:** Includes two variants depending on initial position relative to an obstacle cube 
- **Scenario 09:** Early identification through unique lighting pattern 
- **Scenario 10:** Victims very close to each other 

## 📊 Results

| Scenario | Victims | Collisions | Time (mm:ss:cs) | Distance (m) |
|----------|---------|------------|-----------------|--------------|
| 01 | 2 | 2 | 02:08:15 | 56.60 |
| 02 | 2 | 2 | 02:14:68 | 66.78 |
| 03 | 2 | 2 | 02:09:08 | 59.22 |
| 04 | 2 | 2 | 02:00:22 | 50.81 |
| 05 (case 0) | 2 | 2 | 02:14:20 | 62.75 |
| 05 (case 1) | 2 | 1 | 01:58:04 | 55.39 |
| 06 | 2 | 2 | 02:09:31 | 57.95 |
| 07 | 2 | 2 | 01:55:87 | 54.66 |
| 08 | 2 | 2 | 01:48:54 | 50.30 |
| 09 | 2 | 1 | 01:48:99 | 51.51 |
| 10 | 2 | 2 | 02:19:13 | 61.89 |

**Success rate:** 100% in victim rescue

## 💪 Strengths
1. **Total reliability:** 100% success in victim rescue 
2. **Systematic method:** Robust environment identification 
3. **Optimized efficiency:** Predefined and manually adjusted routes 
4. **Restriction compliance:** Maximum 2 controlled collisions 
5. **Competitive times:** All scenarios completed under 4 minutes 

## ⚠️ Limitations
1. Dependence on initial collisions for identification 
2. Time variability depending on environmental graphics complexity 
3. Rigid routes: Limited adaptability to unforeseen changes 
4. Sensitivity to initial conditions: Orientation and position affect precision 
5. Limited scalability: Optimized for 10 specific scenarios 

## 🛠️ Technologies Used
- **Simulator:** Webots (v2020a recommended)
- **Robot:** Pioneer II 
- **Language:** C++ (full_controller) 
- **Main sensors:** Compass, distance sensors, front RGB camera 

## 🔧 Troubleshooting

### Common Issues

#### Controller Not Compiling
- Verify all necessary headers are included
- Check that the controller directory name matches the controller name
- Ensure the Makefile is present in the controller directory

#### Robot Not Moving
- Verify the controller is assigned to the robot
- Check that the simulation was reset before starting
- Ensure motor names are correct: "left wheel motor" and "right wheel motor"

#### Robot Passes Through Walls
- Verify walls have `boundingObject` defined
- Check that walls are Solid nodes with proper physics properties
- For custom worlds, ensure all obstacles have collision boundaries

#### Compilation Errors with INFINITY
- Add `#include <cmath>` or `#include <math.h>` to your controller

### Performance Tips
- Close unnecessary applications to improve simulation speed
- Use the 2020a version of Webots for better stability
- Reduce graphical quality if experiencing lag (Edit → Preferences → OpenGL)

## 📖 Additional Documentation

For more detailed information about the implementation, algorithms, and methodology, please refer to the complete project report (in Spanish):

**📄 Report:** `docs/Informe_Practica_6.pdf`

This comprehensive document includes:
- Detailed explanation of the scenario identification algorithm
- Complete route optimization methodology
- In-depth analysis of sensor usage and calibration
- Extended discussion of results and performance metrics
- Technical diagrams and flowcharts

## 🔮 Future Work
- Improve precision of movement and orientation functions 
- Eliminate dependence on collisions for identification 
- Implement more adaptive and reactive navigation 
- Optimize processing for environments with complex graphics 
- Develop scalable system for more than 10 scenarios 

## 📝 License
Academic Project - Universidad Carlos III de Madrid © 2024-2025

The code is provided for educational and reference purposes only.

Exception: The Makefile uses Apache License 2.0 (Copyright 1996-2020 Cyberbotics Ltd.)

## 📧 Contact
- Álvaro Cabrera Nieto: 100472152@alumnos.uc3m.es 
- Iván Sebastián Loor Weir: 100448737@alumnos.uc3m.es
