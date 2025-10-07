# Autonomous Rescue System - Pioneer II

## 📋 Project Description

Final project for the Robotics course (2024-25) at Universidad Carlos III de Madrid. It consists of developing an intelligent controller for the Pioneer II robot that must explore different scenarios, rescue two victims, and return to the starting point autonomously and optimally.

**Authors:**
- Álvaro Cabrera Nieto (100472152)
- Iván Sebastián Loor Weir (100448737)

**Group:** 81 - Fourth Year Computer Engineering

---

## 🎯 Objectives

- ✅ Rescue 2 people at the opposite end of the scenario
- ✅ Perform a 360º turn within 1 meter of each victim
- ✅ Return to the starting line without significant collisions
- ✅ Complete the mission in less than 4 minutes
- ✅ Optimize rescue time for each scenario
- ✅ Maximum 3 collisions allowed

---

## 🤖 Pioneer II Robot Features

### Available Sensors
- **16 distance sensors** (360º coverage)
- **Encoders** on each fixed wheel
- **Compass** (precise orientation)
- **GPS** (3-meter resolution)
- **Front RGB camera**
- **Spherical RGB camera**

### Actuators
- **2 independent fixed wheels** (differential drive)
- **1 support wheel** (caster wheel)

---

## 🗺️ Scenarios

The project includes **10 different scenarios** with the following characteristics:

- Dimensions: **10×20 meters**
- Variable obstacles and walls
- Different lighting and fog conditions
- Fixed victim positions (consistent per scenario)
- Consistent GPS coordinate system and compass

---

## 🧠 Solution Strategy

### Phase 1: Scenario Identification

The system implements a **systematic recognition process** based on:

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

Once the scenario is identified, the robot executes an **optimized and predefined route** that includes:

- `MOVE_FORWARD`: Advance for a determined time
- `TURN_LEFT`: Turn left (specific angle)
- `TURN_RIGHT`: Turn right (specific angle)
- `WAIT`: Motor stabilization

### Special Cases

- **Scenario 05:** Includes two variants depending on initial position relative to an obstacle cube
- **Scenario 09:** Early identification through unique lighting pattern
- **Scenario 10:** Victims very close to each other

---

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

---

## 💪 Strengths

1. **Total reliability:** 100% success in victim rescue
2. **Systematic method:** Robust environment identification
3. **Optimized efficiency:** Predefined and manually adjusted routes
4. **Restriction compliance:** Maximum 2 controlled collisions
5. **Competitive times:** All scenarios completed under 4 minutes

---

## ⚠️ Limitations

1. **Dependence on initial collisions** for identification
2. **Time variability** depending on environmental graphics complexity
3. **Rigid routes:** Limited adaptability to unforeseen changes
4. **Sensitivity to initial conditions:** Orientation and position affect precision
5. **Limited scalability:** Optimized for 10 specific scenarios

---

## 🛠️ Technologies Used

- **Simulator:** Webots
- **Robot:** Pioneer II
- **Language:** C++ (`full_controller`)
- **Main sensors:** Compass, distance sensors, front RGB camera

---

## 📁 Project Structure

```
robotics-project/
├── controllers/
│   └── full_controller/
│       └── [controller code]
├── worlds/
│   ├── scenario_01.wbt
│   ├── scenario_02.wbt
│   └── ... (10 scenarios)
├── docs/
│   └── Informe_Practica_6.pdf
└── README.md
```

---

## 🚀 Usage Instructions

1. **Open Webots** and load the desired world
2. **Run the controller** full_controller
3. The robot will automatically identify the scenario
4. It will follow the optimized route for that world
5. It will rescue both victims and return to the starting line

---

## 🔮 Future Work

- Improve precision of movement and orientation functions
- Eliminate dependence on collisions for identification
- Implement more adaptive and reactive navigation
- Optimize processing for environments with complex graphics
- Develop scalable system for more than 10 scenarios

---

## 📝 License

Academic Project - Universidad Carlos III de Madrid © 2024-2025

The code is provided for educational and reference purposes only.

**Exception:** The Makefile uses Apache License 2.0 (Copyright 1996-2020 Cyberbotics Ltd.)


---

## 📧 Contact

- Álvaro Cabrera Nieto: 100472152@alumnos.uc3m.es
- Iván Sebastián Loor Weir: 100448737@alumnos.uc3m.es || sebas80sebas@gmail.com
