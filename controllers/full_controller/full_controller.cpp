/**
 * @file    controller.cpp
 * @brief   Controller for the whole project.
 *
 * @author  Álvaro Cabrera Nieto        <100472152@alumnos.uc3m.es>
 * @author  Iván Sebastián Loor Weir    <100448737@alumnos.uc3m.es>
 * @date    2025
*/
#include "MyRobot.h"
int main(int argc, char **argv)
{
	MyRobot* my_robot = new MyRobot();
	my_robot->run();
	delete my_robot;
	return 0;
}
