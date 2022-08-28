#pragma once
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Supervisor.hpp>

#include <BangBang.h>
#include <string>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <vector>

#define PI 3.14159265359
#define MAXSPEED 6.28
#define WHEELRADIUS 19.991
#define AXLELENGTH 28.438
#define STEPDELTA 0.006
#define ROWS 5
#define COLUMNS 9
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

using namespace webots;

void runbangbang()
{
    // Define motion file path
    const std::string MAP_FILE_NAME = "../../Map.txt";
    // Define csv file path
    const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";
    // Define robot name prefix for printing
    const std::string ROBOT_NAME = "[Pathfinder[20]_MTRN4110_PhaseD]";

    Epuck Epuck{MAP_FILE_NAME, MOTION_EXECUTION_FILE_NAME, ROBOT_NAME, WHEELRADIUS, AXLELENGTH};
    Epuck.readMap();
    Epuck.followPlan();
}