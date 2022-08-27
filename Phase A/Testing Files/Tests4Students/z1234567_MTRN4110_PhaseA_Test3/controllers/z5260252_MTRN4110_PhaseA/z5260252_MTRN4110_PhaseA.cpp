/*
 * File:          z5260252_MTRN4110_PhaseA.cpp
 * Date:          04/06/2022
 * Description:   Controller of E-puck for Phase A - Driving and Perception
 * Author:        Gajendra Jayasekera (z5260252)
 * Modifications: 
 * Platform:      MacOS
 * Notes:         
 */       

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include <fstream>
#include <string>

#define TIME_STEP 64
#define MAXSPEED 6.28
#define PI 3.14159265
#define CELL_SIZE 0.165

const std::string COUT_PREFIX = "[z5260252_MTRN4110_PhaseA] ";
const std::string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";

std::string readMotionPlan();
char updateWallData(double distanceSensorValue);
void updateMessage(int step, int row, int column, char heading, char leftWall, char frontWall, char rightWall);
double updateHeading(char heading, char command);

using namespace webots;

int main(int argc, char **argv) {
  // Create the Robot, Motor, DistanceSensor, InertialUnit and PositionSensor instances and enable them.
  Robot *robot = new Robot();
  Motor *leftMotor{robot->getMotor("left wheel motor")};
  Motor *rightMotor{robot->getMotor("right wheel motor")};
  DistanceSensor *leftDS{robot->getDistanceSensor("dsL")};
  leftDS->enable(TIME_STEP);
  DistanceSensor *frontDS{robot->getDistanceSensor("dsF")};
  frontDS->enable(TIME_STEP);
  DistanceSensor *rightDS{robot->getDistanceSensor("dsR")};
  rightDS->enable(TIME_STEP);
  InertialUnit *imu{robot->getInertialUnit("IMU")};
  imu->enable(TIME_STEP);
  PositionSensor *leftEncoder{robot->getPositionSensor("left wheel sensor")};
  leftEncoder->enable(TIME_STEP);
  PositionSensor *rightEncoder{robot->getPositionSensor("right wheel sensor")};
  rightEncoder->enable(TIME_STEP);
  
  // Read the motion plan from a TXT file and display to console.
  std::string motionPlan = readMotionPlan();
  
  // Set the time step of the current world.
  int timeStep = TIME_STEP;

  int step = 0;
  // Convert from char to int using '0' ~ 48 ASCII
  int row = motionPlan.at(0) - '0';
  int column = motionPlan.at(1) - '0';
  char heading = motionPlan.at(2);
  // Give distance sensors time and update wall data.
  robot->step(TIME_STEP);
  char leftWall = updateWallData(leftDS->getValue());
  char frontWall = updateWallData(frontDS->getValue());
  char rightWall = updateWallData(rightDS->getValue());
  // Print initial location/heading/sensor data to console and save to CSV file.
  updateMessage(step, row, column, heading, leftWall, frontWall, rightWall);
  
  step++;
  
  int motionlength = motionPlan.length();
  // To store current loops position encoder values. 
  double leftEncoderPosition = 0;
  double rightEncoderPosition = 0;
  // Reduced speed to avoid wheel slipping and read the data more accruately.
  double turnVelocity = 0.4 * MAXSPEED;
  // Final trial and error values for wheel radius and axle length.
  float r = 0.019875;
  float l = 0.056575 / 2;
  // Calculation of wheel rotation needed for pure linear and rotation motions.
  double forwardStep = CELL_SIZE / r;
  double turn90Deg = ((PI / 2) * l) / r;
  
  while (robot->step(timeStep) != -1) {
    if ((step + 2) < motionlength) {
      // Get command from motion plan.
      char command = motionPlan.at(step + 2);
      // Get position encoder values.
      robot->step(TIME_STEP);
      leftEncoderPosition = leftEncoder->getValue();
      rightEncoderPosition = rightEncoder->getValue();
      
      // Execute the motion plans commands.
      switch(command) {
        case 'L':
          // Left turn relative to current encoder readings.
          leftMotor->setVelocity(turnVelocity);
          rightMotor->setVelocity(turnVelocity);
          leftMotor->setPosition(leftEncoderPosition - turn90Deg);
          rightMotor->setPosition(rightEncoderPosition + turn90Deg);
          robot->step(1100 + TIME_STEP);
          break;
        case 'F':
          // Forward 1 step relative to current encoder readings.
          leftMotor->setVelocity(MAXSPEED);
          rightMotor->setVelocity(MAXSPEED);
          leftMotor->setPosition(leftEncoderPosition + forwardStep);
          rightMotor->setPosition(rightEncoderPosition + forwardStep);
          robot->step(1350 + TIME_STEP);
          
          // Update location.
          if (heading == 'N') {
            row--;
          } else if (heading == 'E') {
            column++;
          } else if (heading == 'S') {
            row++;
          } else if (heading == 'W') {
            column--;
          }
          break;
        case 'R':
          // Right turn relative to current encoder readings.
          leftMotor->setVelocity(turnVelocity);
          rightMotor->setVelocity(turnVelocity);
          leftMotor->setPosition(leftEncoderPosition + turn90Deg);
          rightMotor->setPosition(rightEncoderPosition - turn90Deg);
          robot->step(1100 + TIME_STEP);
          break;
      }
      
      // Update wall data then heading and print to console and CSV.
      robot->step(TIME_STEP);
      leftWall = updateWallData(leftDS->getValue());
      frontWall = updateWallData(frontDS->getValue());
      rightWall = updateWallData(rightDS->getValue());
      heading = updateHeading(heading, command);
      updateMessage(step, row, column, heading, leftWall, frontWall, rightWall);
      
      step++;
    } else {
      // Breakout of while loop after executing all motionplans.
      break;
    }
  }
  // Exit message and code cleanup.
  std::cout << COUT_PREFIX << "Motion plan executed!" << std::endl;
  delete robot;
  return 0;
}

// Read the motion plan from a TXT file and display to console.
std::string readMotionPlan() {
  // Code adapted from: https://stackoverflow.com/questions/13035674/how-to-read-a-file-line-by-line-or-a-whole-text-file-at-once
  std::cout << COUT_PREFIX << "Reading in motion plan from ../../MotionPlan.txt..." << std::endl;
  std::ifstream file(MOTION_PLAN_FILE_NAME);
  std::string motionPlan;
  std::getline(file, motionPlan);
  std::cout << COUT_PREFIX << "Motion Plan: " << motionPlan << std::endl;
  std::cout << COUT_PREFIX << "Motion plan read in!" << std::endl;
  std::cout << COUT_PREFIX << "Executing motion plan..." << std::endl;
  return motionPlan;
}

// Detect/update walls for passed in distance sensor.
char updateWallData(double distanceSensorValue) {
  // Walls are ~400 from the Epuck.
  // Lower and upper limits of noise cancellation.
  int lowerLimit = 175;
  int upperLimit = 625;
  char updatedWallData;
  if ((distanceSensorValue > lowerLimit) && (distanceSensorValue < upperLimit)) {
      updatedWallData = 'Y';
  } else {
      updatedWallData = 'N';
  }
  return updatedWallData;
}

// Update location/heading/sensor data then print to console and save to CSV file.
void updateMessage(int step, int row, int column, char heading, char leftWall, char frontWall, char rightWall) {
  // Code adapted from: https://stackoverflow.com/questions/530614/print-leading-zeros-with-c-output-operator
  std::cout << COUT_PREFIX << "Step: " << std::setw(3) << std::setfill('0') << step << ", ";
  std::cout << "Row: " << row << ", ";
  std::cout << "Column: " << column << ", ";
  std::cout << "Heading: " << heading << ", ";
  std::cout << "Left Wall: " << leftWall << ", ";
  std::cout << "Front Wall: " << frontWall << ", ";
  std::cout << "Right Wall: " << rightWall << std::endl;
  
  // Code adapted from: https://www.codeproject.com/Questions/417270/Reading-from-and-writing-to-Excel-files-in-Cpluspl
  std::ofstream saveFile;
  if (step == 0) {
    // Open CSV file clearing all current data in file, append table headings and current status then close.
    saveFile.open(MOTION_EXECUTION_FILE_NAME);
    saveFile << "Step" << "," << "Row" << "," << "Column" << "," << "Heading" << "," << "Left Wall" << "," << "Front Wall" << "," << "Right Wall" << "\n";
    saveFile << step << "," << row << "," << column << "," << heading << "," << leftWall << "," << frontWall << "," << rightWall << std::endl;
  } else {
    // Open CSV file, append current status then close.
    // Code adapted from: https://cplusplus.com/forum/beginner/212086/
    saveFile.open(MOTION_EXECUTION_FILE_NAME, std::ios::app);
    saveFile << step << "," << row << "," << column << "," << heading << "," << leftWall << "," << frontWall << "," << rightWall << std::endl;
  }
  saveFile.close();
}

// Update heading based on previous heading and current command.
double updateHeading(char heading, char command) {
  // Leave heading if it was a move forward command.
  char updatedHeading = heading;
  switch (heading) {
    case 'N':
      if (command == 'L') {
        updatedHeading = 'W';
      } else if (command == 'R') {
        updatedHeading = 'E';
      }
      break;
    case 'E':
      if (command == 'L') {
        updatedHeading = 'N';
      } else if (command == 'R') {
        updatedHeading = 'S';
      }
      break;
    case 'S':
      if (command == 'L') {
        updatedHeading = 'E';
      } else if (command == 'R') {
        updatedHeading = 'W';
      }
      break;
    case 'W':
      if (command == 'L') {
        updatedHeading = 'S';
      } else if (command == 'R') {
        updatedHeading = 'N';
      }
      break;
  }
  return updatedHeading;
}
