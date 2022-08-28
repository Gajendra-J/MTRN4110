// File:          Ghost2.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <string>
#include <queue>
#include <utility>
#include <vector>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <string>
#include <iomanip>
#include <cmath>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Supervisor.hpp>

#define PI 3.14159265359
#define MAXSPEED 6.28
// Position Sensor = 82.5369 after 10 cells
// Therefore WHEELRADIUS = 165*10/82.5369
#define WHEELRADIUS 19.991
// Position Sensor = 89.3814 after 10 rotations
// Thereforce AXLELENGTH = WHEELRADIUS * 89.3814/(10 * 2 * Pi)
#define AXLELENGTH 28.438
#define STEPDELTA 0.006
#define TILESIZE 0.165

// All the webots classes are defined in the "webots" namespace
using namespace webots;

class Epuck
{
public:
  Epuck(std::string, double, double);
  void readMap();
  void follow();

private:
  void getPosition();
  int calculateCell(const double *);
  void move1Step();
  void checkEdge();
  void checkCollision();
  void rotate90(char direction);
  std::string pathPlanner(std::vector<int>, char);
  Supervisor *_robot;
  Motor *_leftMotor;
  Motor *_rightMotor;
  PositionSensor *_leftPosSensor;
  PositionSensor *_rightPosSensor;
  InertialUnit *_imu;
  DistanceSensor *_ps0;
  DistanceSensor *_ps1;
  DistanceSensor *_ps6;
  DistanceSensor *_ps7;
  Field *_translation;
  Field *_target_translation;
  Field *_ghost1Translation;
  Field *_ghost3Translation;
  std::string _mapFileName;
  std::string _instructions;
  int _timeStep;
  double _wheelRadius;
  double _axleLength;
  int _start;
  int _end;
  int _ghost1;
  int _ghost3;
  char _startHeading;
  int _numCols;
  std::vector<int> _wallCols;
  std::vector<int> _wallRows;
  std::vector<std::vector<int>> _adjacencyMatrix;
  std::vector<std::vector<int>> _paths;
  std::vector<std::string> _map;
  std::vector<int> _shortestPath;
  std::vector<std::string> _shortestMap;
  void generatePaths();
  void printPaths();
  void generateMotion();
  void followPath(char);
  void getPaths(std::vector<int> &, std::vector<std::vector<int>> &, int);
  void checkCell(int, int, std::queue<int> &, std::vector<int> &, std::vector<std::vector<int>> &);
};

Epuck::Epuck(std::string mapFileName, double wheelRadius, double axleLength)
    : _mapFileName(mapFileName), _wheelRadius(wheelRadius), _axleLength(axleLength)
{
  // Initialise variables
  _robot = new Supervisor();
  _leftMotor = _robot->getMotor("left wheel motor");
  _rightMotor = _robot->getMotor("right wheel motor");
  _leftPosSensor = _robot->getPositionSensor("left wheel sensor");
  _rightPosSensor = _robot->getPositionSensor("right wheel sensor");
  _imu = _robot->getInertialUnit("IMU");
  _ps0 = _robot->getDistanceSensor("ps0");
  _ps1 = _robot->getDistanceSensor("ps1");
  _ps6 = _robot->getDistanceSensor("ps6");
  _ps7 = _robot->getDistanceSensor("ps7");
  _timeStep = (int)_robot->getBasicTimeStep();

  // Enable all the sensors
  _leftPosSensor->enable(_timeStep);
  _rightPosSensor->enable(_timeStep);
  _ps0->enable(_timeStep);
  _ps1->enable(_timeStep);
  _ps6->enable(_timeStep);
  _ps7->enable(_timeStep);
  _imu->enable(_timeStep);

  _startHeading = 'N';
  _translation = _robot->getFromDef("Ghost_2")->getField("translation");
  _target_translation = _robot->getFromDef("Player")->getField("translation");
  _ghost1Translation = _robot->getFromDef("Ghost_1")->getField("translation");
  _ghost3Translation = _robot->getFromDef("Ghost_3")->getField("translation");
  
  _robot->simulationReset();
  while (_robot->step(_timeStep) != -1) {
    if (_robot->getTime() > 10) {
      break;
    }
  }
}

void Epuck::readMap() {
  // Create adjacency matrix from file
  std::ifstream infile(_mapFileName);

  std::string line;
  // Get first line of map which should tell what wall columns are present
  std::getline(infile, line);
  _map.push_back(line);

  // end with - 5 as map will always have last column and space
  for (auto i = 1; i <= line.length() - 5; ++i)
  {
    if (line[i] == ' ')
    {
      _wallCols.back() = 1;
    }
    else if (line[i] == '-')
    {
      _wallCols.push_back(0);
      i += 2;
    }
  }
  _numCols = _wallCols.size() + 1;

  while (std::getline(infile, line))
  {
    _map.push_back(line);
    if (line[0] == ' ')
    {
      _wallRows.back() = 1;
    }
    else if (line[0] == '|')
    {
      _wallRows.push_back(0);
    }
  }
  // While loop processed last row so pop
  _wallRows.pop_back();

  // For every cell, populate adjacency matrix with the directions that each cell can go
  for (auto cell = 0; cell < _numCols * (_wallRows.size() + 1); ++cell)
  {
    const auto row = cell / _numCols;
    const auto col = cell % _numCols;
    const auto rowPos = std::accumulate(_wallRows.begin(), _wallRows.begin() + row, row + 1);
    const auto colPos = std::accumulate(_wallCols.begin(), _wallCols.begin() + col, 3 * col + 2);

    std::vector<int> directions(4, 0);
    // Check in each direction to see if a wall exists to determine adjacency matrix
    if (_map[rowPos - 1][colPos] != '-')
    {
      directions.at(0) = 1;
    }
    if (_map[rowPos][colPos + 2] == ' ')
    {
      directions.at(1) = 1;
    }
    if (_map[rowPos + 1][colPos] != '-')
    {
      directions.at(2) = 1;
    }
    if (_map[rowPos][colPos - 2] == ' ')
    {
      directions.at(3) = 1;
    }
    _adjacencyMatrix.push_back(directions);
  }

    // Get target position for 1st time moving
  _end = calculateCell(_target_translation->getSFVec3f());
}

void Epuck::getPosition() {
  _start = calculateCell(_translation->getSFVec3f());
  // Only update target position when reached old position
  if (_start == _end) {
    _end = calculateCell(_target_translation->getSFVec3f());
  }
  _ghost1 = calculateCell(_ghost1Translation->getSFVec3f());
  _ghost3 = calculateCell(_ghost3Translation->getSFVec3f());
}

int Epuck::calculateCell(const double *values) {
  auto x = values[0];
  auto z = values[2];
  auto column = (int)((x + 1.5675)/TILESIZE);
  auto row = (int)((z + 1.485)/TILESIZE);
  return row * _numCols + column;
}

// Synchronous function from https://cyberbotics.com/doc/reference/motor
void Epuck::move1Step()
{
  const auto linearIncrement = 165 / _wheelRadius;
  auto currLeftPos = _leftPosSensor->getValue();
  auto currRightPos = _rightPosSensor->getValue();
  // Nan check for when robot hasn't moved
  auto newLeftPos = isnan(currLeftPos) ? linearIncrement : currLeftPos + linearIncrement;
  auto newRightPos = isnan(currRightPos) ? linearIncrement : currRightPos + linearIncrement;

  _leftMotor->setPosition(newLeftPos);
  _rightMotor->setPosition(newRightPos);
  _leftMotor->setVelocity(MAXSPEED);
  _rightMotor->setVelocity(MAXSPEED);
  double leftEffective;  // effective left position
  double rightEffective; // effective position position
  do
  {
    if (_robot->step(_timeStep) == -1)
      break;
    checkEdge();
    leftEffective = _leftPosSensor->getValue();
    rightEffective = _rightPosSensor->getValue();
    checkCollision();
  } while (fabs(newLeftPos - leftEffective) > STEPDELTA || fabs(newRightPos - rightEffective) > STEPDELTA);
}

void Epuck::checkCollision() {
  double ps0;
  double ps1;
  double ps6;
  double ps7;
  do {
    if (_robot->step(_timeStep) == -1) break; 
    ps0 = _ps0->getValue();
    ps1 = _ps1->getValue();
    ps6 = _ps6->getValue();
    ps7 = _ps7->getValue();
  } while (ps0 > 80 || ps1 > 80 || ps6 > 80 || ps7 > 80);
}

void Epuck::rotate90(char direction)
{
  const auto turningIncrement = PI / 2 / _wheelRadius * _axleLength;
  auto currLeftPos = _leftPosSensor->getValue();
  auto currRightPos = _rightPosSensor->getValue();
  // Nan check for when robot hasn't moved
  auto newLeftPos = isnan(currLeftPos) ? 0 : currLeftPos;
  auto newRightPos = isnan(currRightPos) ? 0 : currRightPos;

  auto yaw = (_imu->getRollPitchYaw())[2];
  if (direction == 'L')
  {
    yaw += PI / 2;
    if (yaw > PI)
    {
      yaw -= 2 * PI;
    }
    newLeftPos -= turningIncrement;
    newRightPos += turningIncrement;
  }
  else if (direction == 'R')
  {
    yaw -= PI / 2;
    if (yaw < -PI)
    {
      yaw += 2 * PI;
    }
    newLeftPos += turningIncrement;
    newRightPos -= turningIncrement;
  }
  _leftMotor->setPosition(newLeftPos);
  _rightMotor->setPosition(newRightPos);
  _leftMotor->setVelocity(0.2 * MAXSPEED);
  _rightMotor->setVelocity(0.2 * MAXSPEED);

  double effective;
  double leftEffective;
  double rightEffective;
  do
  {
    if (_robot->step(_timeStep) == -1)
      break;
    effective = (_imu->getRollPitchYaw())[2];
    leftEffective = _leftPosSensor->getValue();
    rightEffective = _rightPosSensor->getValue();
  } while (fabs(newLeftPos - leftEffective) > STEPDELTA || fabs(newRightPos - rightEffective) > STEPDELTA || fabs(yaw - effective) > _imu->getNoise());
}

void Epuck::printPaths()
{
  auto numPaths = 0;
  int leastTurns = INFINITY;
  _shortestPath.clear();
  _shortestMap.clear();

  for (const auto &path : _paths)
  {
    // Make a copy of the map to modify
    auto map = _map;
    auto step = path.size() - 2;
    // First cell is always robot initial position
    auto numTurns = 0;
    auto currRow = _start / _numCols;
    auto currCol = _start % _numCols;
    auto vertical = (_startHeading == 'N' || _startHeading == 'S') ? true : false;
    // Need to consider 180 degree turn for initial position
    auto nextRow = path.at(1) / _numCols;
    auto nextCol = path.at(1) % _numCols;
    if (currCol == nextCol)
    {
      // Moving vertically, if facing North and move down or South and move up
      // add 2 turns
      if ((_startHeading == 'N' && nextRow == currRow + 1) || (_startHeading == 'S' && nextRow == currRow - 1))
      {
        numTurns += 2;
      }
    }
    else if (currRow == nextRow)
    {
      // Moving vertically, if facing West and doesn't move left or East and doesn't move right
      // add 2 turns
      if ((_startHeading == 'W' && nextCol == nextCol + 1) || (_startHeading == 'E' && nextCol == currCol - 1))
      {
        numTurns += 2;
      }
    }
    for (auto cell = path.begin() + 1; cell < path.end(); ++cell)
    {
      const auto row = *cell / _numCols;
      const auto col = *cell % _numCols;
      const auto rowPos = std::accumulate(_wallRows.begin(), _wallRows.begin() + row, row + 1);
      const auto colPos = std::accumulate(_wallCols.begin(), _wallCols.begin() + col, 3 * col + 2);
      // Replace copied map with path numbers
      if (step >= 10)
      {
        map[rowPos][colPos] = std::to_string(step / 10)[0];
        map[rowPos][colPos + 1] = std::to_string(step % 10)[0];
      }
      else
      {
        map[rowPos][colPos] = std::to_string(step)[0];
      }
      --step;
      // Check if change in direction
      if (vertical && currCol != col)
      {
        vertical = false;
        ++numTurns;
      }
      else if (!vertical && currRow != row)
      {
        vertical = true;
        ++numTurns;
      }
      currRow = row;
      currCol = col;
    }
    ++numPaths;

    // If this path has the least number of turns, assign shortest path/map to this
    if (numTurns < leastTurns)
    {
      _shortestPath = path;
      _shortestMap = map;
      leastTurns = numTurns;
    }
  }
}

void Epuck::checkCell(int cell, int next, std::queue<int> &queue, std::vector<int> &dist, std::vector<std::vector<int>> &parent)
{
  // Check if path will be blocked by another ghost robot
  if (next == _ghost1 || next == _ghost3) {
    return;
  }
  if (dist.at(next) > dist.at(cell) + 1)
  {
    dist.at(next) = dist.at(cell) + 1;
    queue.push(next);
    parent[next].clear();
    parent[next].push_back(cell);
  }
  else if (dist.at(next) == dist.at(cell) + 1)
  {
    parent[next].push_back(cell);
  }
}

void Epuck::getPaths(std::vector<int> &path, std::vector<std::vector<int>> &parent, int curr)
{
  // Base case as we assigned
  if (curr == -1)
  {
    _paths.push_back(path);
    return;
  }
  for (const auto &cell : parent[curr])
  {
    path.push_back(cell);
    getPaths(path, parent, cell);
    path.pop_back();
  }
}

// Referenced from https://www.geeksforgeeks.org/print-all-shortest-paths-between-given-source-and-destination-in-an-undirected-graph/
void Epuck::generatePaths()
{
  auto mapSize = _numCols * (_wallRows.size() + 1);
  std::queue<int> queue;
  std::vector<std::vector<int>> parent(mapSize, std::vector<int>());
  std::vector<int> dist(mapSize, INT_MAX);
  _paths.clear();

  queue.push(_start);
  parent.at(_start) = {-1};
  dist.at(_start) = 0;

  while (!queue.empty())
  {
    auto cell = queue.front();
    queue.pop();
    // For each direction (N,E,S,W), check if adjacent cell can be added
    auto directions = _adjacencyMatrix.at(cell);
    if (directions.at(0) == 1)
    {
      checkCell(cell, cell - _numCols, queue, dist, parent);
    }
    if (directions.at(1) == 1)
    {
      checkCell(cell, cell + 1, queue, dist, parent);
    }
    if (directions.at(2) == 1)
    {
      checkCell(cell, cell + _numCols, queue, dist, parent);
    }
    if (directions.at(3) == 1)
    {
      checkCell(cell, cell - 1, queue, dist, parent);
    }
  }

  std::vector<int> path;
  getPaths(path, parent, _end);

  // For each path, remove source node parent of -1,
  // reverse for start to end path and append end node
  for (auto &routes : _paths)
  {
    routes.pop_back();
    std::reverse(routes.begin(), routes.end());
    routes.push_back(_end);
  }
}

void Epuck::generateMotion()
{
  if (_shortestPath.empty()) return;
  auto currRow = _start / _numCols;
  auto currCol = _start % _numCols;

  auto cell = _shortestPath.begin() + 1;
  const auto nextRow = *cell / _numCols;
  const auto nextCol = *cell % _numCols;

  // Moving horizontally
  if (nextRow == currRow)
  {
    // Moving right
    if (nextCol == currCol + 1)
    {
      switch (_startHeading) {
        case 'N':
          followPath('R');
        break;
        case 'E':
          followPath('F');
        break;
        case 'S':
        case 'W':
          followPath('L');
        break;
        default:
          std::cout << "Switch heading failed" << std::endl;
        break;
      }
    }
    // Moving left
    else if (nextCol == currCol - 1)
    {
      switch (_startHeading) {
        case 'N':
        case 'E':
          followPath('L');
        break;
        case 'S':
          followPath('R');
        break;
        case 'W':
          followPath('F');
        break;
        default:
          std::cout << "Switch heading failed" << std::endl;
        break;
      }
    }
  }
  // Moving vertically
  else if (nextCol == currCol)
  {
    // Moving up
    if (nextRow == currRow - 1) {
      switch (_startHeading) {
        case 'N':
          followPath('F');
        break;
        case 'E':
        case 'S':
          followPath('L');
        break;
        case 'W':
          followPath('R');
        break;
        default:
          std::cout << "Switch heading failed" << std::endl;
        break;
      }
    } 
    // Moving down
    else if (nextRow == currRow + 1) {
      switch (_startHeading) {
        case 'N':
        case 'W':
          followPath('L');
        break;
        case 'E':
          followPath('R');
        break;
        case 'S':
          followPath('F');
        break;
        default:
          std::cout << "Switch heading failed" << std::endl;
        break;
      }
    }
  }
}

void Epuck::followPath(char instruction){
  if (instruction == 'F')
  {
    move1Step();
  }
  else if (instruction == 'L')
  {
    rotate90('L');
    switch (_startHeading)
    {
    case 'N':
      _startHeading = 'W';
      break;
    case 'S':
      _startHeading = 'E';
      break;
    case 'E':
      _startHeading = 'N';
      break;
    case 'W':
      _startHeading = 'S';
      break;
    default:
      std::cout << " Warning: Move 'L' has the wrong heading." << std::endl;
    }
  }
  else if (instruction == 'R')
  {
    rotate90('R');
    switch (_startHeading)
    {
    case 'N':
      _startHeading = 'E';
      break;
    case 'S':
      _startHeading = 'W';
      break;
    case 'E':
      _startHeading = 'S';
      break;
    case 'W':
      _startHeading = 'N';
      break;
    default:
      std::cout << " Warning: Move 'R' has the wrong heading." << std::endl;
    }
  }
}


void Epuck::follow()
{
  while (_robot->step(_timeStep) != -1) {
    getPosition();
    generatePaths();
    printPaths();
    generateMotion();
  }
}

void Epuck::checkEdge() {
  const auto values = _translation->getSFVec3f();
  const auto x = values[0];
  const auto y = values[1];
  const auto z = values[2];
  // If position of robot is at teleport edges, change position to other side.
  if (x >= 1.485 && z >= 0 && z <= 0.165) {
    const double location[3] = {-1.485, y, z};
    _translation->setSFVec3f(location);
  } else if (x <= -1.485 && z >= 0 && z <= 0.165) {
    const double location[3] = {1.485, y, z};
    _translation->setSFVec3f(location);
  }
}

int main(int argc, char **argv) {
  const std::string MAP_FILE_NAME = "../../PacMan.txt";

  Epuck Epuck{MAP_FILE_NAME, WHEELRADIUS, AXLELENGTH};
  Epuck.readMap();
  Epuck.follow();

  return 0;
}
