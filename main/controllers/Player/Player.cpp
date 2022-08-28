// File:          Player.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <vector>
#include <cmath>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Supervisor.hpp>

#define MAXSPEED 6.28
#define ROBOTDIAMETER 0.074

using namespace webots;

class Epuck
{
public:
  Epuck();
  void changeSpeed(double, double);
  int getKey();
  int step();

private:
  Supervisor *_robot;
  Motor *_leftMotor;
  Motor *_rightMotor;
  Keyboard *_keyboard;
  Field *_translation;
  int _timeStep;
  int _score;
  std::vector<Field *> _pointsTransparency;
  std::vector<Field *> _pointsLocations;
  int _numPointsTouched;
  int _lives;
  Field *_ghost1;
  Field *_ghost2;
  Field *_ghost3;
  bool checkCollision(double, double, Field*);
  void resetPositions();
  void restartWorld();
};

Epuck::Epuck()
{
  // Initialise variables
  _robot = new Supervisor();
  _timeStep = (int)_robot->getBasicTimeStep();
  _score = 0;
  _numPointsTouched = 172;
  _lives = 3;

  _leftMotor = _robot->getMotor("left wheel motor");
  _rightMotor = _robot->getMotor("right wheel motor");

  _keyboard = _robot->getKeyboard();
  _keyboard->enable(_timeStep);
  
  _translation = _robot->getFromDef("Player")->getField("translation");
  _ghost1 = _robot->getFromDef("Ghost_1")->getField("translation");
  _ghost2 = _robot->getFromDef("Ghost_2")->getField("translation");
  _ghost3 = _robot->getFromDef("Ghost_3")->getField("translation");

  auto maze = _robot->getFromDef("MAZE");
  auto children = maze->getField("children");
  // Store all the point locations and transparency for changing
  for (auto n = 0; n <= 171; ++n) {
    auto point = children->getMFNode(n);

    auto appearance = point->getField("children")->getMFNode(0)->getField("appearance");
    auto transparency = appearance->getSFNode()->getField("transparency");
    transparency->setSFFloat(0.0);

    _pointsTransparency.push_back(transparency);
    _pointsLocations.push_back(point->getField("translation"));
  }

  _robot->setLabel(0, "Score: 00", 0, 0, 0.05, 0xffffff, 0, "Arial");
  _robot->setLabel(1, "Lives: " + std::to_string(_lives), 0, 0.02, 0.05, 0xffffff, 0, "Arial");
}

void Epuck::changeSpeed(double leftSpeed, double rightSpeed)
{
  _leftMotor->setPosition(INFINITY);
  _rightMotor->setPosition(INFINITY);
  _leftMotor->setVelocity(leftSpeed);
  _rightMotor->setVelocity(rightSpeed);
}

int Epuck::getKey()
{
  return _keyboard->getKey();
}

int Epuck::step()
{
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
  // Check if any points have been passed over
  for (auto point = _pointsLocations.begin(); point < _pointsLocations.end(); ++point) {
    auto loc = (*point)->getSFVec3f();
    const auto lowerX = loc[0] - 0.05;
    const auto upperX = loc[0] + 0.05;
    const auto lowerZ = loc[2] - 0.05;
    const auto upperZ = loc[2] + 0.05;
    if (x >= lowerX && x <= upperX && z >= lowerZ && z <= upperZ) {
      auto transparency = _pointsTransparency.at(point - _pointsLocations.begin());
      if (transparency->getSFFloat() == 0) {
        _score += 50;
          _robot->setLabel(0, "Score: " + std::to_string(_score), 0, 0, 0.05, 0xffffff, 0, "Arial");
        transparency->setSFFloat(0.8);
        ++_numPointsTouched;
        if (_numPointsTouched == _pointsLocations.size()) {
          restartWorld();
        }
      }
    }
  }

  if (checkCollision(x, z, _ghost1) || checkCollision(x, z, _ghost2) || checkCollision(x, z, _ghost3)) {
    --_lives;
    resetPositions();
    if (_lives <= 0) {
      // game over
      _robot->simulationSetMode(Supervisor::SIMULATION_MODE_PAUSE);
    }
    _robot->setLabel(1, "Lives: " + std::to_string(_lives), 0, 0.02, 0.05, 0xffffff, 0, "Arial");
  }

  return _robot->step(_timeStep);
}

bool Epuck::checkCollision (double x, double z, Field *ghost) {
  const auto values = ghost->getSFVec3f();
  const auto distX = x - values[0];
  const auto distZ = z - values[2];
  // Calculate hypotenuse
  const auto minDistance = std::sqrt(std::pow(distX, 2) + std::pow(distZ, 2));
  if (minDistance <= ROBOTDIAMETER) {
    return true;
  }
  return false;
}

void Epuck::resetPositions() {
  // Restart all epucks
  changeSpeed(0, 0);
  _robot->simulationReset();
  _robot->getFromDef("Ghost_1")->restartController();
  _robot->getFromDef("Ghost_2")->restartController();
  _robot->getFromDef("Ghost_3")->restartController();
}

void Epuck::restartWorld() {
  resetPositions();
  // Set all points back to untouched
  for (auto n = _pointsTransparency.begin(); n < _pointsTransparency.end(); ++n) {
    (*n)->setSFFloat(0);
  }
  _numPointsTouched = 0;
}

int main(int argc, char **argv) {
  Epuck Epuck{};
  int key{-1};

  while (Epuck.step() != -1) {
    key = Epuck.getKey();
    if (key == ' ')
    {
      Epuck.changeSpeed(0, 0);
    }
    else if (key == 'W' || key == Keyboard::UP)
    {
      Epuck.changeSpeed(0.9 * MAXSPEED, 0.9 * MAXSPEED);
    }
    else if (key == 'S' || key == Keyboard::DOWN)
    {
      Epuck.changeSpeed(-0.9 * MAXSPEED, -0.9 * MAXSPEED);
    }
    else if (key == 'A' || key == Keyboard::LEFT)
    {
      Epuck.changeSpeed(-0.4 * MAXSPEED, 0.4 * MAXSPEED);
      
    }
    else if (key == 'D' || key == Keyboard::RIGHT)
    {
      Epuck.changeSpeed(0.4 * MAXSPEED, -0.4 * MAXSPEED);
    }
  };

  return 0;
}
