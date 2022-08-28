#include <fstream>
#include <string>
#include <iomanip>
#include <cmath>
#include <vector>

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
#define ROWS 5
#define COLUMNS 9
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// All the webots classes are defined in the "webots" namespace
using namespace webots;

class Epuck
{
public:
    Epuck(std::string, std::string, std::string, double, double);
    void readMap();
    void followPlan();
    void move_one_step();
    void rotate_90(char direction);

private:
    void printPath(std::vector<int>, int, std::vector<std::string>);
    void floodFill(int[ROWS * COLUMNS], int, int[ROWS + 1][COLUMNS], int[ROWS][COLUMNS + 1]);
    void findPaths(std::vector<std::vector<int>> &, std::vector<int>, int[ROWS * COLUMNS], int, int, int[ROWS + 1][COLUMNS], int[ROWS][COLUMNS + 1]);
    std::string pathPlanner(std::vector<int>, char);
    Supervisor *_robot;
    Motor *_leftMotor;
    Motor *_rightMotor;
    PositionSensor *_leftPosSensor;
    PositionSensor *_rightPosSensor;
    InertialUnit *_imu;
    DistanceSensor *_leftDistSensor;
    DistanceSensor *_frontDistSensor;
    DistanceSensor *_rightDistSensor;
    std::string _mapFileName;
    std::string _instructions;
    std::string _csvFileName;
    std::string _robotName;
    int _timeStep;
    double _wheelRadius;
    double _axleLength;
};

Epuck::Epuck(std::string mapFileName, std::string csvFileName, std::string robotName, double wheelRadius, double axleLength)
    : _mapFileName(mapFileName), _csvFileName(csvFileName), _robotName(robotName), _wheelRadius(wheelRadius), _axleLength(axleLength)
{
    // Initialise variables
    _robot = new Supervisor();
    _leftMotor = _robot->getMotor("left wheel motor");
    _rightMotor = _robot->getMotor("right wheel motor");
    _leftPosSensor = _robot->getPositionSensor("left wheel sensor");
    _rightPosSensor = _robot->getPositionSensor("right wheel sensor");
    _imu = _robot->getInertialUnit("IMU");
    _leftDistSensor = _robot->getDistanceSensor("dsL");
    _frontDistSensor = _robot->getDistanceSensor("dsF");
    _rightDistSensor = _robot->getDistanceSensor("dsR");
    _timeStep = (int)_robot->getBasicTimeStep();

    // Enable all the sensors
    _leftPosSensor->enable(_timeStep);
    _rightPosSensor->enable(_timeStep);
    _imu->enable(_timeStep);
    _leftDistSensor->enable(_timeStep);
    _frontDistSensor->enable(_timeStep);
    _rightDistSensor->enable(_timeStep);
}

void Epuck::readMap()
{
    std::vector<std::string> map;
    int horizontalWalls[ROWS + 1][COLUMNS];
    int verticalWalls[ROWS][COLUMNS + 1];
    char heading = '^';
    int startNode = 0;
    int goalNode = 0;

    std::cout << _robotName << " Reading in map from " << _mapFileName << "..." << std::endl;

    // Open map TXT file to read in from.
    std::ifstream file(_mapFileName);
    std::string line;
    int i = 0;
    while (std::getline(file, line))
    {
        // Append to map.
        map.push_back(line);

        std::cout << _robotName << " " << line << std::endl;

        // Horizontal wall lines.
        if (i % 2 == 0)
        {
            int col = 0;
            while (col < COLUMNS)
            {
                // .compare(position, lenght of string, string) == 0 for match
                if (line.compare(1 + col * 4, 3, "---") == 0)
                {
                    horizontalWalls[i / 2][col] = 1;
                }
                else
                {
                    horizontalWalls[i / 2][col] = 0;
                }
                col++;
            }
            // Vertical wall lines.
        }
        else
        {
            int col = 0;
            while (col < COLUMNS + 1)
            {
                if (line.compare(col * 4, 1, "|") == 0)
                {
                    verticalWalls[i / 2][col] = 1;
                }
                else
                {
                    verticalWalls[i / 2][col] = 0;
                }
                // If 'x', set goal node, else if '<^v>' and not a null terminator, set start node.
                // std::cout << "pos: " << 2 + col*4 << "\tsize: " << line.size() << std::endl;
                if (2 + col * 4 < 38)
                {
                    if (line.compare(2 + col * 4, 1, "x") == 0)
                    {
                        goalNode = (i / 2) * COLUMNS + col;
                    }
                    // .find_first_of(chars to look for, position)
                    else if (line.find_first_of("<^v>", 2 + col * 4) != std::string::npos)
                    {
                        startNode = (i / 2) * COLUMNS + col;
                        heading = line[2 + col * 4];
                    }
                }

                col++;
            }
        }
        i++;
    }
    file.close();

    std::cout << _robotName << " Map read in!" << std::endl;
    std::cout << _robotName << " Finding shortest paths..." << std::endl;

    // Store node values.
    int cellValues[ROWS * COLUMNS];
    // Run flood filled algorithm to fill cellValues array.
    floodFill(cellValues, goalNode, horizontalWalls, verticalWalls);

    // Store all shortest paths and current path.
    std::vector<std::vector<int>> paths;
    std::vector<int> path;
    // Find all shortest paths.
    findPaths(paths, path, cellValues, cellValues[startNode], startNode, horizontalWalls, verticalWalls);

    // Print all shortest paths found to map and compare their path plans to find the shortest path with the least turns.
    i = 1;
    std::string shortestPathPlan;
    int shortestPathPlanIndex = 0;
    for (auto path : paths)
    {
        // Print path.
        printPath(path, i, map);
        // Get paths plan.
        std::string currentPathsPlan = pathPlanner(path, heading);
        // Compare with current shortest path and update if shorter.
        if ((i == 1) || (currentPathsPlan.length() <= shortestPathPlan.length()))
        {
            shortestPathPlan = currentPathsPlan;
            shortestPathPlanIndex = i - 1;
        }
        i++;
    }

    std::cout << _robotName << " " << i - 1 << " shortest paths found!" << std::endl;
    std::cout << _robotName << " "
              << "Finding shortest path with least turns..." << std::endl;

    i = 0;
    printPath(paths[shortestPathPlanIndex], i, map);

    std::cout << _robotName << " Shortest path with least turns found!" << std::endl;
    std::cout << _robotName << " Path Plan (" << shortestPathPlan.length() - 3 << " steps): " << shortestPathPlan << std::endl;

    _instructions = shortestPathPlan;
    std::cout << "_instructions are: " << _instructions << std::endl;
}

void Epuck::printPath(std::vector<int> path, int i, std::vector<std::string> map)
{
    // Print the path number if not the shortest path with hte least turns.
    if (i != 0)
    {
        std::cout << _robotName << " Path - " << i << ":" << std::endl;
    }
    int length = path.size();
    for (int i = 1; i < length; i++)
    {
        // Get the current step of the node and convert it to a string.
        int step = length - i - 1;
        std::string stepString = std::to_string(step);
        // If the step value is a single digit add a space after it.
        if ((step >= 0) && (step <= 9))
        {
            stepString.push_back(' ');
        }
        // Update to map replacing the centre and respective right cell.
        map[(path[i] / COLUMNS) * 2 + 1].replace((path[i] % COLUMNS) * 4 + 2, 2, stepString);
    }
    for (auto line : map)
    {
        std::cout << _robotName << " " << line << std::endl;
    }
}

void Epuck::floodFill(int cellValues[ROWS * COLUMNS], int goalNode, int horizontalWalls[ROWS + 1][COLUMNS], int verticalWalls[ROWS][COLUMNS + 1])
{
    int node = 0;
    // Fill cell values with a large number and goal node with 0.
    while (node < ROWS * COLUMNS)
    {
        if (node == goalNode)
        {
            cellValues[node] = 0;
        }
        else
        {
            cellValues[node] = ROWS * COLUMNS;
        }
        node++;
    }
    int currentExploredValue = 0;
    int mazeValueChanged = 1;
    while (mazeValueChanged != 0)
    {
        mazeValueChanged = 0;
        // For all nodes - replaces need for row and column loop.
        for (int node = 0; node < ROWS * COLUMNS; node++)
        {
            // If current node value equals current explored value.
            if (cellValues[node] == currentExploredValue)
            {
                // For all directions of node.
                for (int direction = 0; direction < 4; direction++)
                {
                    if (direction == NORTH)
                    {
                        // If neighbouring wall does not exist in direction.
                        if (horizontalWalls[node / COLUMNS][node % COLUMNS] == 0)
                        {
                            // If neighbouring cell in direction equals ROWS*COLUMNS.
                            if (cellValues[node - COLUMNS] == ROWS * COLUMNS)
                            {
                                // Update node value.
                                cellValues[node - COLUMNS] = currentExploredValue + 1;
                                mazeValueChanged = 1;
                            }
                        }
                    }
                    else if (direction == EAST)
                    {
                        if (verticalWalls[node / COLUMNS][node % COLUMNS + 1] == 0)
                        {
                            if (cellValues[node + 1] == ROWS * COLUMNS)
                            {
                                cellValues[node + 1] = currentExploredValue + 1;
                                mazeValueChanged = 1;
                            }
                        }
                    }
                    else if (direction == SOUTH)
                    {
                        if (horizontalWalls[node / COLUMNS + 1][node % COLUMNS] == 0)
                        {
                            if (cellValues[node + COLUMNS] == ROWS * COLUMNS)
                            {
                                cellValues[node + COLUMNS] = currentExploredValue + 1;
                                mazeValueChanged = 1;
                            }
                        }
                    }
                    else if (direction == WEST)
                    {
                        if (verticalWalls[node / COLUMNS][node % COLUMNS] == 0)
                        {
                            if (cellValues[node - 1] == ROWS * COLUMNS)
                            {
                                cellValues[node - 1] = currentExploredValue + 1;
                                mazeValueChanged = 1;
                            }
                        }
                    }
                }
            }
        }
        currentExploredValue++;
    }
}

void Epuck::findPaths(std::vector<std::vector<int>> &paths, std::vector<int> path, int cellValues[ROWS * COLUMNS], int step, int currentNode, int horizontalWalls[ROWS + 1][COLUMNS], int verticalWalls[ROWS][COLUMNS + 1])
{
    // Until reaching the goal node.
    while (step > 0)
    {
        step--;
        // Add to path.
        path.push_back(currentNode);
        int nextNode = currentNode;
        // Flag condition for if current path splits off into 2 viable short paths.
        int single_path = 1;
        // For all directions of node.
        for (int direction = 0; direction < 4; direction++)
        {
            if (direction == NORTH)
            {
                // If neighbouring wall does not exist in direction.
                if (horizontalWalls[currentNode / COLUMNS][currentNode % COLUMNS] == 0)
                {
                    // If neighbouring cell in direction equals next step.
                    if (cellValues[currentNode - COLUMNS] == step)
                    {
                        // Check if flag has been triggered.
                        if (single_path)
                        {
                            // If not, get next node.
                            nextNode = currentNode - COLUMNS;
                            single_path = 0;
                            // Otherwise another path is available, call the function down that path direction then return to continue current path.
                        }
                        else
                        {
                            findPaths(paths, path, cellValues, step, (currentNode - COLUMNS), horizontalWalls, verticalWalls);
                        }
                    }
                }
            }
            else if (direction == EAST)
            {
                if (verticalWalls[currentNode / COLUMNS][currentNode % COLUMNS + 1] == 0)
                {
                    if (cellValues[currentNode + 1] == step)
                    {
                        if (single_path)
                        {
                            nextNode = currentNode + 1;
                            single_path = 0;
                        }
                        else
                        {
                            findPaths(paths, path, cellValues, step, (currentNode + 1), horizontalWalls, verticalWalls);
                        }
                    }
                }
            }
            else if (direction == SOUTH)
            {
                if (horizontalWalls[currentNode / COLUMNS + 1][currentNode % COLUMNS] == 0)
                {
                    if (cellValues[currentNode + COLUMNS] == step)
                    {
                        if (single_path)
                        {
                            nextNode = currentNode + COLUMNS;
                            single_path = 0;
                        }
                        else
                        {
                            findPaths(paths, path, cellValues, step, (currentNode + COLUMNS), horizontalWalls, verticalWalls);
                        }
                    }
                }
            }
            else if (direction == WEST)
            {
                if (verticalWalls[currentNode / COLUMNS][currentNode % COLUMNS] == 0)
                {
                    if (cellValues[currentNode - 1] == step)
                    {
                        if (single_path)
                        {
                            nextNode = currentNode - 1;
                            single_path = 0;
                        }
                        else
                        {
                            findPaths(paths, path, cellValues, step, (currentNode - 1), horizontalWalls, verticalWalls);
                        }
                    }
                }
            }
        }
        // Update node for next.
        currentNode = nextNode;
    }
    // Once the goal node is reached it store the path and returns to caller.
    path.push_back(currentNode);
    paths.push_back(path);
}

std::string Epuck::pathPlanner(std::vector<int> path, char heading)
{
    std::string pathPlan;
    // Appends Epucks starting row, column and heading.
    // .append(size_t n, char c), int to char - https://stackoverflow.com/questions/4629050/convert-an-int-to-ascii-character
    pathPlan.append(1, '0' + path[0] / COLUMNS);
    pathPlan.append(1, '0' + path[0] % COLUMNS);

    int currentHeading;
    if (heading == '^')
    {
        pathPlan.append("N");
        currentHeading = NORTH;
    }
    else if (heading == '>')
    {
        pathPlan.append("E");
        currentHeading = EAST;
    }
    else if (heading == 'v')
    {
        pathPlan.append("S");
        currentHeading = SOUTH;
    }
    else if (heading == '<')
    {
        pathPlan.append("W");
        currentHeading = WEST;
    }

    // For the number of nodes.
    int currentNode = path[0];
    for (auto nextNode : path)
    {
        int nextHeading = currentHeading;
        // Ignores starting node
        if (nextNode != path[0])
        {
            // Figure out the heading at the next node based on the difference between nodes.
            if (nextNode - currentNode == -COLUMNS)
            {
                nextHeading = NORTH;
            }
            else if (nextNode - currentNode == 1)
            {
                nextHeading = EAST;
            }
            else if (nextNode - currentNode == COLUMNS)
            {
                nextHeading = SOUTH;
            }
            else if (nextNode - currentNode == -1)
            {
                nextHeading = WEST;
            }

            // Based on the differce between headings, set the motions required.
            if ((nextHeading - currentHeading == -1) || (nextHeading - currentHeading == 3))
            {
                pathPlan.append("LF");
            }
            else if ((nextHeading - currentHeading == 1) || (nextHeading - currentHeading == -3))
            {
                pathPlan.append("RF");
            }
            else if (nextHeading - currentHeading == 0)
            {
                pathPlan.append("F");
            }
            else if ((nextHeading - currentHeading == 2) || (nextHeading - currentHeading == -2))
            {
                pathPlan.append("LLF");
            }
        }
        currentNode = nextNode;
        currentHeading = nextHeading;
    }
    return pathPlan;
}

void Epuck::followPlan()
{
    auto step = 0;
    auto currLeftPos = 0;
    auto currRightPos = 0;
    auto newLeftPos = 0;
    auto newRightPos = 0;
    int row = _instructions[0] - '0';
    int col = _instructions[1] - '0';
    auto heading = _instructions[2];
    std::string::size_type instruct_step = 3;

    // Clear/create csv file
    std::ofstream fout{_csvFileName, std::ios::out};
    // check if open successfully
    if (fout.is_open())
    {
        fout << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall" << std::endl;
    }
    else
    {
        // throw an error message
        throw std::runtime_error(_robotName + " Creating " + _csvFileName + " failed!\n");
    }

    std::cout << _robotName << " Executing motion plan..." << std::endl;

    // Start recording
    // void wb_supervisor_movie_start_recording(const char *filename, int width, int height, int codec, int quality, int acceleration, bool caption);
    std::cout << _robotName << " Starting recording!" << std::endl;
    _robot->movieStartRecording("../../RobotRecording.mp4", 800, 600, 0, 35, 4, false);

    while (_robot->step(_timeStep) != -1)
    {
        // Take multiple samples and apply Moving Average
        auto i = 0;
        auto leftValue = 0;
        auto frontValue = 0;
        auto rightValue = 0;
        do
        {
            if (_robot->step(_timeStep) == -1)
                break;
            leftValue += _leftDistSensor->getValue();
            frontValue += _frontDistSensor->getValue();
            rightValue += _rightDistSensor->getValue();
            ++i;
        } while (i < 3);

        auto left = (leftValue / 3 < 700) ? 'Y' : 'N';
        auto front = (frontValue / 3 < 700) ? 'Y' : 'N';
        auto right = (rightValue / 3 < 700) ? 'Y' : 'N';

        std::cout << _robotName << " Step: " << std::setw(3) << std::setfill('0') << step;
        std::cout << ", Row: " << row << ", Column: " << col << ", Heading: " << heading;
        std::cout << ", Left Wall: " << left << ", Front Wall: " << front << ", Right Wall: " << right << std::endl;

        // Append data to csv
        std::ofstream fout{_csvFileName, std::ios::out | std::ios::app}; // open file in append mode
        // check if open successfully
        if (fout.is_open())
        {
            fout << step << "," << row << "," << col << "," << heading << "," << left << "," << front << "," << right << std::endl;
        }
        else
        {
            // throw an error message
            throw std::runtime_error(_robotName + " Writing to " + _csvFileName + " failed!\n");
        }

        // If the current step is the last step, quit the loop and exit
        if (instruct_step == _instructions.length())
        {
            break;
        }
        char curr_step = _instructions[instruct_step];
        ++instruct_step;

        if (curr_step == 'F')
        {
            const auto linearIncrement = 165 / _wheelRadius;
            currLeftPos = _leftPosSensor->getValue();
            currRightPos = _rightPosSensor->getValue();
            _leftMotor->setVelocity(MAXSPEED);
            _rightMotor->setVelocity(MAXSPEED);
            newLeftPos = isnan(currLeftPos) ? linearIncrement : currLeftPos + linearIncrement;
            newRightPos = isnan(currRightPos) ? linearIncrement : currRightPos + linearIncrement;

            double leftEffective;  // effective left position
            double rightEffective; // effective position position
            do
            {
                if (_robot->step(_timeStep) == -1)
                    break;
                leftEffective = _leftPosSensor->getValue();
                rightEffective = _rightPosSensor->getValue();
            } while (fabs(newLeftPos - leftEffective) > STEPDELTA || fabs(newRightPos - rightEffective) > STEPDELTA);

            if (_instructions[instruct_step + 1] == 'F')
            {
                newLeftPos = currLeftPos + linearIncrement;
                newRightPos = currRightPos + linearIncrement;
                instruct_step++;
            }

            switch (heading)
            {
            case 'N':
                --row;
                break;
            case 'S':
                ++row;
                break;
            case 'E':
                ++col;
                break;
            case 'W':
                --col;
                break;
            default:
                std::cout << _robotName << " Warning: Move 'F' has the wrong heading." << std::endl;
            }
            _leftMotor->setPosition(newLeftPos);
            _rightMotor->setPosition(newRightPos);
        }
        else if (curr_step == 'L')
        {
            const auto turningIncrement = PI / 2 / _wheelRadius * _axleLength;
            auto yaw = (_imu->getRollPitchYaw())[2];
            currLeftPos = _leftPosSensor->getValue();
            currRightPos = _rightPosSensor->getValue();
            _leftMotor->setVelocity(0.2 * MAXSPEED);
            _rightMotor->setVelocity(0.2 * MAXSPEED);
            newLeftPos -= turningIncrement;
            newRightPos += turningIncrement;

            if (_instructions[instruct_step + 1] == 'L')
            {
                newLeftPos = currLeftPos - turningIncrement;
                newRightPos = currRightPos + turningIncrement;
                instruct_step++;
            }

            yaw += PI / 2;
            if (yaw > PI)
            {
                yaw -= 2 * PI;
            }

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

            switch (heading)
            {
            case 'N':
                heading = 'W';
                break;
            case 'S':
                heading = 'E';
                break;
            case 'E':
                heading = 'N';
                break;
            case 'W':
                heading = 'S';
                break;
            default:
                std::cout << _robotName << " Warning: Move 'L' has the wrong heading." << std::endl;
            }
            _leftMotor->setPosition(newLeftPos);
            _rightMotor->setPosition(newRightPos);
        }
        else if (curr_step == 'R')
        {
            const auto turningIncrement = PI / 2 / _wheelRadius * _axleLength;
            auto yaw = (_imu->getRollPitchYaw())[2];
            currLeftPos = _leftPosSensor->getValue();
            currRightPos = _rightPosSensor->getValue();
            _leftMotor->setVelocity(0.2 * MAXSPEED);
            _rightMotor->setVelocity(0.2 * MAXSPEED);
            newLeftPos += turningIncrement;
            newRightPos -= turningIncrement;

            if (_instructions[instruct_step + 1] == 'R')
            {
                newLeftPos = currLeftPos + turningIncrement;
                newRightPos = currRightPos - turningIncrement;
                instruct_step++;
            }

            yaw -= PI / 2;
            if (yaw < -PI)
            {
                yaw += 2 * PI;
            }

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

            switch (heading)
            {
            case 'N':
                heading = 'E';
                break;
            case 'S':
                heading = 'W';
                break;
            case 'E':
                heading = 'S';
                break;
            case 'W':
                heading = 'N';
                break;
            default:
                std::cout << _robotName << " Warning: Move 'R' has the wrong heading." << std::endl;
            }
            _leftMotor->setPosition(newLeftPos);
            _rightMotor->setPosition(newRightPos);
        }
        currLeftPos = currLeftPos + newLeftPos;
        currRightPos = currRightPos + newRightPos;
        ++step;
    };

    std::cout << _robotName << " Motion plan executed!" << std::endl;

    // Stop recording and pause simulation
    std::cout << _robotName << " Saving recording!" << std::endl;
    _robot->movieStopRecording();
    _robot->simulationSetMode(_robot->SIMULATION_MODE_PAUSE);

    delete _robot;
}

// Synchronous function from https://cyberbotics.com/doc/reference/motor
void Epuck::move_one_step()
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
        leftEffective = _leftPosSensor->getValue();
        rightEffective = _rightPosSensor->getValue();
    } while (fabs(newLeftPos - leftEffective) > STEPDELTA || fabs(newRightPos - rightEffective) > STEPDELTA);
}

void Epuck::rotate_90(char direction)
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

int main(int argc, char **argv)
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

    return 0;
}
