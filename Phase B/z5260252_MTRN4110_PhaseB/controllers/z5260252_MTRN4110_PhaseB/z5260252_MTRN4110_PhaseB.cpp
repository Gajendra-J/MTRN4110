/*
 * File:          z5260252_MTRN4110_PhaseB.cpp
 * Date:          26/06/2022
 * Description:   Controller of E-puck for Phase B - Path Planning
 * Author:        Gajendra Jayasekera (z5260252)
 * Modifications: 
 * Platform:      MacOS
 * Notes:         
 */    

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <algorithm>

#define ROWS 5
#define COLUMNS 9
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

const std::string COUT_PREFIX = "[z5260252_MTRN4110_PhaseB] ";
const std::string MAP_FILE_NAME = "../../Map.txt";
const std::string PATH_PLAN_FILE_NAME = "../../PathPlan.txt";
const std::string OUTPUT_FILE_NAME = "../../Output.txt";

void processMap(std::vector<std::string> &map, int horizontalWalls[ROWS + 1][COLUMNS], int verticalWalls[ROWS][COLUMNS + 1], char &heading, int &startNode, int &goalNode, std::ofstream &saveFile);
void floodFill(int cellValues[], int goalNode, int horizontalWalls[ROWS + 1][COLUMNS], int verticalWalls[ROWS][COLUMNS + 1]);
void findPaths(std::vector<std::vector<int>> &paths, std::vector<int> path, int cellValues[], int step, int currentNode, int horizontalWalls[ROWS + 1][COLUMNS], int verticalWalls[ROWS][COLUMNS + 1]);
void printPath(std::vector<int> path, int i, std::vector<std::string> map, std::ofstream &saveFile);
std::string pathPlanner(std::vector<int> path, char heading);
void savePathPlan(std::string shortestPathPlan);

int main(int argc, char **argv) {
  // Create/open output TXT file clearning all current data in file.
  std::ofstream saveFile;
  saveFile.open(OUTPUT_FILE_NAME);
  
  // Store map, walls, heading, start and goal nodes (defined from left to right, top to bottom).
  std::vector<std::string> map;
  int horizontalWalls[ROWS + 1][COLUMNS];
  int verticalWalls[ROWS][COLUMNS + 1];
  char heading = '^';
  int startNode = 0;
  int goalNode = 0;
  
  saveFile << COUT_PREFIX << "Reading in map from " << MAP_FILE_NAME << "..." << std::endl;
  std::cout << COUT_PREFIX << "Reading in map from " << MAP_FILE_NAME << "..." << std::endl;
  
  // Read in map from TXT file and process it to get the map, walls, heading, start and goal nodes.
  processMap(map, horizontalWalls, verticalWalls, heading, startNode, goalNode, saveFile);
  
  saveFile << COUT_PREFIX << "Map read in!" << std::endl;
  std::cout << COUT_PREFIX << "Map read in!" << std::endl;
  saveFile << COUT_PREFIX << "Finding shortest paths..." << std::endl;
  std::cout << COUT_PREFIX << "Finding shortest paths..." << std::endl;
 
  // Store node values.
  int cellValues[ROWS*COLUMNS];
  // Run flood filled algorithm to fill cellValues array.
  floodFill(cellValues, goalNode, horizontalWalls, verticalWalls);
  
  // Store all shortest paths and current path.
  std::vector<std::vector<int>> paths;
  std::vector<int> path;
  // Find all shortest paths.
  findPaths(paths, path, cellValues, cellValues[startNode], startNode, horizontalWalls, verticalWalls);
  
  // Print all shortest paths found to map and compare their path plans to find the shortest path with the least turns.
  int i = 1;
  std::string shortestPathPlan;
  int shortestPathPlanIndex = 0;
  for (auto path : paths) {
    // Print path. 
    printPath(path, i, map, saveFile);
    // Get paths plan.
    std::string currentPathsPlan = pathPlanner(path, heading);
    // Compare with current shortest path and update if shorter.
    if ((i == 1) || (currentPathsPlan.length() <= shortestPathPlan.length())) {
      shortestPathPlan = currentPathsPlan;
      shortestPathPlanIndex = i - 1;
    }
    i++;
  }
  
  saveFile << COUT_PREFIX << i - 1 << " shortest paths found!" << std::endl;
  std::cout << COUT_PREFIX << i - 1 << " shortest paths found!" << std::endl;
  saveFile << COUT_PREFIX << "Finding shortest path with least turns..." << std::endl;
  std::cout << COUT_PREFIX << "Finding shortest path with least turns..." << std::endl;
  
  i = 0;
  printPath(paths[shortestPathPlanIndex], i, map, saveFile);
  
  saveFile << COUT_PREFIX << "Shortest path with least turns found!" << std::endl;
  std::cout << COUT_PREFIX << "Shortest path with least turns found!" << std::endl; 
  saveFile << COUT_PREFIX << "Path Plan (" << shortestPathPlan.length() - 3 << " steps): " << shortestPathPlan << std::endl;
  std::cout << COUT_PREFIX << "Path Plan (" << shortestPathPlan.length() - 3 << " steps): " << shortestPathPlan << std::endl;
  saveFile << COUT_PREFIX << "Writing path plan to " << PATH_PLAN_FILE_NAME << "..." << std::endl;
  std::cout << COUT_PREFIX << "Writing path plan to " << PATH_PLAN_FILE_NAME << "..." << std::endl;
  
  // Save path plan to the TXT file.
  savePathPlan(shortestPathPlan);
  
  saveFile << COUT_PREFIX << "Path plan written to " << PATH_PLAN_FILE_NAME << "!" << std::endl;
  std::cout << COUT_PREFIX << "Path plan written to " << PATH_PLAN_FILE_NAME << "!" << std::endl;
  
  // Enter here exit cleanup code.
  saveFile.close();
  return 0;
}



// Read in map from TXT file and process it to get the map, walls, heading, start node and goal node.
void processMap(std::vector<std::string> &map, int horizontalWalls[ROWS + 1][COLUMNS], int verticalWalls[ROWS][COLUMNS + 1], char &heading, int &startNode, int &goalNode, std::ofstream &saveFile) {
  // Open map TXT file to read in from.
  std::ifstream file(MAP_FILE_NAME);
  std::string line;
  int i = 0;
  while (std::getline(file, line)) { 
    // Append to map.
    map.push_back(line);
    
    saveFile << COUT_PREFIX << line << std::endl;
    std::cout << COUT_PREFIX << line << std::endl;
    
    // Horizontal wall lines.
    if (i % 2 == 0) {
      int col = 0;
      while (col < COLUMNS) {
        // .compare(position, lenght of string, string) == 0 for match
        if (line.compare(1 + col*4, 3, "---") == 0) {
          horizontalWalls[i/2][col] = 1;
        } else {
          horizontalWalls[i/2][col] = 0;
        }
        col++;
      }
    // Vertical wall lines.
    } else { 
      int col = 0;
      while (col < COLUMNS + 1) {
        if (line.compare(col*4, 1, "|") == 0) {
          verticalWalls[i/2][col] = 1;
        } else {
          verticalWalls[i/2][col] = 0;
        }
        // If 'x', set goal node, else if '<^v>' and not a null terminator, set start node.
        //std::cout << "pos: " << 2 + col*4 << "\tsize: " << line.size() << std::endl;
        if (2 + col*4 < 38) {
          if (line.compare(2 + col*4, 1, "x") == 0) {
            goalNode = (i/2)*COLUMNS + col;
          }
          // .find_first_of(chars to look for, position)
          else if (line.find_first_of("<^v>", 2 + col*4) != std::string::npos) {
            startNode = (i/2)*COLUMNS + col;
            heading = line[2 + col*4];
          }
        }
        
        col++;
      }
    }
    i++;
  }
  file.close();
}

// Flood Fill Algorithm - Code adapted from: Lecture 4 pseudocode (Slide 107).
void floodFill(int cellValues[], int goalNode, int horizontalWalls[ROWS + 1][COLUMNS], int verticalWalls[ROWS][COLUMNS + 1]) {
  int node = 0;
  // Fill cell values with a large number and goal node with 0.
  while (node < ROWS*COLUMNS) {
    if (node == goalNode) {
      cellValues[node] = 0;
    } else {
      cellValues[node] = ROWS*COLUMNS;
    }
    node++;
  }
  int currentExploredValue = 0;
  int mazeValueChanged = 1;
  while (mazeValueChanged != 0) {
    mazeValueChanged = 0;
    // For all nodes - replaces need for row and column loop.
    for (int node = 0; node < ROWS*COLUMNS; node++) {
      // If current node value equals current explored value.
      if (cellValues[node] == currentExploredValue) {
        // For all directions of node.
        for (int direction = 0; direction < 4; direction++) {
          if (direction == NORTH) {
            // If neighbouring wall does not exist in direction.
            if (horizontalWalls[node/COLUMNS][node%COLUMNS] == 0) {
              // If neighbouring cell in direction equals ROWS*COLUMNS.
              if (cellValues[node - COLUMNS] == ROWS*COLUMNS) {
                // Update node value.
                cellValues[node - COLUMNS] = currentExploredValue + 1;
                mazeValueChanged = 1;
              }
            }
          } else if (direction == EAST) {
            if (verticalWalls[node/COLUMNS][node%COLUMNS + 1] == 0) { 
              if (cellValues[node + 1] == ROWS*COLUMNS) {
                cellValues[node + 1] = currentExploredValue + 1;
                mazeValueChanged = 1;
              }
            }
          } else if (direction == SOUTH) {
            if (horizontalWalls[node/COLUMNS + 1][node%COLUMNS] == 0) {
              if (cellValues[node + COLUMNS] == ROWS*COLUMNS) {
                cellValues[node + COLUMNS] = currentExploredValue + 1;
                mazeValueChanged = 1;
              }
            }
          } else if (direction == WEST) {
            if (verticalWalls[node/COLUMNS][node%COLUMNS] == 0) {
              if (cellValues[node - 1] == ROWS*COLUMNS) {
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

// Finds all shortest paths from start node to goal node using a combination of Flood Fill and BFS Algorithms.
// Code adapted from: Lecture 4 pseudocode (Slide 107) for Flood Fill, Week 4 Tutorial code for BFS and vectors.
void findPaths(std::vector<std::vector<int>> &paths, std::vector<int> path, int cellValues[], int step, int currentNode, int horizontalWalls[ROWS + 1][COLUMNS], int verticalWalls[ROWS][COLUMNS + 1]) {
  // Until reaching the goal node.
  while (step > 0) {
    step--;
    // Add to path.
    path.push_back(currentNode);
    int nextNode = currentNode;
    // Flag condition for if current path splits off into 2 viable short paths.
    int single_path = 1;
    // For all directions of node.
    for (int direction = 0; direction < 4; direction++) {
      if (direction == NORTH) {
        // If neighbouring wall does not exist in direction.
        if (horizontalWalls[currentNode/COLUMNS][currentNode%COLUMNS] == 0) {
          // If neighbouring cell in direction equals next step.
          if (cellValues[currentNode - COLUMNS] == step) {
            // Check if flag has been triggered.
            if (single_path) {
            // If not, get next node.
              nextNode = currentNode - COLUMNS;
              single_path = 0;
            // Otherwise another path is available, call the function down that path direction then return to continue current path.  
            } else {
              findPaths(paths, path, cellValues, step, (currentNode - COLUMNS), horizontalWalls, verticalWalls);
            }
          }
        }
      } else if (direction == EAST) {
        if (verticalWalls[currentNode/COLUMNS][currentNode%COLUMNS + 1] == 0) { 
          if (cellValues[currentNode + 1] == step) {
            if (single_path) {
              nextNode = currentNode + 1;
              single_path = 0;
            } else {
              findPaths(paths, path, cellValues, step, (currentNode + 1), horizontalWalls, verticalWalls);
            }
          }
        }
      } else if (direction == SOUTH) {
        if (horizontalWalls[currentNode/COLUMNS + 1][currentNode%COLUMNS] == 0) {
          if (cellValues[currentNode + COLUMNS] == step) {
            if (single_path) {
              nextNode = currentNode + COLUMNS;
              single_path = 0;
            } else {
              findPaths(paths, path, cellValues, step, (currentNode + COLUMNS), horizontalWalls, verticalWalls);
            }
          }
        }
      } else if (direction == WEST) {
        if (verticalWalls[currentNode/COLUMNS][currentNode%COLUMNS] == 0) {
          if (cellValues[currentNode - 1] == step) {
            if (single_path) {
              nextNode = currentNode - 1;
              single_path = 0;
            } else {
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

// Prints the map with the given path.
void printPath(std::vector<int> path, int i, std::vector<std::string> map, std::ofstream &saveFile) {
  // Print the path number if not the shortest path with hte least turns.
  if (i != 0) {
    saveFile << COUT_PREFIX << "Path - " << i << ":" << std::endl;
    std::cout << COUT_PREFIX << "Path - " << i << ":" << std::endl;
  }
  int length = path.size();
  for (int i = 1; i < length; i++) {
    // Get the current step of the node and convert it to a string.
    int step = length - i - 1;
    std::string stepString = std::to_string(step);
    // If the step value is a single digit add a space after it.
    if ((step >= 0) && (step <= 9)) {
      stepString.push_back(' ');
    }
    // Update to map replacing the centre and respective right cell.
    map[(path[i]/COLUMNS)*2 + 1].replace((path[i]%COLUMNS)*4 + 2, 2, stepString);
  }
  for (auto line : map) {
    saveFile << COUT_PREFIX << line << std::endl;
    std::cout << COUT_PREFIX << line << std::endl;
  }
}

// Determines the path plan for the given path.
std::string pathPlanner(std::vector<int> path, char heading) {
  std::string pathPlan;
  // Appends Epucks starting row, column and heading.
  // .append(size_t n, char c), int to char - https://stackoverflow.com/questions/4629050/convert-an-int-to-ascii-character
  pathPlan.append(1, '0' + path[0]/COLUMNS);
  pathPlan.append(1, '0' + path[0]%COLUMNS);
  
  int currentHeading;
  if (heading == '^') {
    pathPlan.append("N");
    currentHeading = NORTH;
  } else if (heading == '>') {
    pathPlan.append("E");
    currentHeading = EAST;
  } else if (heading == 'v') {
    pathPlan.append("S");
    currentHeading = SOUTH;
  } else if (heading == '<') {
    pathPlan.append("W");
    currentHeading = WEST;
  }
  
  // For the number of nodes.
  int currentNode = path[0];
  for (auto nextNode : path) {
    int nextHeading = currentHeading;
    // Ignores starting node
    if (nextNode != path[0]) {
      // Figure out the heading at the next node based on the difference between nodes.
      if (nextNode - currentNode == -COLUMNS) {
        nextHeading = NORTH;
      } else if (nextNode - currentNode == 1) {
        nextHeading = EAST;
      } else if (nextNode - currentNode == COLUMNS) {
        nextHeading = SOUTH;
      } else if (nextNode - currentNode == -1) {
        nextHeading = WEST;
      }
      
      // Based on the differce between headings, set the motions required.     
      if ((nextHeading - currentHeading == -1) || (nextHeading - currentHeading == 3)) {
        pathPlan.append("LF");
      } else if ((nextHeading - currentHeading == 1) || (nextHeading - currentHeading == -3)) {
        pathPlan.append("RF");
      } else if (nextHeading - currentHeading == 0) {
        pathPlan.append("F");
      } else if ((nextHeading - currentHeading == 2) || (nextHeading - currentHeading == -2)) {
        pathPlan.append("LLF");
      }
    }
    currentNode = nextNode;
    currentHeading = nextHeading;
  }
  return pathPlan;
}

// Saves the shortest path plan to the TXT file.
void savePathPlan(std::string shortestPathPlan) {
  std::ofstream saveFile;
  saveFile.open(PATH_PLAN_FILE_NAME);
  
  saveFile << shortestPathPlan;
  
  saveFile.close();
}
