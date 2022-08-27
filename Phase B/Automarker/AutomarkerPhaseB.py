# -*- coding: utf-8 -*-
"""
Created on Tue Jun 29 15:08:39 2022
@author: Calvin Chua
Usuage: 
python ./Automarker.py [zid]
nstructions:
If you have installed python or have had experience with using python, skip step 1.
1. Download and install Anaconda Python - https://www.anaconda.com/products/individual
2. Save "Automarker.py" into the same directory you output your "Output.txt" and "PathPlan.txt"
3. Rename the "Output.txt" shared by Leo/other students as "Solution.txt"
4. Open Anaconda Prompt, and change directory to the folder where the files above are located
(you should replace [path] with the actual directory of the "Output.txt")

cd [path]
For example, if your directory is at C:\z5159352\OneDrive - UNSW\Desktop\Teaching
input 
cd "C:\z5159352\OneDrive - UNSW\Desktop\Teaching"

5. You should now be at the folder where your "Output.txt" is located. Run the automarker
using (you should replace [zID] with your own zID):

python3 ./AutomarkerPhaseB.py [zID]

For example, if your zid is z1234567
input 
python3 ./AutomarkerPhaseB.py z1234567


6. Please contact Ming Xuan Chua on Teams if you have any questions

Disclaimer:
The purpose of providing this automarker is for you to check the format. There should be no assumption extended from
the test cases provided.

"""



import sys
import math
defaultPrefix ="[z1234567_MTRN4110_PhaseB]"


def compare_maps(soln_map, stu_map):
    if(len(soln_map)!=len(stu_map)):
        return False
    # Compare current stu_path with current soln_path
    for i, soln_row in enumerate(soln_map):
        if soln_row != stu_map[i]:
            return False

    return True

# Task A: check that input and map match

def taskA(solution, student,prefix):
    # Get our solution's map, without outer walls
    solMap =[]
    startingLine =defaultPrefix + " Reading in map from ../../Map.txt..."
    endingLine =defaultPrefix +" Map read in!"
    if(solution.readline().rstrip() == startingLine):
        line = solution.readline().rstrip()
        while line != endingLine:
            line = line.replace(defaultPrefix,"")
            solMap.append(line)
            #print(line)
            line = solution.readline().rstrip()
            
    # Get the student's map
    stuMap = []
    startingLine =prefix + " Reading in map from ../../Map.txt..."
    endingLine =prefix +" Map read in!"
    if(student.readline().rstrip() == startingLine):
        line = student.readline().rstrip()
        while (line != endingLine and line):
            line = line.replace(prefix,"")
            stuMap.append(line)
            line = student.readline().rstrip()            
    mark =0
    correctLines =0
    if(len(stuMap)!=len(solMap)):
        print("Student might not follow the syntax or has extra lines in maze")
        print("Please validate and rerun the code")
        return 0
    
    for i, row in enumerate(solMap):
        if(i==len(stuMap)):
            break
        if("--- --- --- --- --- --- --- --- ---" in row):
            continue
        if(row != stuMap[i]):
            print(f'Line {i}:\n\tExpected:\n\t{row}\n\tGot:\n\t{stuMap[i]}\n')
        else:
            correctLines +=1
            
    if(correctLines ==(len(solMap)-2)):
        mark =10
    else:
        #map incomplete 
        mark =10-((len(solMap)-2)-correctLines)
        if(mark < 2):
            # at least 1 line is correct
            if(correctLines>0):
                mark =2
            else:
                # Incorrect map
                mark =0
                
    return round(mark,2)

# Task B: check all the paths


def getPaths(maps, startingLine, endingLine,prefix):
    #Store both solution and student mazes into a list of lists of string
    #[[maze 1] [maze 2]]
    mazeList =[]

    if(maps.readline().rstrip() == startingLine):
        line = maps.readline().rstrip()
        maze =[]
        pathLine =True
        while line != endingLine:
            line = line.replace(prefix,"")
            if("Path - " in line):
                
                if(pathLine ==True):
                    pathLine =False
                else:
                    mazeList.append(maze)
                    #pathLine =True
                maze =[]
            else:
                if("shortest paths found!" not in line): 
                    maze.append(line)
                    #print(line)
                else:
                    mazeList.append(maze)
            line = maps.readline().rstrip()
    return mazeList

def taskB(solution, student,prefix):
    #Store both solution and student mazes into a list of lists of string
    #[[maze 1] [maze 2]]
    startingLine =defaultPrefix + " Finding shortest paths..."
    endingLine =defaultPrefix +" Finding shortest path with least turns..."
    solMazeList = getPaths(solution,startingLine, endingLine,defaultPrefix)
    startingLine =prefix + " Finding shortest paths..."
    endingLine =prefix +" Finding shortest path with least turns..."
    stuMazeList = getPaths(student,startingLine, endingLine,prefix)
    
    temp =[]
    for path in stuMazeList:
        if path not in temp:
            temp.append(path)
            #stuMazeList.remove(path)
            
    dupNum =len(stuMazeList) -len(temp)
    stuMazeList =temp
    numPathSol = len(solMazeList)
    numPathStu = 0

    for solIndex, solMaze in enumerate(solMazeList):
        matchFound =False
        for stuIndex, stuMaze in enumerate(stuMazeList):
            if compare_maps(solMaze, stuMaze):
                stuMazeList.remove(stuMaze)
                matchFound =True
                break
        if(matchFound ==True):
            numPathStu +=1
        else:
            print(f'Path {solIndex+1} in solution is not found')
    wrongPathNum =len(stuMazeList)
    if(wrongPathNum !=0):
        print(f'{len(stuMazeList)} path/s in the student answer is/are not part of the solution')
        
    if(numPathStu ==numPathSol):
        mark =50
    else:
        if(numPathStu>numPathSol):
            print("Something goes wrong")
        mark =numPathStu/numPathSol *50
    mark -= (wrongPathNum +dupNum)/numPathSol *50
    if(mark<10):
        if(numPathStu>0):
            mark =10
        else:
            print("If you have at least found a valid path, self award yourself 5%")
    if(dupNum>0):
        print(f"There is {dupNum} of duplicate maze")
    
    return round(mark,2)


if __name__ == "__main__":
    zid =sys.argv[1]
    prefix ="["+zid+"_MTRN4110_PhaseB]"
    # This is where the actual marking happens
    with open("Solution.txt", "r") as solution:
        with open("Output.txt", "r") as student:
            scoreTaskA =taskA(solution, student, prefix)
            print(f"Student achieved {scoreTaskA}/10 for task A")
            scoreTaskB =taskB(solution, student, prefix)
            print(f"Student achieved {scoreTaskB}/50 for task B")

