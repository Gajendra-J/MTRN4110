# MTRN4110PhaseC_Testing
Testing and test photos for MTRN4110 Phase C


# Downloading the autotester
## Adding the contents of this test repo to your project

#### Either
Download this repo as a .zip folder and add it to your project's root directory

#### Or - to stay up to date
To update your current project git repo with these latest files, add this repo as an upstream friend. In your terminal in your project's root directory, run:

`git remote add upstream https://github.com/courtzc/MTRN4110PhaseC_Testing.git`

Then, to pull latest changes from that upstream into your project, go to your **project's** root directory and run:

`git pull upstream main --allow-unrelated-histories`

After the first time, you won't need the `--allow-unrelated-histories` tag. Best practice is to pull the upstream every time you go to start testing. Use the command:

`git pull upstream main`


## Running the autotester

### Change autotester.py
Change the `path` variable to be the your project's tests directory. i.e. mine is:

`path = 'C:\\Users\\Court\\source\\repos\MTRN4110PhaseC\\tests'`

Change the number of worlds variable to be however many worlds you want to test. i.e.

`x = 7`

### Change project_function_code.py
Export your ipynb notebook into a python script (it's an option in vscode command palette). Copy the content into the file `project_function_code.py`, which looks like this:


```
def my_project(maze_pic, bug_pic, robot_pic):


  < all your code >
  
  
  return <output text in the form of 37x11 array of chars>
```

The project_function_code.py function should return a str which is the maze. I did it like this (for each row that I was writing to a .txt)

```
actual_output = ""
# for each row (obvs str('') content will vary)
  actual_output += str(' --- --- --- --- --- --- --- --- --- \n')
return actual_output
```


Change your paths for the three pictures to be the three input variables, i.e.:

```
  MAZE_FILE_NAME = maze_pic
  ROBOT_FILE_NAME = robot_pic
  IMAGE_LADYBUG_FILE_NAME = bug_pic
```

### Go to anaconda prompt
Go to your anaconda prompt and navigate to the root directory of **your project git repo**. Then type:

`python auto_tester.py`

Done!

## Recommendations
Comment out your `plt.show()` statements if you're doing lots - they pause the execution until you close them

# Contributing to the test repo (i.e. uploading worlds and photosets)

All changes to the test things (pictures, the autotester, expected output maps) **should be committed and pushed to this test repo** (not your project repo, even though it will have the files). This is because the testing repo isn't watching your project, and won't update automatically.

It's just your normal clone procedure to have this repo on your computer and contribute directly to it.

## Requirements for photos
I've added a file `region_overlay.pptx` which has a set of lines on it that line up with the spec "ABCD" regions. If you take the maze fixed size screenshot and copy it into the powerpoint and "send to back", you can quickly make sure it's a valid shot (i.e. cornerstones are in regions, pink cornerstone is in either top left or bottom right region).

## Syntax for filenames

### World
These go in the worlds/ folder.

Put the number of the world (001 - 999) somewhere in the world name so we can keep track and make more photos later if we need to. 

i.e. for world one, it would be

`z1234567_MTRN4110_PhaseC_001.wbt`

*note: not sure if we need to put the .wbproj file in for every world?*

### Outputs

Put these bad boys in the tests/ directory.

#### Output text file (the known map):

world(001-999).txt

i.e. for world one expected output, it would be

`001.txt`

(this ensures it's first in the list of files returned by the 001 search)

### Photo sets

These also go in the tests/ directory.

Each set of photos of the world (maze & robot) have this syntax:

#### maze:
maze(001-999)_photo(a-z)_maze_inversion('inv' or 'nor')_robotpos(rowcolumn)_robotheading(N S E or W)_realtargetpos(rowcolumn)_faketargetpos(rowcolumn)

i.e. for world one, 2nd photoset, which has the maze inverted, it would be

`001_b_maze_inv_00_S_24_16.png`

(not sure if we need all this info, but it might be useful for say giving an output like "you said it was north, but it's actually south". Also, the 'inv' or 'nor' need to be 3 characters each so I can continue to hack at the substrings)

#### robot:
world(001-999)_photoset(a-z)_robot

i.e. for world one, 2nd photoset, it would be

`001_b_robot.png`


Let's get testin!
