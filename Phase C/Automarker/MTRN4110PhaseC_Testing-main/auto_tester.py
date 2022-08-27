import os
import os.path
from project_function_code import project_function

# ---
# note: use the command palette to set the interpreter as python 3.7.10
# ---

# -----------------------------------------------------------------------------

### CHANGE THESE INPUTS ###

# what is the path to your tests folder?
path = "D:\\OneDrive\\Documents\\AAUNSW\\2022 T2\\MTRN4110\\MTRN4110PhaseC\\tests"

# how many worlds are you testing?
x = 6

# -----------------------------------------------------------------------------

# Summary code
total_tests = 0
passed = 0
failed = 0


### CODE ###
t = x + 1

# for each world
for j in range(1,t):

    h = str(j).zfill(3)
    print("\n=======================================")
    print("\nWorld", h, ":")
    
    files = [i for i in os.listdir(path) if os.path.isfile(os.path.join(path,i)) and i.startswith(h)]

    # read in expected output .txt
    map_file_temp = ["tests/", files[0]]
    map_file = ''.join(map_file_temp)
    
    map_ex = ""
    with open(map_file, 'r') as f:
        line = f.readline()
        while line:
            map_ex+=str(line)
            line = f.readline()

    # for each test set
    k = 1
    while k < len(files):
     
        # take the three files
        bugfile =  ["tests/Ladybug_small.png"]
        mazefile =  ["tests/", files[k]]
        robotfile =  ["tests/", files[k+1]]
        bug_photo = ''.join(bugfile)
        maze_photo = ''.join(mazefile)
        robot_photo = ''.join(robotfile)
        
        set_letter = maze_photo[6:12]

        # feed them into the project as inputs
        map_generated = project_function(bug_photo, maze_photo, robot_photo)
        text_file_temp = [set_letter, 'failed_test_comparison.txt']
        text_file_name = ''.join(text_file_temp)


        # compare output to the output text file
        # if identical, yay!
        if (map_generated == map_ex):
            print('\n', set_letter, ": success!")
            total_tests += 1
            passed += 1

            # we can delete this failure.txt
            if os.path.exists(text_file_name):
                os.remove(text_file_name)
            
        # if not identical, sad.
        else:
            print('\n', set_letter, ": failure.")
            print("\nyou generated: ")

            # export the failure to a txt for clearer comparison
            with open(text_file_name, 'w') as f:
                
                f.write('Map generated:\n')
                f.write(map_generated)
                f.write('\n\n')
                f.write('Map expected: \n')
                f.write(map_ex)

            print(map_generated)
            print("\nbut the expected output was: ")
            print(map_ex)
            total_tests += 1
            failed += 1
        
        
        k += 2


print("\n\n================SUMMARY=================")
#print("Conducted " + str(total_tests) + " total tests")
print("Passed: " + str(passed) + " / " + str(total_tests))
#print("Failed: " + str(failed) + " / " + str(total_tests))
print("================SUMMARY=================")

