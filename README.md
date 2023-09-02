# Masters
## 2D GAIT GENERATION:


## 3D GAIT GENERATION CODE SETUP:
![code_upgrade_2](https://github.com/MaretheCoetzer/Masters/assets/80690410/3ce0c294-2a61-4e3c-a8c5-e1fb41fd2306)

The step-up.py script is where the main trajectory optimisation was performed. The
other scripts were simply created to make it easier to find navigate through the code.
A JSON scripts, run-config.json, was used to feed the input parameters to the step-up
script. This improved the ability to track and change a greater amount of inputs. The
2D trajectory had three adjustable inputs, the amount of nodes, the travel distance and
the travel velocity. Whereas the 3D trajectory problem had the following parameters
that could be set: The result name, a description, the log level, the amount of nodes, the
amount of epsilon iterations, the solver could be set to continue on from a previous run,
the minimum travel distance and the minimum height the feet need to lift off the ground
when stepping forward.

The config-parser.py script was used parse the JSON script parameters into usable
python parameters and fed these parameters back to the step-up.py script. While parsing
the parameters it also creates a results folder with the provided result name and append a
unique run identification number to it, and places a copy of the JSON file in the folder.
The collocation matrix, time bounds and quadruped DOFs and dimensions and were
not expected to alter frequently and was therefore stored in constants.py. Log.py is a
custom logger that aids in the documentation and fault finding of the code and was used
to track the computation time of each TO run.

The seed.py script allowed the use of three different seeding methods in the step-up
script. The first function still started the optimiser first iteration with randomly assigned
angles. The purpose of this being to mitigate the fact that the solver finds the local
optimal solution and not the global optimal solution. If a gait is rerun numerous times, the
best gait can be selected from the results and assumed to be the global optimal solution.
This however takes long for the solver to solve. Therefore, the second function, allows
the search space to start from a previous successful generated gait. The third function is
used to refine previously completed runs. This function compares the amount of epsilon
iterations that needs to be performed, with the amount the previous trajectory completed
and picks up from there. This allows the user to first test the validity of the inputs to a less
refined ε value and based on the results either tune the input parameters or run it again
to a more refined ε value without the need to restart the computation process from the start.
The step-up script outputs the raw trajectory results in csv format, which is then run
through the linearisation script to obtain the linearised servo motor angles. In addition to
that, the results are also sent to the trajectory-reader to generate gif files, for the user to
evaluate the quality of the generated gait. After each ε iteration the model.pkl is saved,
7.3. Address increase in computation time

which can be used for future runs, to refine the gait further than it was initially set to
