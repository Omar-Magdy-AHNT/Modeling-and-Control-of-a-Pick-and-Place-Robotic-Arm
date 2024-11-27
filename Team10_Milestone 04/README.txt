-------FOR MATLAB AND SIMSCAPE
To get the angles for the joints for the full trajectory, you have to open and run "ALL_OF_THE_SIMULATION.m"
This will output all the angles in the command window view;in addition, it will show plots for the joint angle values overtime and another plot for the EEF position in xyz over time. You can uncomment line 108 to run the simscape simulation directly.
else
You can run RoboticArm1.slx for the simulink model and run simulation to see the robot move on simscape
and also check the scope for the EEF positions in the xyz and compare it to the values in the code.

The "Manga.m" file includes the code to run the simulation to an Arduino for the hardware implementation
u can directly run from there with an Arduino connected.

-------FOR PYTHON AND COPPELIASIM
You can run the pyhton code named "AllTraj.py" to get plots also for EEF pos in xyz and joint angle values same as matlab.
To simulate it in coppelia, open the file named "ALLTRAJECTORY.ttt" and change line 38 script path to wherever u save the "AllTraj.py" file and run simulation.