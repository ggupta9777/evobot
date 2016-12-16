# evobot
# Gaurav Gupta

About: 
This code was written as a part of course project on Advanced Topics in Robotics (ME769A) at IIT Kanpur (India) in Nov. 2015.
The program involves creating a robot, imparting it sensors, geometry, programming the control law, designing the enviornment with obstacles and performing optimization using GA to get a control function (Neural Network based) which maximizes the the distance the robot can travel in a given time. 

Libraries used - Numpy, PyGame, Pyevolve, math, random, matplotlib, time, scipy

How to run? 
Just run evobot.py, at the current configuration,optimization is performed for 25 generations, after which the best individual is used to present a visual using pygame (can take a few minutes!), the window closes after one run. Setting such as generation number, position of obstacles, enviornment etc can be changed from within the code.
To quickly see what the code is about, find 'ga.setGenerations' in the code and set it to maybe 5, this should give an idea. To see a good enough result however, I'd recommend using a value of 20. (should be clear from the report below)

More details - 

Video compilation of the results of this work are present here - https://www.youtube.com/watch?list=PLKApMD4Rh1z067ayPYozdUH_wMv-uPPOz&v=aSdighMppxg

An elaborate report on the underlying theory is presented here - http://home.iitk.ac.in/~gaurag/files/reportevo.pdf

