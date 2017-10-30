# Project: SWARM Robotics Robustness


## What it is!
This project analyzes and experiments with swarm robustness building off prior
experimentation done by Bjerknes and Windfield's 2013 paper entitled "On fault
tolerance and scalability of swarm robotic systems".


## Versions
This is it.


## Documentation
See paper.


## Files:
* README.md
* argos3-examples (http://www.argos-sim.info/examples.php)
* controllers/epuck_brownian/epuck_brownian.cpp
* controllers/epuck_brownian/epuck_brownian.h
* controllers/epuck_brownian/CMakeLists.txt
* controllers/CMakeLists.txt
* experiments/epuck_brownian.argos
* loop_functions/brownian_loop_functions/brownian_loop_functions.cpp
* loop_functions/brownian_loop_functions/brownian_loop_functions.h
* loop_functions/brownian_loop_functions/CMakeLists.txt
* loop_functions/CMakeLists.txt


## Requirements:
* ARGOS v3


## Installation
1. Download files to a location of your choice from:
GitHub: https://github.com/jkrukar/SwarmRobustness.git
2. In the root directory run: ./build.sh


## Instructions:
In the root directory run: argos3 -c experiments/epuck_brownian.argos
To modify simulation parameters edit epuck_brownian.argos file.  In order
to repeat the scalability vs. robustness experiment change the following
attributes in the .argos XML file.
1. failure_case - 1 is complete failure, 2 is sensors failure, and 3 is motor failure
2. k - the number of Epucks that will fail (10% according to experiment)
3. N - the total number of Epucks in experiment (should equal quantity attribute)
4. layout - grid layout of Epucks (must add up to quantity attribute)
5. quantity - the total number of Epucks in experiment


## Licensing
FREE


## Troubleshooting
For ARGOS errors see ARGOS help documentation at http://www.argos-sim.info/documentation.php


## Contacts
Akil Andrews - akil.andrews@gmail.com
John Krukar - john.a.krukar@gmail.com
Brandon Wade - bwade@unm.edu


**Enjoy!**
