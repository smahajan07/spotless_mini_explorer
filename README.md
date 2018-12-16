# Frontier Exploration Robot
[![Build Status](https://travis-ci.com/smahajan07/spotless_mini_explorer.svg?token=JmJzv9NxrpFcdhLWReKp&branch=master)](https://travis-ci.com/smahajan07/spotless_mini_explorer)
[![Coverage Status](https://coveralls.io/repos/github/smahajan07/spotless_mini_explorer/badge.svg?branch=master)](https://coveralls.io/github/smahajan07/spotless_mini_explorer?branch=master)
![Version](https://img.shields.io/badge/version-0.1-orange.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Overview
Autonomous robots have been an active field of research before and since the DARPA Grand Challenge in 2005 and the Urban Challenge in 2007. Even after a decade, autonomous robots are helping solve problems in a diverse space and creating potential applications of the future. Autonomous robots are now present in different spaces such as self driving cars, sidewalk-delivery robots, autonomous sea explorers and autonomous drones and every environment setting brings along myriad of challenges and while some might be generic others might be particular to that setting. 

The aim of this package is to introduce the user to a specific problem in the field of autonomous robots, i.e. frontier exploration. When a robot navigates in a known space, some of the localization (knowing the location of the robot in the physical space) problems are easy to solve, however if the robot is placed in an unknown environment the way to tackle localization is to create a map of your surrounding simultaneously. This is usually referred to as the chicken and egg problem but for now let's assume that is being taken care of. The interesting part that we will solve is to autonomously perform this task!

In order to autonomously exlpore an unknwon space, the robot needs to keep visiting the _unknown_ parts in its map and stop only if commanded or if there are no more _unknown_ spaces left. This is the key concept behind frontier exploration. Sounds simple, right!

__Implementation__

For the scope of this package, we will be using a turtle bot, which is small yet powerful robot with a lot of support available online and the algorithms implemented on the turtle bot can be scaled up to differenct robots. Certain packages are already provided to solve the task of mapping and navigation such as gmapping and move_base respectively. 

_Visiting_ the unknown parts involves computing the route to a goal and this can be done via many different algorithms and for this package a variant of Breadth First Search is selected for it's simplicity and effeciency. Gazebo will be used for simulation and [Optional] RVIZ for visualization.

The project is programmed in C++ and uses modern C++ features and strongly follows the concepts of Object Oriented Programming. The project follows a Test Driven Development approach and a Solo Iterative Process (SIP). Doxygen documentation id also provided for reference. 

For reference, this is how it looks when the package is run:
![implementation](results/sprint2/nav_with_move_base.png)

__Applications__

The primary aim of this package is to help ACME robotics in it's frontier exploration tasks but it can be scaled in the future and run as a complementary module for other exploration robots with complicated tasks such as industrial inspection.

## Dependencies
The following dependencies need to be met before installing this package:
* [ROS Kinetic](http://wiki.ros.org/ROS/Installation) on Ubuntu 16.04
* [Gazebo 7.x](http://gazebosim.org/download)
* [Turtlebot](http://wiki.ros.org/turtlebot) and [Turtlebot_Gazebo](http://wiki.ros.org/turtlebot_gazebo)
* [Move Base](http://wiki.ros.org/move_base) (Ideally this is a part of standard ROS installation, please download if not already installed)

## Presentation and Video demonstration
The user can also refer to the following presentation slides and the video for more details about the project and video demo to see how to build, run tests or run the demo.

[![Presentation](https://img.shields.io/badge/Presentation-v0.1-brightgreen.svg)](https://docs.google.com/presentation/d/1Ei56q79E8JUMGSWKWq4-YBZp_kIpXm-DbYsL_7wLOZI/edit?usp=sharing)

Part 1 - Introduction and Overview [![video1](https://img.shields.io/badge/video-1-blue.svg)](https://youtu.be/QeWW-rN9QWs)

Part 2 - Build, run tests and run demo [![video2](https://img.shields.io/badge/video-2-blue.svg)](https://youtu.be/_23vLnvyQ6w)

Part 3 - Final Output, Gitg and future work [![video3](https://img.shields.io/badge/video-3-blue.svg)](https://youtu.be/UuaJW4zg3QM)

BONUS video - complete demo video! Note: It's 20 min long and recomended to watch at 2x! [![bonus_video](https://img.shields.io/badge/bonus%20video-demo-blue.svg)](https://youtu.be/rbME59c9Ask)

## Instructions to Build
* If you already have a catkin workspace then:
```
cd <catkin workspace>
cd src
git clone https://github.com/smahajan07/spotless_mini_explorer.git
cd ..
catkin_make
source devel/setup.bash
```

* If you do not have a catkin workspace, you can create one by following:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/smahajan07/spotless_mini_explorer.git
cd ..
catkin_make
```
## Instructions to run tests
Before you proceed to run it, check whether it passes the unit tests:
```
cd <your catkin workspace>
source devel/setup.bash
catkin_make run_tests
```
This should end with an output like

![rostest_text](results/rostests.png)

## Instructions to run Demo
Two terminals would be required for this, so in the first terminal: 
```
cd <your catkin workspace>
source devel/setup.bash
roslaunch spotless_mini_explorer demo.launch
```
In the second terminal, run:
```
rosrun spotless_mini_explorer frontier_explorer
```
Note: Gazebo is known to fail sometimes and in case it does fail to open, please kill tasks in both terminals and try again.

At any point in time is the user wishes to save the map built, open a terminal and run:
```
rosrun map_server map_saver -f <map_name>
```
Note: This will save a map in your CURRENT working directory, so it's best to call it from a directory of your choice (say, results) or give the appropriate path.

Moreover, the demo will potentially run for a long time, basically till it maps the unknown space, so in order to kill the demo please press CTRL+C in the active terminals.

## Rosbags
* Inspect

Along with the package a sample ROS bag was provided ```results/explorerRecording.bag```. 
If you wish to look at the topics in the rosbag and find out more details about it:
```
cd <your catkin workspace>
source devel/setup.bash
cd src/spotless_mini_explorer/results
rosbag info explorerRecording.bag
```
* Play

You can play the above mentioned rosbag by running the following in two different terminals:
```
roscore
```
 and
```
rosbag  play explorerRecording.bag
```
* Record

If you wish to record the data in a rosbag you can run the rosbags command separately in a new terminal OR you can make use of the option provided in the launch file. You just need to set the record argument to true by running:
```
cd <your catkin workspace>
source devel/setup.bash
roslaunch spotless_mini_explorer demo.launch record:=true
```
This will save a rosbag in your results directory of the package, by the name ```explorerRecording.bag```.

## Track progress
The development followed a Solo Iterative Process and the progress can be tracked via the following links:
* [SIP Google Sheet](https://docs.google.com/spreadsheets/d/1qItKc6DQDyJmSBWIZsrJj505HqRsbLVi_X6wqlamRMk/edit#gid=0)
* [Planning Log](https://docs.google.com/document/d/1M3QvbsZYWknKas6uhFjU1NELlZafBw9uz3sJWOEfes0/edit)

## Code coverage
![Code coverage](https://img.shields.io/badge/coverage-92%25-green.svg)

(NOTE: THIS IS A SELF GENERATED TAG, not to be confused with the automatic generated tag from coveralls)

Since there is currently some issue with coveralls picking up the build from travis and checking for code coverage, provided below is a screenshot of the coverage report generated using lcov, locally:
![code_coverage](results/lcov_coverage.png)

Instructions to run code coverage yourself:
```
cd <your catkin workspace>
cd build/
lcov --directory . --capture --output-file coverage.info
lcov --remove coverage.info '/opt/*' '/usr/*' '*/devel/*' '*test_*' '*_test*' --output-file coverage.info
lcov --list coverage.info
```

## Known issues/bugs
* Gazebo might take some time to load, especially if a VM is being used. So, if gazebo fails while running the demo, please try launching again
* The turtlebot may sometimes get stuck in between obstacles, trying to clear it's path or even in a situation where the surrounding is homogeneous or no frontiers are left in the _locally surrounding_ open space, both these situations is where human intervention might be required and the user can either teleoperate and move the bot to a new location (and kill teleoperation) or simply start the nodes again, however it should be noted that the information generated uptill this point will be lost.

## Doxygen
Using Doxygen, documents have already been generated and you can view them by, going to your package directory and running:
```
cd docs
cd html
firefox index.html
```
However, if you wish to generate it yourself, you will need to install doxygen (if not already installed, follow the given steps), and then you can follow the steps below for generating the docs:

* To install doxygen run the following command: 
```
sudo apt-get install doxygen
```

* Now from your package directory, run the following command:

```
doxygen spotlessDocs
```

Doxygen files will be generated in /docs folder

## About the developer
My name is Sarthak Mahajan and I'm currently pusruing my Masters of Engineering in Robotics and University of Maryland, College Park. My areas of interest are computer vision, machine learning and deep learning. I have worked on multiple projects in these areas and keep trying to do more hands on projects. As a budding roboticist, I love to work towards solving real world problems. Working on ROS (and other open source projects) has made me appreciate the value of an open source community and I would like to contribute back to such communities.

## Disclaimer
This package has been created as a part of the course ENPM 808X Software Development for Robotics. Please read the license for details on the terms and conditions of usage.

## License
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
