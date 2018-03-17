# DD2438-Multi_agent_systems 

### (Under construction...)
This repository includes solutions to 5 different multi agent problems: Multi-agent collision avoidance,
the Vehicle Routing Problem........

The implementation is done in Unity 2017.3.0f3 using C#.

Below a description of each problem is given along with a description of our solution. (The names in the parentheses are the file names containing the data for the respective problems)

## Multi-agent collision avoidance (P21)
Given the starting and the goal positions of N vehicles, the goal is to control the vehicles so as to arrive to their goals
in the minimum time possible without colliding. The vehicle is assumed to have a radius of 0.5 distance units.


## Vehicle Routing Problem (P22)
Given the starting and the goal positions of N vehicles, the goal is to find or approximate the optimal trajectories so that the vehicles reach their goals after passing from a set of points of interest (e.g. customers). This is similar to the classical Vehicle Routing Problem with the main difference of each vehicle having a different start and destination node instead of all the vehicles having a single depot where they start and finish. 

We provide a solution to this problem with the use of a genetic algorithm where the paths between each node in the enviroment are given by a visibility graph. For this problem we have used a simple kinematic point model.

## Formation Along Trajectory (P25)
Given starting positions, a desired formation and a trajectory of a friendly vehicle, the goal is to move the agents so as to keep the formation around the friendly vehicle (leader). In this problem obstacles are ignored as well as collisions between agents. A dynamic point model with bounded velocity and acceleration is used for the agents in the formation.
