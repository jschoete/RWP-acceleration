# RWP-acceleration

## Project specifications

This is a Gradle project. The project's java files are found in `/app/src/main/java/`.
The aim of this project is to showcase the Vector Racer mobility model, which aims to modelize the movement of an autonomous entity, such as a drone, effectively simulating acceleration and inertia forces.

## How to run

To start, run `Main.java`. A JBotSim window will appear with two points to visit and the visting entity. A precomputation phase is necessary, to avoid significant computation times. To launch the precomputation, click right inside the JBotSim window and select **precompute**. A message will appear in the console when the computation is done. At this moment, click right again in the JBotSim window and select **start execution**.

## Description

Our entity uses an algorithm to visit the two present points in an efficient manner, trying to optimize its trajectory accordingly. It is unknown to the entity what points to visit will appear in the future, and because of this, it refrains from accelerating constantly and naively towards its current points to visit. We claim the precomputation phase is indeed only hiding computation time, *i.e.* at no point in time (or in the computation) the entity has knowledge of more than two points to visit.

## Detailed algorithm description

The algorithm works as follows. In short, we construct a phase space, *i.e.* a directed graph in which vertices represent the entity's possible configurations. Arcs between phase space vertices represent the possiblity of the visiting entity to go from one configuration to another in exactly one time unit.
The algorithm in `Inter_check_2BFS.java` consists of finding the shortest path between 2 phase space vertices. This is done through two breadth first searches (BFS) in the phase space, each starting at one of the two phase space vertics.
One of these phase space vertices is the visiting entity's current configuration, denoted **O**, while the other corresponds in fact not to the next point to visit, denoted **A**, as one might expect, but to the *second* next point to visit, denoted **B**. The idea is to ensure that the visiting entity will not just accept any good trajectory that visits **A**, but one that is, in a sense, a good trajectory to visit **B** as well afterwards.
Once the two breadth first searches meet in the phase space, they thus have to verify if one of them has visited **A**. If the conditions are met, the trajectory is accepted.
The trajectory is then shortened, such that the visiting entity can visit **A**. The entity follows this shortened trajectory, leading to it visiting **A**. At this time, the point **A** dissapears, and another point to visit appears in the area, denoted **C**.
The algorithm is then repeated, for points **B** and **C**, *etc.
