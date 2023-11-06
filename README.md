# mazesolver
Title: Maze solving robot with webots

Project members: Juho Matilainen

Project leader: Juho Matilainen

Goal: Make e-puck robot able to solve different kind of mazes

Time reserved: 50h per member

Due date: Friday 8.12.2023, according course moodle

Project supervisors: Dr. Ilkka Jormanainen, Dr. Samuel Tewelde Yigzaw

Concept Plan- refers to the robotics project you want to implement or demonstrate. It answers the questions:
    why you chose the topic: Because it seemed interesting
    what functionalities you want to implement in the project: I want e-puck robot to be able to solve mazes
    what you want to demonstrate by the end of your project: E-puck robot solving mazes
    how you think the project will be feasible for development: 

Technical Plan- includes:
    the list of tools you need to use to implement the project and modify as considered in the concept plan (hardware and software tools, programming language, etc): Webots and computer supporting it.
    the implementation sequence / procedure so that you can mention how to start the project implementation and what to do first until the mid-project presentation period. Then the technical implementation plans until the end of the project.: b4 midway I hope that E-puck robot is able to traverse mazes and at the end completely solve them.

TODO: 

  1) Make e-puck controller that can traverse a maze and not get stuck.
  3) Make e-puck controler able to create a graph of the maza that it traverses
  4) Make bsf algorithm to solve the graph
  5) Make the robot use the solved path
  6) Add more worlds/mazes for the robot to solve

Project journal:

  //16.10.2023 	
	
  This is projectwork for Robotics course. Goal of this project is to make e-puck robot able to solve mazes.
  We will discuss with project members how we will implement the function to solve mazes.
  Initial plan was to make the robot use Breadth-first search(bfs) and map the maze junctions as vertexes and make edges between vertexes that are connected in the maze.
  Also added base files for webot program. There is 1 world with easy maze.

  //1.11.2023 	

  Added controller for the e-puck to use, named maze. It only has setup done and no maze solving function, insted only goes straight max speed.
  Also thought how to implement the robot to move in the maze regarding the 16.10 plan. Still no idea.

  //3.11.2023

  Added fall following maze solve for perfect mazes, it doesn't work for example if maze has a loop. Since I am working alone and couldn't figure out how to implement bfs, also I think it would take too much time.
  If I get wallfollow to work I will use remaning time to implement bfs.
  
  //4.11.2023

  Started making movement functions for robot, plan on using them for betterwall following first. Then figure out how to implement bfs.
  
  //5.11.2023

  Done with forward movement function. It will move robot forward wanted distance in meters. Turn left/right still unfinished.
  
  //6.11.2023

  Finished all movement functions, but for some reason turns are not accurate. When turning pi/2 it will only turn like 5pi/12. If turn input value 100deg it seems to be close to pi/2. Needs fixing.
