# mazesolver
Title: Maze solving robot with webots

Project members: Juho Matilainen

Project leader: Juho Matilainen

Goal: Make e-puck robot able to solve different kind of mazes

Time reserved: 50h per member

Due date: Friday 8.12.2023, according course moodle

Project supervisors: Dr. Ilkka Jormanainen, Dr. Samuel Tewelde Yigzaw

TODO: 

  1) Make e-puck controller that can traverse a maze and not get stuck.
  2) Make e-puck controler able to create a graph of the maza that it traverses
  3) Make bsf algorithm to solve the graph
  4) Make the robot use the solved path
  5) Add more worlds/mazes for the robot to solve

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
