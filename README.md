Lab 6 Readme
Group 10
Team members: Dexter Callender III (dec2148), Daniel Hong (sh3266), Yanrong Wo (yw2513)

Overall Usage:
	python lab6.py [obstacle file] [start/goal file] [step size] [lab part (1,2,e)]
	
Part 1: RRT
	File: 
		lab6.py
	Usage:
		python lab6.py [obstacle file] [start/goal file] [step size] 1
		
		example: python lab6.py obstacle_map.txt start_and_goal.txt 30 1

		The program will start drawing the obstacles in red followed by the tree in orange. It will print each iteration number as it goes so the user knows that the program is still running. Once a path is found, it will be drawn in blue. Click on the turtle window to end the program.

	Basic Algorithm:
		1) Parse input files and create objects
		2) Draw the objects
		3) Build the tree
			-Generate a semi-random point.
				- Every 20th iteration, set the point to the goal
				- Every 5th iteration, set the point to somewhere near the goal (goal_x +/- 50, goal_y +/- 50)
				- Otherwise, pick a random point within the dimensions
			-Find the closest node to this point.
			-If it is valid to move one step size from this closest node to towards the random point, then add this node to the tree. 
			-Repeat until the new node added is the goal
		4) Traverse the tree back from the goal to get the shortest path. Reverse this path so that it is from start to goal.
		5) Draw this path

Part 2: Bi-directional RRT
	File: 
		lab6.py
	Usage:
		python lab6.py [obstacle file] [start/goal file] [step size] 2
		
		example: python lab6.py obstacle_map.txt start_and_goal.txt 30 2

		The program will start drawing the obstacles in red followed by the trees in orange (from the start) and green (from the goal). It will print each iteration number as it goes so the user knows that the program is still running. Once a path is found, it will be drawn in blue. Click on the turtle window to end the program.

	Basic Algorithm:
		1) Parse input files and create objects
		2) Draw the objects
		3) Build the tree
			-Generate a uniformly random point.
			-Find the closest node to this point from the start tree. 
			-Find the closest node to this point from the goal tree. 
			-Check if it is valid to move one step size from the closest_node in the start_tree towards the random point. 
			-Check if it is valid to move one step size from the closest_node in the goal tree towards the random point. 
			- Check if the random point is within reach from both of the nodes. If so, set both nodes to the random point.
			- Add the nodes if they are valid.
			- If either node was valid, find the closest node in the other tree and see if you can make a connection between the closest node in the other tree and this new node. If so, add in this connection.
			- Repeat until you come across an interation in which the x,y values of the nodes are the same.
		4) Traverse the two trees to create the path. 
			- Given the two nodes where the trees met. Traverse the start tree from the start tree's meeting node to the start. Reverse it. Append to this the traversal of the meeting node of the goal tree to the goal.
		5) Draw this path
	Note:
		We chose the modify the original bi-directional tree algorithm. The original algorithm choose a random point, grows the start tree towards the random point, then grows the goal tree towards the new point in the start tree. However, we found that this often keeps the goal tree in the test scenario to not grow because, the start tree is normally to the left and below the goal point and the goal is blocked off by an obstacle in that direction. We chose instead to grow both trees by the random point; this increases the chance of being able to get out of the obstacle surrounding the goal. 

Extra Credit: Translation/Rotation
	File: 
		lab6.py
	Usage:
		python lab6.py [obstacle file] [start/goal file] [step size] e
		
		example: python lab6.py obstacle_map.txt extra_credit_start_goal.txt 30 e

		The program will start drawing the obstacles in red followed by the trees in orange (from the start) and green (from the goal). It will print each iteration number as it goes so the user knows that the program is still running. Once a path is found, it will be drawn in blue. The program will then continuously draw the robot's path from start to goal (including the robot's orientation). To end the program, Ctrl+C or kill it from the command line. 
	Basic Algorithm:
		Same as part 2, but when checking if a step is valid, we generate a new angle for the robot that is within +/- step size degrees of the current angle of the robot. Then for every centimeter of the step size, we add in all the lines of the robot translated by the distance and rotated slightly towards the new angle. We then check to make sure all of these lines do not intersect any obstacles. 
	Note:
		Please note that the start/goal file for this part is different. As stated in the lab, "The reference point on the robot is its centroid (10,25), and the robot can rotate about this point. Find an RRT collision free path for this robot using the obstacle set above where the reference point starts at location (75,50) in zero degree configuration and the goal is the reference point at location (482,577) with the robot rotated +90 degrees about its reference point (robot is horizontal)". Thus for this part we used a start/goal file with start = (75,50) and goal = (482, 577).
