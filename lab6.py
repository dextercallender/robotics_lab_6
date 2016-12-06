import math
import sys
import copy

obstacles = []
start = []
goal = []

class Obstacle:
    def __init__(self):
        '''
            Initialize an obstacle
        '''
        self.vertices =  []
        self.original_vertices = []

    def set_vertices(self, new_vertices):
        ''' Set the verticles of the obstacle '''
        self.vertices = new_vertices

def create_obstacles(input_file):
    ''' Create the obstacles and world dimensions from the specified input file '''
    global obstacles
    with open(input_file) as f:
        # Obstacles
        num_obstacles = int(f.readline().strip())
        for i in range(0, num_obstacles):
            num_vertices = int(f.readline().strip())
            obstacle = Obstacle()
            for j in range(0, num_vertices):
                str_vertex = f.readline().strip().split()
                obstacle.vertices.append((float(str_vertex[0]),float(str_vertex[1])))
            obstacle.original_vertices = copy.deepcopy(obstacle.vertices)
            obstacles.append(obstacle)       

def read_start_goal(input_file):
    global start
    global goal
    with open(input_file) as f:
        start_vertex = f.readline().strip().split()
        start.append(start_vertex[0])
        start.append(start_vertex[1])
        goal_vertex = f.readline().strip().split()
        goal.append(goal_vertex[0])
        goal.append(goal_vertex[1])

def main():
    global obstacles
    global start 
    global goal
  
    create_obstacles(sys.argv[1])
    read_start_goal(sys.argv[2])

    for i in range(0, len(obstacles)):
        print( obstacles[i].vertices )

    print( start )
    print( goal )

if __name__ == "__main__":
    main()
