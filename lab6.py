import math
import sys

obstacles = []

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
    global start, end, obstacles, dimensions
    with open(input_file) as f:
        # Start
        str_start = f.readline().strip().split()
        start = Obstacle()
        start.vertices.append((float(str_start[0]), float(str_start[1])))
        # End
        str_end = f.readline().strip().split()
        end = Obstacle()
        end.vertices.append((float(str_end[0]), float(str_end[1])))
        # World Dimensions
        str_dim = f.readline().strip().split()
        dimensions = (float(str_dim[0]), float(str_dim[1]))
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
        
def main():
  global obstacles
  
  createobstacles("obstacle_map.txt")

  if __name__ == "__main__":
    main()
