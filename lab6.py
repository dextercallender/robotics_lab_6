import math
import sys
import copy
from turtle import *
import random

obstacles = []
start = []
goal = []
dimensions = [600,600]
step_size = 20.0
window = None
start_tree = None

class Obstacle:
    ''' Represents an obstacle on the map '''
    
    def __init__(self):
        ''' Initialize an obstacle with no vertices '''
        self.vertices =  []

    def set_vertices(self, new_vertices):
        ''' Set the verticles of the obstacle '''
        self.vertices = new_vertices

class Node:
    ''' Represents a node in the tree '''
    
    def __init__(self, param_x, param_y):
        ''' Initializes a node at location param_x, param_y '''
        self.x = param_x
        self.y = param_y
        self.predecessor = None
        self.neighbors = []

    def dist_to_point(self, point):
        ''' Get euclidean distance from this node to the given point '''
        return sqrt((self.x - point[0])**2 + (self.y - point[1])**2)

class Line_Segment:
    ''' Represents a line segment (of an object or in a tree '''
    
    def __init__(self, param_x1, param_y1, param_x2, param_y2):
        '''
            Initialize a line segment with endpoints
            (param_x1, param_y1) and (param_x2, param_y2)
        '''
        # Calculate variables for eq of form
        # [x y] + [cos(theta) sin(theta0] * t = [x' y']
        # where theta is the angle of the line seg to the x-axis
        self.x1 = param_x1
        self.y1 = param_y1
        self.x2 = param_x2
        self.y2 = param_y2
        x_change = param_x2 - param_x1
        y_change = param_y2 - param_y1
        dist = sqrt((x_change)**2 + (y_change)**2)
        self.delta_x = x_change / dist # equivalent to cos(theta)
        self.delta_y = y_change / dist # equivalent to sin(theta)
        self.t = self.solve_t(param_x2, param_y2) # time t to get from (x1, y1) to (x2, y2)
        # Calculate variables for eq of form y = mx + b
        self.slope = y_change / x_change if x_change != 0 else float('inf')
        self.b = param_y1 - self.slope * param_x1

    def is_on_line(self, target_x, target_y):
        if self.slope == float('inf'):
            if x == target_x:
                return True
            else:
                return False
        my_y = self.slope * target_x + self.b
        if my_y == target_y:
            return True
        return False
    
    def solve_t(self, x2, y2):
        '''
            Given x and y solve for time to intersection to that point
        '''
        return (x2 - self.x1) / self.delta_x \
               if self.delta_x != 0 else (y2 - self.y1) / self.delta_y

    def solve_x_y(self, t):
        '''
            Given a time to intersection solve for x and y
        '''
        result_x = self.delta_x * t + self.x1
        result_y = self.delta_y * t + self.y1
        return (result_x, result_y)
    
    def intersect_point(self, other_seg):
        '''
            Get the intersection between this line segment
            and the given line segment
        '''
        # Same slopes shouldn't intersect
        if other_seg.slope == self.slope:
            return None
        result_x = 0
        # Case if this segment is vertical
        if self.slope == float('inf'):
            result_x = self.x1
        # Case if this other segment is vertical
        elif other_seg.slope == float('inf'):
            result_x = other_seg.x1
        else:
            result_x = (self.b - other_seg.b) / (other_seg.slope - self.slope)
        result_y = self.slope * result_x + self.b if self.slope != float('inf') else other_seg.slope * result_x + other_seg.b
        return (result_x,result_y)

    def intersects(self, other_seg):
        # Get the intersection point
        intersect = self.intersect_point(other_seg)
        if intersect:
            intersect_x = intersect[0]
            intersect_y = intersect[1]
            # Get the times to intersection for both line segments
            other_intersect_t = other_seg.solve_t(intersect_x, intersect_y)
            self_intersect_t = self.solve_t(intersect_x, intersect_y)
            # Variable for accommodate for floating point error
            floating_error = 0.0000001
            # Check that it lies on both line segments
            if self_intersect_t > floating_error and \
               self_intersect_t < self.t - floating_error and \
               other_intersect_t > floating_error and \
               other_intersect_t < other_seg.t - floating_error:
                return True
        return False

    def modify_to_step_size(self):
        '''
            Make the line segment step size long
            instead of the originally given second endpoint
        '''
        # Get unit vector
        length = sqrt(self.delta_x**2 + self.delta_y**2)
        self.delta_x /= length 
        self.delta_y /= length
        # Calculate new endpoint/time to that point
        self.x2 = self.x1 + (self.delta_x * step_size)
        self.y2 = self.y1 + (self.delta_y * step_size)
        self.t = self.solve_t(self.x2, self.y2)        
                            
def create_obstacles(input_file):
    ''' Create the obstacles and world dimensions from the specified input file '''
    global obstacles
    with open(input_file) as f:
        # Read number of obstacles
        num_obstacles = int(f.readline().strip())
        # Read in each obstacle
        for i in range(0, num_obstacles):
            num_vertices = int(f.readline().strip())
            obstacle = Obstacle()
            for j in range(0, num_vertices):
                str_vertex = f.readline().strip().split()
                obstacle.vertices.append((float(str_vertex[0]),float(str_vertex[1])))
            obstacles.append(obstacle)       

def read_start_goal(input_file):
    ''' Read start/goal from input file '''
    global start
    global goal
    with open(input_file) as f:
        start_vertex = f.readline().strip().split()
        start.append(float(start_vertex[0]))
        start.append(float(start_vertex[1]))
        goal_vertex = f.readline().strip().split()
        goal.append(float(goal_vertex[0]))
        goal.append(float(goal_vertex[1]))

def draw_inputs():
    ''' Use turtle to draw the inputs parsed from the input file '''
    global window
    # Create turtle window
    window = Screen()
    window.reset()
    window.setworldcoordinates(0, 0, max(dimensions) + 4, \
                               max(dimensions) + 4)
    # Draw the obstacles
    red = Turtle()
    red.speed(0)
    red.hideturtle()
    red.color("red")
    red.penup()
    for obstacle in obstacles:
        first_vertex = None
        for vertex in obstacle.vertices:
            if not first_vertex:
                first_vertex = vertex
            red.setpos(vertex[0], vertex[1])
            red.pendown()
        red.setpos(first_vertex[0], first_vertex[1])
        red.penup()
    # Draw the world boundaries
    red.setpos(0,0)
    red.pendown()
    red.setpos(dimensions[0], 0)
    red.setpos(dimensions[0], dimensions[1])
    red.setpos(0, dimensions[1])
    red.setpos(0,0)
    red.penup()
    # Draw start/goal points
    red.setpos(start[0], start[1])
    red.pendown()
    red.circle(2)
    red.penup()
    red.setpos(goal[0], goal[1])
    red.pendown()
    red.circle(2)

def generate_random_point(iteration_num):
    '''
        Generate a semi-random point in the map
    '''
    # Bias to goal
    # Every 20th iteration, set "random" point to the goal
    if iteration_num % 20 == 0:
        return goal
    # Every 5th iteration, set "random" point to area near goal
    elif iteration_num % 5 == 0:
        x = max(0, min((goal[0] + random.randint(-50, 50)), dimensions[0]))
        y = max(0, min((goal[1] + random.randint(-50, 50)), dimensions[1]))
        return [x, y]
    # Remainig iteration, choose uniformly random point
    else:
        x = random.randint(0, dimensions[0])
        y = random.randint(0, dimensions[1])
        return [x, y]

def find_closest_node(point, node):
    '''
        Recursively get the closest node to the given point
    '''
    # Get dist for this node
    closest_dist = node.dist_to_point(point)
    closest_node = node
    # Get dist for each neighbor
    for neighbor in node.neighbors:
        neighbor_closest_node, neighbor_dist = find_closest_node(point, neighbor)
        # Keep smallest dist/node
        if neighbor_dist < closest_dist:
            closest_dist = neighbor_dist
            closest_node = neighbor_closest_node
    return closest_node, closest_dist

def get_new_node_if_valid(random_point, closest_node, line_segs):
    '''
        Get the location of the new node, check if it is valid,
        if so, return the new node, otherwise return None
    '''
    # Get line segment for this step
    new_line_seg = Line_Segment(closest_node.x, closest_node.y, random_point[0], random_point[1])
    new_line_seg.modify_to_step_size()
    # Make sure the point is within bounds
    if new_line_seg.x2 < 0 or new_line_seg.x2 > dimensions[0] \
       or new_line_seg.y2 < 0 or new_line_seg.y2 > dimensions[0]:
        return None
    # Make sure it doesn't intersect any obstacles
    for line_seg in line_segs:
        if new_line_seg.intersects(line_seg):
            return None
    # If the goal is on the line segment of the step, check if the goal
    # is within reach
    if random_point == goal or new_line_seg.is_on_line(goal[0], goal[1]):
        t_to_goal = new_line_seg.solve_t(goal[0], goal[1])
        # If so, return the goal
        if t_to_goal <= new_line_seg.t:
            return Node(goal[0], goal[1])
    return Node(new_line_seg.x2, new_line_seg.y2)

def build_tree():
    ''' Build and draw the tree '''
    global start_tree
    # Turtle graphics
    orange = Turtle()
    orange.speed(0)
    orange.hideturtle()
    orange.color("orange")
    orange.penup()
    # Initialize variables
    iteration_num = 0
    start_tree = Node(start[0], start[1])
    # Draw first node at start
    orange.setpos(start[0], start[1])
    orange.pendown()
    orange.circle(1)
    line_segs = []
    # Create a list of the line segments of the objects
    for j in range(0, len(obstacles)):
        obstacle = obstacles[j]
        for i in range(0, len(obstacle.vertices)):
            # Add the line segment of the current vertex and the next vertex
            if len(obstacle.vertices) > 1:
                next_index = (i + 1) % len(obstacle.vertices)
                line_seg = Line_Segment(obstacle.vertices[i][0], obstacle.vertices[i][1], obstacle.vertices[next_index][0], obstacle.vertices[next_index][1])
                line_segs.append(line_seg)
    # Start building the tree
    while True:
        # Get random point
        random_point = generate_random_point(iteration_num)
        # Get closest node to the random point
        closest_node, closest_dist = find_closest_node(random_point, start_tree)
        # Get the new node if it doesn't collide with an obstacle
        new_node = get_new_node_if_valid(random_point, closest_node, line_segs)
        if new_node:
            # Add the new node to the tree
            new_node.predecessor = closest_node
            closest_node.neighbors.append(new_node)
            # Draw line from predecessor to new node
            orange.setpos(closest_node.x, closest_node.y)
            orange.pendown()
            orange.setpos(new_node.x, new_node.y)
            orange.circle(2)
            orange.penup()
            # If new_node is the goal, stop
            if new_node.x == goal[0] and new_node.y == goal[1]:
                return new_node
        iteration_num += 1

def get_shortest_path(goal):
    ''' Get the shortest path to the goal from the tree '''
    path = []
    node = goal
    while node:
        path.append(node)
        node = node.predecessor
    return path

def draw_path(path):
    ''' Draw the given path '''
    blue = Turtle()
    blue.speed(0)
    blue.hideturtle()
    blue.color("blue")
    blue.penup()
    for node in path:
        blue.setpos(node.x, node.y)
        blue.pendown()
    
def main():
    global obstacles, start, goal, window, step_size

    if len(sys.argv) < 4:
        print 'Usage: python lab6.py [obstacle file] [start/goal file] [step_size]'
        return

    # Parse inputs    
    create_obstacles(sys.argv[1])
    read_start_goal(sys.argv[2])
    step_size = float(sys.argv[3])

    # Draw inputs
    draw_inputs()

    # Build tree
    goal = build_tree()

    # Get shortest path
    path = get_shortest_path(goal)

    # Draw path
    draw_path(path)

    # Close turtle window on click
    window.exitonclick()

if __name__ == "__main__":
    main()
