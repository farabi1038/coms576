import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from shapely.geometry import Polygon, Point
# parameter
N_SAMPLE = 1000  # number of sample_points
N_KNN = 15  # number of edge from one sampled point

show_animation = True


class Node:
    """
    Node class for dijkstra algorithm .The structure is taken from a online source
    """

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," +\
               str(self.cost) + "," + str(self.parent_index)


def sub_planning(start_x, start_y, goal_x, goal_y,
                 obstacle_x_list, obstacle_y_list, robot_radius,dt=0.02):
    """
    Run probabilistic road map planning
    :param start_x: start x position
    :param start_y: start y position
    :param goal_x: goal x position
    :param goal_y: goal y position
    :param obstacle_x_list: obstacle x positions
    :param obstacle_y_list: obstacle y positions
    :param rr: robot radius
    :param dt: (Optional)tunable parameter
    :return:
    """
    #First calculating the obstracles in the map
    #print("ox obstacle_y_list",obstacle_x_list,obstacle_y_list)
    obstacleL = [(obstacle_x_list[0], obstacle_y_list[0], 1-dt,'top'), (obstacle_x_list[1], obstacle_y_list[1], 1-dt,'bot')] 

    # calculating CFREE space for search avoiding obstracles in the map 
    sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y,
                                       robot_radius-dt,
                                       obstacle_x_list, obstacle_y_list)
    #print("sample_x",sample_x,)
    #print("sample_y",sample_y)
    #if 2 in sample_x:
        #print("X is there")
    #if -0.5 in sample_y:
        #print("Y is there")    
    if show_animation:
        plt.plot(sample_x, sample_y, ".b")

    #calculating road map for dijkstra
    road_map = road_map_gen(sample_x, sample_y,
                                 robot_radius-dt,obstacleL)
    #print("road map",road_map)

    #finding existing path using dijkstra
    rx, ry = dijkstra_search(
        start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y,obstacleL)

    return rx, ry


#Checking for collison between possible two points
def is_collision(start_x, start_y, goal_x, goal_y, rr, obstacle_L):
    x = start_x
    y = start_y
    dx = goal_x - start_x
    dy = goal_y - start_y
    yaw = math.atan2(goal_y - start_y, goal_x - start_x)
    d = math.hypot(dx, dy)

    D = rr
    n_step = round(d / D)
    #print("n_step",n_step)

    for i in range(n_step):
        if check_collision((x,y),obstacle_L,rr):
            #print("no collison in is_col")
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)
        #else:
        #    print("collision")    

    # goal point check
    
    if not check_collision((goal_x,goal_y),obstacle_L,rr):
        True


    return False

#the road map avoiding the obstracles also building the edge list of neigbours 
#  
def road_map_gen(sample_x, sample_y, rr, obstacleL):
    """
    Road map generation
    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    robot_radius: Robot Radius[m]
    obstacle_kd_tree: KDTree object of obstacles
    """
    #obstacleL = [(ox[0], ox[1], 1-rr,'top'), (oy[0], oy[1], 1-rr,'bot')] 
    road_map = []
    n_sample = len(sample_x)
    sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

    #building the edge list of neigbours     

    for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

        dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
        #print("indexes",len(indexes))
        edge_id = []

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]
            #print("tuple",is_collision(ix, iy, nx, ny, rr, obstacleL))

            if not is_collision(ix, iy, nx, ny, rr, obstacleL):
                #print("no collision")
                edge_id.append(indexes[ii])   

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)


    return road_map
 
 #this function is for plotting the both obstracle
def plot_circle(x, y, size, color="-b",orin='top'):  # pragma: no cover
        #print("value for circle ",x,y,orin)
        
        if orin=='bot':

            deg = list(range(180, 360, 1))
            deg.append(0)
        else:
            deg = list(range(0, 180, 1))
            deg.append(0)

        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)
                

#finding the shortest path avoiding obstracle using dijkstra. 

def dijkstra_search(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y,obstacle_list):
    """
    Plan a path from start position to goal position avoiding obstacles using Dijkstra algorithm.
    :param start_x: start x position
    :param start_y: start y position 
    :param goal_x: goal x position 
    :param goal_y: goal y position 
    :param obstacle_list: list of tuples (x, y, size, orientation) representing obstacles [m]
    :param road_map: a graph where nodes are connected by edges representing possible movements
    :param sample_x: x positions of nodes in the graph [m]
    :param sample_y: y positions of nodes in the graph [m]
    :return: two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty lists if no path was found
    """
    # Create start and goal nodes
    start_node = Node(start_x, start_y, 0.0, -1)
    goal_node = Node(goal_x, goal_y, 0.0, -1)

    # Create open and closed sets for roadmap
    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    # Find the path using Dijkstra algorithm
    path_found = True
    for (obstacle_x_list, obstacle_y_list, size,ori) in obstacle_list:
            #print("circle ",obstacle_x_list,obstacle_y_list)
            plot_circle(obstacle_x_list, obstacle_y_list, size,orin=ori)

    while True:
        if not open_set:
            #print("Cannot find path")
            path_found = False
            break
        # Select node with lowest cost from the open set    
        c_id = min(open_set, key=lambda o: open_set[o].cost)
        current = open_set[c_id]

        # Show the graph if animation is enabled
        
        if show_animation and len(closed_set.keys()) % 2 == 0:
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        # Goal found
        if c_id == (len(road_map) - 1):
            print("goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        # Remove current node from open set and add it to closed set
        del open_set[c_id]
        # Add it to the closed set
        closed_set[c_id] = current

        # Expand search grid based on motion
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.hypot(dx, dy)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)
            # Check if node is already in the closed set
            if n_id in closed_set:
                continue
            # Check if node is already in the open set and update its cost if necessary
            if n_id in open_set:
                if open_set[n_id].cost > node.cost:
                    open_set[n_id].cost = node.cost
                    open_set[n_id].parent_index = c_id
            else:
                open_set[n_id] = node

    if path_found is False:
        return [], []

    # generate final road 
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry



def plot_road_map(road_map, sample_x, sample_y):  # pragma: no cover

    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


        
def check_collision(point, obstacleList, robot_radius):
    bol_flag=True
    point_checked=Point(point[0], point[1])
    #print("radius",robot_radius)
    for obs in obstacleList:
        obs_origin_point=Point(obs[0],obs[1])
        circle_area=obs_origin_point.buffer(robot_radius)
        #print("actual point",point_checked)
        #print("obs origi",obs_origin_point)
        if circle_area.contains(point_checked):
            #print("inside nocollision")
            bol_flag=False
            break
        
        else:
            bol_flag=True
            #print("going backkkkkk")
    #print("final return",bol_flag)
    return bol_flag


def sample_points(start_x, start_y, goal_x, goal_y, rr, obstacle_x_list, obstacle_y_list):
    max_x = 3
    max_y = 1
    min_x = -3
    min_y = -1
    obstacleL = [(obstacle_x_list[0], obstacle_y_list[0], 1-rr,'top'), (obstacle_x_list[1], obstacle_y_list[1], 1-rr,'bot')]  # [x, y, radius]

    sample_x, sample_y = [], []

    rng = np.random.default_rng()

    while len(sample_x) <= N_SAMPLE:
        tx = (rng.random() * (max_x - min_x)) + min_x
        ty = (rng.random() * (max_y - min_y)) + min_y
        tx = np.around(tx, 1)
        ty = np.around(ty, 1)


        if check_collision((tx,ty),obstacleL,rr):
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(start_x)
    sample_y.append(start_y)
    sample_x.append(goal_x)
    sample_y.append(goal_y)

    return sample_x, sample_y





def main_prm(start,goal,O,rr,dt):
    print(__file__ + " start!!",start,goal,O,rr,dt)

    # start and goal position
    start_x = start[0]  
    start_y = start[1] 
    goal_x = goal[0] 
    goal_y = goal[1]  
    robot_rad = rr  
    dt=dt
    obstacle_x_list = []
    obstacle_y_list = []

   

    if show_animation:
        
        plt.plot(obstacle_x_list, obstacle_y_list, ".k")
        plt.plot(start_x, start_y, "^r")
        plt.plot(goal_x, goal_y, "^c")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = sub_planning(start_x, start_y, goal_x, goal_y, [O[0][0], O[1][0]], [O[0][1], O[1][1]], rr,dt)

    assert rx, 'Cannot found path'

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()

