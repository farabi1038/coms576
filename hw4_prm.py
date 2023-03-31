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
    A class representing a node in Dijkstra's search algorithm.
    """

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," +\
               str(self.cost) + "," + str(self.parent_index)


def prm_planning(start_x, start_y, goal_x, goal_y,
                 obstacle_x_list, obstacle_y_list, robot_radius):
    """
    Run Probabilistic Road Map (PRM) planning algorithm.

    Args:
        start_x (float): The x-coordinate of the starting position.
        start_y (float): The y-coordinate of the starting position.
        goal_x (float): The x-coordinate of the goal position.
        goal_y (float): The y-coordinate of the goal position.
        obstacle_x_list (list): A list of x-coordinates of the obstacle positions.
        obstacle_y_list (list): A list of y-coordinates of the obstacle positions.
        robot_radius (float): The radius of the robot.

    Returns:
        tuple: A tuple of two lists: the x-coordinates and y-coordinates of the planned path.
    """
    
    # Define the time step for the simulation
    dt = 0.5
    
    # Define the obstacles as a list of tuples
    obstacle_list = [(obstacle_x_list[0], obstacle_y_list[0], 1-dt, 'top'), 
                     (obstacle_x_list[1], obstacle_y_list[1], 1-dt, 'bottom')]
    
    # Sample points between the start and goal positions, avoiding obstacles
    sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y, 1-dt, obstacle_x_list, obstacle_y_list)
    
    # Plot the sampled points, if show_animation flag is set to True
    if show_animation:
        plt.plot(sample_x, sample_y, ".b")
    
    # Generate the roadmap connecting the sampled points
    road_map = generate_road_map(sample_x, sample_y,1-dt, obstacle_list)
    print("-------------")
    print(road_map)
    print("-------------")
    
    # Plan the path using Dijkstra's search algorithm
    rx, ry = dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y, obstacle_list)

    return rx, ry




def is_collision(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_list):
    """
    Check if there's any collision between a straight path between start and goal points
    and obstacles in the environment
    :param start_x: x-coordinate of start point
    :param start_y: y-coordinate of start point
    :param goal_x: x-coordinate of goal point
    :param goal_y: y-coordinate of goal point
    :param robot_radius: radius of the robot
    :param obstacle_list: list of obstacles in the environment
    :return: boolean value indicating whether there's a collision or not
    """
    x = start_x
    y = start_y
    dx = goal_x - start_x
    dy = goal_y - start_y
    yaw = math.atan2(goal_y - start_y, goal_x - start_x)
    d = math.hypot(dx, dy)

    D = robot_radius
    n_step = round(d / D)

    for i in range(n_step):
        if check_collision((x,y), obstacle_list, 1, 0.02):
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)

    if not check_collision((goal_x, goal_y), obstacle_list, 1, 0.02):
        return True

    return False


def generate_road_map(sample_x, sample_y, robot_radius, obstacle_list):
    """
    Generates a road map for a given set of sample points and obstacles in the environment
    :param sample_x: list of x-coordinates of sample points
    :param sample_y: list of y-coordinates of sample points
    :param robot_radius: radius of the robot
    :param obstacle_list: list of obstacles in the environment
    :return: the generated road map
    """
    road_map = []
    n_sample = len(sample_x)
    sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

        dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
        edge_id = []

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]

            if not is_collision(ix, iy, nx, ny, robot_radius, obstacle_list):
                edge_id.append(indexes[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    return road_map

def plot_circle(x_center, y_center, radius, color="-b", orientation='top'):
    """
    Plots a circle with center at (x_center, y_center) and radius 'radius' using Matplotlib.
    'color' specifies the color of the plot, and 'orientation' specifies whether the circle is plotted from the top or bottom.
    """
    if orientation == 'bot':
        # plot circle from the bottom
        degrees = list(range(180, 360, 1))
        degrees.append(0)
    else:
        # plot circle from the top
        degrees = list(range(0, 180, 1))
        degrees.append(0)

    # calculate x and y coordinates for the circle
    x_list = [x_center + radius * math.cos(np.deg2rad(d)) for d in degrees]
    y_list = [y_center + radius * math.sin(np.deg2rad(d)) for d in degrees]

    # plot the circle
    plt.plot(x_list, y_list, color)
                

def dijkstra_planning(start_x, start_y, goal_x, goal_y, obstacle_list, road_map, sample_x, sample_y):
    """
    Plan a path from start position to goal position avoiding obstacles using Dijkstra algorithm.
    :param start_x: start x position [m]
    :param start_y: start y position [m]
    :param goal_x: goal x position [m]
    :param goal_y: goal y position [m]
    :param obstacle_list: list of tuples (x, y, size, orientation) representing obstacles [m]
    :param road_map: a graph where nodes are connected by edges representing possible movements
    :param sample_x: x positions of nodes in the graph [m]
    :param sample_y: y positions of nodes in the graph [m]
    :return: two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty lists if no path was found
    """

    # Create start and goal nodes
    start_node = Node(start_x, start_y, 0.0, -1)
    goal_node = Node(goal_x, goal_y, 0.0, -1)

    # Create open and closed sets
    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    # Find the path using Dijkstra algorithm
    path_found = True
    while True:
        if not open_set:
            print("Cannot find path")
            path_found = False
            break

        # Select node with lowest cost from the open set
        current_id = min(open_set, key=lambda o: open_set[o].cost)
        current_node = open_set[current_id]

        # Show the graph if animation is enabled
        if show_animation and len(closed_set.keys()) % 2 == 0:
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(current_node.x, current_node.y, "xg")
            plt.pause(0.001)

        # Goal found
        if current_id == (len(road_map) - 1):
            print("Goal is found!")
            goal_node.parent_index = current_node.parent_index
            goal_node.cost = current_node.cost
            break

        # Remove current node from open set and add it to closed set
        del open_set[current_id]
        closed_set[current_id] = current_node

        # Expand search grid based on motion model
        for i, node_id in enumerate(road_map[current_id]):
            dx = sample_x[node_id] - current_node.x
            dy = sample_y[node_id] - current_node.y
            d = math.hypot(dx, dy)
            node = Node(sample_x[node_id], sample_y[node_id], current_node.cost + d, current_id)

            # Check if node is already in the closed set
            if node_id in closed_set:
                continue

            # Check if node is already in the open set and update its cost if necessary
            if node_id in open_set:
                if open_set[node_id].cost > node.cost:
                    open_set[node_id].cost = node.cost
                    open_set[node_id].parent_index = current_id
            else:
                open_set[node_id] = node

    if path_found is False:
        return [], []



    # generate final course
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry


      


        
def check_collision(point, obstacleList, robot_radius,dt):

    flag=False
    point_checked=Point(point[0], point[1])
    #print(obstacleList)
    for obs in obstacleList:
        obs_origin_point=Point(obs[0],obs[1])
        circle_area=obs_origin_point.buffer(robot_radius-dt)
        if circle_area.contains(point_checked):
            return True
    return False


def sample_points(sx, sy, gx, gy, rr, ox, oy):
    max_x = 3
    max_y = 1
    min_x = -3
    min_y = -1
    obstacleL = [(ox[0], oy[0], 1-rr,'top'), (ox[1], oy[1], 1-rr,'bot')]  # [x, y, radius]

    sample_x, sample_y = [], []

    rng = np.random.default_rng()

    while len(sample_x) <= N_SAMPLE:
        tx = (rng.random() * (max_x - min_x)) + min_x
        ty = (rng.random() * (max_y - min_y)) + min_y
        tx = np.around(tx, 1)
        ty = np.around(ty, 1)


        if check_collision((tx,ty),obstacleL,0.1,0.02):
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y


def main(rng=None):
    print(__file__ + " start!!")

    # start and goal position
    sx = -2  # [m]
    sy = -0.5  # [m]
    gx = 2  # [m]
    gy = -0.5  # [m]
    robot_rad = 0.01  # [m]
    dt=0.02

    ox = []
    oy = []

   

    if show_animation:
        
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "^r")
        plt.plot(gx, gy, "^c")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = prm_planning(sx, sy, gx, gy, [0,-0], [-1,1], robot_rad)

    assert rx, 'Cannot found path'

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)


if __name__ == '__main__':
    main()