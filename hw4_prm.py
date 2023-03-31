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
    Node class for dijkstra search
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
                 ox, oy, robot_radius,dt):
    """
    Run probabilistic road map planning
    :param start_x: start x position
    :param start_y: start y position
    :param goal_x: goal x position
    :param goal_y: goal y position
    :param obstacle_x_list: obstacle x positions
    :param obstacle_y_list: obstacle y positions
    :param robot_radius: robot radius
    :param rng: (Optional) Random generator
    :return:
    """
    #obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)
    print("ox oy",ox,oy)
    obstacleL = [(ox[0], oy[0], 1-dt,'top'), (ox[1], oy[1], 1-dt,'bot')]  
    print("obs list",obstacleL)
    sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y,
                                       robot_radius,
                                       ox, oy)
    #print("sample_x",sample_x,)
    #print("sample_y",sample_y)
    if 2 in sample_x:
        print("X is there")
    if -0.5 in sample_y:
        print("Y is there")    
    if show_animation:
        plt.plot(sample_x, sample_y, ".b")

    road_map = generate_road_map(sample_x, sample_y,
                                 robot_radius,obstacleL)
    #print("road map",road_map)

    rx, ry = dijkstra_planning(
        start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y,obstacleL)

    return rx, ry



def is_collision(sx, sy, gx, gy, rr, obstacle_L):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.hypot(dx, dy)

    D = rr
    n_step = round(d / D)
    #print("n_step",n_step)

    for i in range(n_step):
        if check_collision((x,y),obstacle_L,1,0.02):
            #print("no collison in is_col")
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)
        #else:
        #    print("collision")    

    # goal point check
    
    if not check_collision((gx,gy),obstacle_L,1,0.02):
        True


    return False
def generate_road_map(sample_x, sample_y, rr, obstacleL):
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

        #plot_road_map(road_map, sample_x, sample_y)

    return road_map
def plot_circle(x, y, size, color="-b",orin='top'):  # pragma: no cover
        print("value for circle ",x,y,orin)
        
        if orin=='bot':

            deg = list(range(180, 360, 1))
            deg.append(0)
        else:
            deg = list(range(0, 180, 1))
            deg.append(0)

        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)
                

def dijkstra_planning(sx, sy, gx, gy, road_map, sample_x, sample_y,obstacle_list):
    """
    s_x: start x position [m]
    s_y: start y position [m]
    goal_x: goal x position [m]
    goal_y: goal y position [m]
    obstacle_x_list: x position list of Obstacles [m]
    obstacle_y_list: y position list of Obstacles [m]
    robot_radius: robot radius [m]
    road_map: ??? [m]
    sample_x: ??? [m]
    sample_y: ??? [m]
    @return: Two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty list when no path was found
    """

    start_node = Node(sx, sy, 0.0, -1)
    goal_node = Node(gx, gy, 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    path_found = True
    for (ox, oy, size,ori) in obstacle_list:
            print("circle ",ox,oy)
            plot_circle(ox, oy, size,orin=ori)

    while True:
        if not open_set:
            print("Cannot find path")
            path_found = False
            break

        c_id = min(open_set, key=lambda o: open_set[o].cost)
        current = open_set[c_id]

        # show graph
        
        if show_animation and len(closed_set.keys()) % 2 == 0:
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            print("goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        # Remove the item from the open set
        del open_set[c_id]
        # Add it to the closed set
        closed_set[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.hypot(dx, dy)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            if n_id in closed_set:
                continue
            # Otherwise if it is already in the open set
            if n_id in open_set:
                if open_set[n_id].cost > node.cost:
                    open_set[n_id].cost = node.cost
                    open_set[n_id].parent_index = c_id
            else:
                open_set[n_id] = node

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



def plot_road_map(road_map, sample_x, sample_y):  # pragma: no cover

    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


        
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

    rx, ry = prm_planning(sx, sy, gx, gy, [0,-0], [-1,1], robot_rad,dt)

    assert rx, 'Cannot found path'

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()