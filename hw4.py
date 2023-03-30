

# def isInSemiCircle(vex, obstacles, radius):
#     #radius = distance(a,b)/2
#     #center_pt = (a+b)/2
#     for obs in obstacles:
#         if obs[1]>0:
#             orientation='up'

#     center_pt = circle_center_1   
#     a=center_pt+radius
#     b=center_pt-radius 
#     vec1 = b - center_pt
#     vec2 = vex - center_pt
#     prod = np.cross(vec1,vec2) 
#     if orientation == 'down':
#         return prod >= 0 and distance(center_pt) <= radius
#     else:
#         return prod <= 0 and distance(vex,center_pt) <= radius


import math
import random
import json, sys, os, argparse

import matplotlib.pyplot as plt
import numpy as np

from scipy.spatial import KDTree

show_animation = True


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0][0])
            self.xmax = float(area[0][1])
            self.ymin = float(area[1][0])
            self.ymax = float(area[1][1])


    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=0.15,
                 path_resolution=0.1,
                 goal_sample_rate=1,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 ):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        play_area:stay inside this area [xmin,xmax,ymin,ymax]
        robot_radius: robot body modeled as circle with given radius
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand_X = rand_area[0][0]
        self.max_rand_X = rand_area[0][1]

        self.min_rand_Y = rand_area[1][0]
        self.max_rand_Y = rand_area[1][1]

        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

    def planning(self, animation=True):
        """
        rrt path planning
        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_if_outside_play_area(new_node, self.play_area) and \
               self.check_collision(
                   new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(
                        final_node, self.obstacle_list, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path
    
        
        

    def planning2(self, animation=True,prob=0.1):
        """
        rrt path planning
        animation: flag for animation on or off
        """
        random_node_gen=0

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_if_outside_play_area(new_node, self.play_area) and \
               self.check_collision(
                   new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)
            
            if random.choices([1,0], weights=(1,int(prob*100)), k=1):
                if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                    final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                    if self.check_collision(
                            final_node, self.obstacle_list, self.robot_radius):
                        return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand_X, self.max_rand_X),
                random.uniform(self.min_rand_Y, self.max_rand_Y)) #-1,1
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, '-o')
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size,ori) in self.obstacle_list:
            self.plot_circle(ox, oy, size,orin=ori)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-3, 3, -1, 1])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b",orin='top'):  # pragma: no cover
        if orin=='bot':

            deg = list(range(180, 360, 1))
            deg.append(0)
        else:
            deg = list(range(0, 180, 1))
            deg.append(0)

        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok

    @staticmethod
    def check_collision(node, obstacleList, robot_radius):

        if node is None:
            return False

        for (ox, oy, size,ori) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size+robot_radius)**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(gx=2, gy=-0.5):
    

        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()





def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Run forward search")
    parser.add_argument(
        "desc",
        metavar="problem_description_path",
        type=str,
        help="path to the problem description file containing the obstacle region in the world as well as the size and shape of the robot, including the width and length of each link, and the distance between two points of attachment",
    )
    parser.add_argument(
        "--prob",
        choices=['1a', '1b', '2', '3'],
        required=False,
        default='1a',
        dest="problem",
        help="default prob 1a",
    )
    args = parser.parse_args(sys.argv[1:])
    return args


def parse_desc(desc):
    """Parse problem description json file to get the problem description"""
    with open(desc) as desc:
        data = json.load(desc)
    O = data["O"]
    C = data["C"]
    xI = list(data["xI"])
    XG=list(data["XG"])

    RAD=data["RADIUS"]
    dt=data["DT"]
    return (O, C, xI, XG, RAD, dt)






if __name__ == '__main__':
    # sys.argv=[os.path.basename(__file__), "hw4_world.json", '--task', '1b']
    args = parse_args()
    (O, C, xI, XG, RAD, dt) = parse_desc(args.desc)
    


    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [(O[0][0], O[0][1], 1-dt,'top'), (O[1][0], O[1][1], 1-dt,'bot')]  # [x, y, radius]
    #obstacleList =[]


    if args.task=='1a':
        rrt = RRT(start=xI,goal=XG,rand_area=C,obstacle_list=[],play_area=C,robot_radius=RAD)
        path = rrt.planning(animation=show_animation)
    #path = rrt.planning2(animation=show_animation)
    
    elif args.task=='1b':
        rrt = RRT(start=xI,goal=XG,rand_area=C,obstacle_list=obstacleList,play_area=C,robot_radius=0.001)
        path = rrt.planning(animation=show_animation)
    elif args.task=='2':
        rrt = RRT(start=xI,goal=XG,rand_area=C,obstacle_list=obstacleList,play_area=C,robot_radius=0.001)
        path = rrt.planning2(animation=show_animation)
    elif args.task=='3':
        path, vertices, edges  = PRM(xI, XG, X, O, RADIUS, DT)
        plot_chain([],path, edges, xI, XG)
    else : 
        rrt = RRT(start=xI,goal=XG,rand_area=C,obstacle_list=[],play_area=C,robot_radius=0.001)
        path = rrt.planning(animation=show_animation)
    
    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
    # Set Initial parameters

    if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()
    

    