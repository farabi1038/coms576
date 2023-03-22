#############################################################################################################
# COM S 476/576 Project 2 Solution
# Tichakorn Wongpiromsarn
#############################################################################################################

import json, sys, os, argparse
import matplotlib.pyplot as plt
from discrete_search import fsearch, ALG_BFS
from hw1 import Grid2DStates, GridStateTransition, Grid2DActions, draw_path
import numpy as np
from shapely.geometry import Polygon
from hw2_chain_plotter import  get_link_positions
import math 

LINK_ANGLES = [i - 180 for i in range(360)]


def check_coll(configX,configY,W,L,D,O):
            joint,linkV = get_link_positions((configX,configY),W,L,D)
            # x1 = D * np.cos(np.deg2rad(i))
            # y1 = D * np.sin(np.deg2rad(i))
            # x2 = x1 + L * np.cos(np.deg2rad(i + j))
            # y2 = y1 + L * np.sin(np.deg2rad(i + j))

            link1_poly = Polygon([linkV[0][0],linkV[0][1],linkV[0][2],linkV[0][3]])
            link2_poly = Polygon([linkV[1][0],linkV[1][1],linkV[1][2],linkV[1][3]])
            for obstacle in O:
                obstacle_poly = Polygon([obstacle[0],obstacle[1],obstacle[2],obstacle[3]])
                if obstacle_poly.intersects(link1_poly) or obstacle_poly.intersects(link2_poly):
                    return True
            
            return False


def compute_Cobs(O, W, L, D):
    """Compute C-Space obstacles for a 2-link robot

    @type O:   a list of obstacles, where for each i, O[i] is a list [(x_0, y_0), ..., (x_m, y_m)]
               of coordinates of the vertices of the i^th obstacle
    @type W:   float, representing the width of each link
    @type L:   float, representing the length of each link
    @type D:   float, the distance between the two points of attachment on each link

    @return: a list of configurations (theta_1, theta_2) of the robot that leads to a collision
        between the robot and an obstacle in O.
    """
    # TODO: Implement this function

    Cobs = []
    for i in range(-180, 180):
        for j in range(-180, 180):
            configX = np.deg2rad(i)
            configY = np.deg2rad(j)
            
            if check_coll(configX,configY,W,L,D,O):
                Cobs.append((i, j))
            
    print("length of Cobs",len(Cobs))
    return Cobs
    raise NotImplementedError


def compute_Cfree(Cobs):
    """Compute the free space for a 2-link robot

    @type Cobs: a list of configurations (theta_1, theta_2) of the robot that leads to a collision
                between the robot and an obstacle in O.

    @return an instance of Grid2DStates that represents the free space
    """
    # TODO: Implement this function

    # Cfree = Grid2DStates(-180,180,-180,180,Cobs)
    # C = Grid2DStates(-180,180,-180,180,Cobs)
    # for c in range(-180,180):
    #     if c not in Cobs:
    #         Cfree.add(c)
    
    return Grid2DStates(-180,180,-180,180,Cobs)
    raise NotImplementedError


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
        "--out",
        metavar="output_path",
        type=str,
        required=False,
        default="",
        dest="out",
        help="path to the output file",
    )

    args = parser.parse_args(sys.argv[1:])
    if not args.out:
        args.out = os.path.splitext(os.path.basename(args.desc))[0] + "_out" + ".json"

    print("Problem description: ", args.desc)
    print("Output:              ", args.out)

    return args


def parse_desc(desc):
    """Parse problem description json file to get the problem description"""
    with open(desc) as desc:
        data = json.load(desc)

    O = data["O"]
    W = data["W"]
    L = data["L"]
    D = data["D"]
    xI = tuple(data["xI"])
    XG = [tuple(x) for x in data["XG"]]
    U = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    return (O, W, L, D, xI, XG, U)

def check_collision(point, obstacles):
    """Check if a point collides with any of the obstacles."""
    print("asasasasasasas",point,obstacles)
    for obstacle in obstacles:
        if point[0] >= obstacle[0][0] and point[0] <= obstacle[1][0] and \
           point[1] >= obstacle[0][1] and point[1] <= obstacle[2][1]:
            return True
    return False

def cal(p1,p2):
    
    if p1[0]==p2[0]:
  
        list1=[]
        list2= np.arange(p1[1],p2[1],0.1)
        for i in list2:
            list1.append((p1[0],round(i,1)))
        return list1       

    elif p1[1]==p2[1]:
        list1=[]
        list2= np.arange(p1[0],p2[0],0.1)
        for i in list2:
            list1.append((round(i,1),p1[1]))
        return list1 
    else:
        # if p1[1]>p2[1]:
        #     p1[1],p2[1]=p2[1],p1[1]
        #     p1[0],p2[0]=p2[0],p1[0]

        # if p1[0]>p2[0]:
        #     p1[1],p2[1]=p2[1],p1[1]
        #     p1[0],p2[0]=p2[0],p1[0]     


        list1=[]
        list2= np.arange(p1[0],p2[0],0.1)
        list3= np.arange(p1[1],p2[1],0.1)

        for i in range(len(list2)):
            
            

            list1.append((round(list2[i],1),round(list3[i],1)))


        

def interpolate_path(elements):
    fpath = []
    
    interpolated_points = []

    # Iterate through each pair of consecutive elements and interpolate points with step size 0.1
    for i in range(len(elements) - 1):
        # Calculate the difference between the two elements
        p1 = elements[i]
        p2 = elements[i+1]

        cor=cal(p1,p2)
        #ycor=cal(p1,p2)
        #interpolated_point = [list(a) for a in zip(xcor,ycor)]
        for i in cor:
            interpolated_points.append(i)

    # Add the last element to the interpolated points
    interpolated_points.append(elements[-1])

    # Print the interpolated points
    print(interpolated_points)
    return interpolated_points




if __name__ == "__main__":
    args = parse_args()
    (O, W, L, D, xI, XG, U) = parse_desc(args.desc)
    Cobs = compute_Cobs(O, W, L, D)

    X = compute_Cfree(Cobs)
    f = GridStateTransition()
    U = Grid2DActions(X, f)

    search_result = fsearch(X, U, f, xI, XG, ALG_BFS)

    result = {"Cobs": Cobs, "path": search_result["path"]}



    print("length of fpath", len(search_result["path"]))
    fpath= interpolate_path(search_result["path"])

    #print("length of fpath", len(fpath))
    #print("points in fpath", fpath)
    fpath_col=[]
    for config in fpath:
        if check_coll(config[0],config[1],W,L,D,O):    
            #print("inside check")
            fpath_col.append(config)      


    print("collisions",fpath_col)
    #print("length of f_col", len(fpath_col))
    with open(args.out, "w") as outfile:
        json.dump(result, outfile)

    fig, ax = plt.subplots()
    X.draw(ax, grid_on=False, tick_step=[30, 30])
    draw_path(ax, search_result["path"])
    plt.show()
