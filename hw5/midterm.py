import sys, argparse
import matplotlib.pyplot as plt
from planning import (
    rrt,
    prm,
    StraightEdgeCreator,
    EuclideanDistanceComputator,
    EmptyCollisionChecker,
    ObstacleCollisionChecker,
)
from obstacle import construct_circular_obstacles, WorldBoundary2D
from draw_cspace import draw

import json, sys, os, argparse, math
import matplotlib.pyplot as plt
from shapely.geometry import Polygon

ALG_RRT = "rrt"


#functions for extracting vertices,edges and path
def extract_vertices(graph):
    vertices = []
    for vid, config in graph.vertices.items():
        vertices.append({"id": vid, "config": list(config)})
    return vertices

def extract_edges(graph):
    edges = []
    for edge_key, _ in graph.edges.items():
        edges.append(list(edge_key))
    return edges

def extract_path(graph, root_vertex, goal_vertex):
    path = graph.get_vertex_path(root_vertex, goal_vertex)
    return path

    
def main_rrt(
    cspace, qI, qG, edge_creator, distance_computator, collision_checker, obs_boundaries
):
    fig, ax3 = plt.subplots(1, 1)

    
    # Task 1: RRT algorithm
    title3 = "RRT planning"
    (G3, root, goal) = rrt(
        cspace=cspace,
        qI=qI,
        qG=qG,
        edge_creator=edge_creator,
        distance_computator=distance_computator,
        collision_checker=collision_checker,
    )

    
    path = []
    if goal is not None:
        path = G3.get_path(root, goal)
        if (len(path)>0):
            vertices = extract_vertices(G3)
            edges = extract_edges(G3)
            pathList= extract_path(G3,root,goal)
            # print(vertices)
            # print(edges)
            # print(pathList)

            jsonDict = {"vertices": extract_vertices(G3),"edges":extract_edges(G3), "path":extract_path(G3,root,goal) }
            print("json dict",jsonDict)
            with open(args.out, "w") as outfile:
                json.dump(jsonDict, outfile)
            

            draw(ax3, cspace, obs_boundaries, qI, qG, G3, path, title3)
            plt.show()
        else:
            print("could not find path this time.Try again!")
            return    

   






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
    xG = tuple(data["xG"])
    U = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    return (O, W, L, D, xI, xG, U)



if __name__ == "__main__":

    args = parse_args()
    (O, W, L, D, xI, XG, U) = parse_desc(args.desc)
    #Cobs = compute_Cobs(O, W, L, D)
    cspace = [(-3, 3), (-3, 3)]
    qI = xI
    qG = XG
    obstacles = O
    #obs_boundaries = [obstacle.get_boundaries() for obstacle in obstacles]

    # We don't really need to explicitly need to check the world boundary
    # because the world is convex and we're connecting points by a straight line.
    # So as long as the two points are within the world, the line between them
    # are also within the world.
    # I'm just including this for completeness.
    #world_boundary = WorldBoundary2D(cspace[0], cspace[1])
    #obstacles.append(world_boundary)

    edge_creator = StraightEdgeCreator(0.1)
    collision_checker = ObstacleCollisionChecker(obstacles,W,L,D)
    distance_computator = EuclideanDistanceComputator()

    args = parse_args()

    main_rrt(
            cspace,
            qI,
            qG,
            edge_creator,
            distance_computator,
            collision_checker,
            obstacles,
        )
