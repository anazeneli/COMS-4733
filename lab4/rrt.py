# Program to load obstacle course for Lab 4 - RRT
# usage:  python rrt.py obstacles_file start_goal_file
from __future__ import division
from line import Line
from tree import Tree
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import random, math
from math import cos, sin, atan

# global variables
step_size = 50.0
# grid bounds
xtop = 600
xbottom = 0
ytop = 600
ybottom = 0

# function that reads the obstacle file and returns line of obstacles
def readObs(filename):
    # opening the file
    obsFile = open(filename, 'r')

    # dictionaray of obs in the file
    obstacles = []

    # first line of the file is the number of objects
    numObs = int(obsFile.readline());

    # print "The number of obstacles is: ", numObs
    # iterating over all the obstaces
    for x in range(numObs):
        # firstline of each obstacle is the number of edges
        numEdges = int(obsFile.readline());
        # print "the number of edges of:", x, "is", numEdges
        # iterating ove all the edges
        corners = []
        for y in range(numEdges):
            c = obsFile.readline();
            # removing whitespace and storing the resulting words in array
            c = c.split();
            corners.append((float(c[0]), float(c[1])))

        for x in range(len(corners)):
            if x == len(corners)-1:
                theLine = Line((corners[len(corners)-1][0], corners[len(corners)-1][1]), (corners[0][0], corners[0][1]))
                obstacles.append(theLine)
            else:
                theLine = Line((corners[x][0], corners[x][1]), (corners[x+1][0], corners[x+1][1]))
                obstacles.append(theLine)

    return obstacles

def build_obstacle_course(obstacle_path, ax):
    vertices = list()
    codes = [Path.MOVETO]
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        lines = 0
        for line in f:
            coordinates = tuple(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                codes += [Path.MOVETO] + [Path.LINETO]*(coordinates[0]-1) + [Path.CLOSEPOLY]
                vertices.append((0,0)) #Always ignored by closepoly command
            else:
                vertices.append(coordinates)
    vertices.append((0,0))
    vertices = np.array(vertices, float)
    path = Path(vertices, codes)
    pathpatch = patches.PathPatch(path, facecolor='None', edgecolor='xkcd:violet')

    ax.add_patch(pathpatch)
    ax.set_title('Rapidly-exploring Random Tree')

    ax.dataLim.update_from_data_xy(vertices)
    ax.autoscale_view()
    ax.invert_yaxis()

    return path

def add_start_and_goal(start_goal_path, ax):
    start, goal = None, None
    with open(start_goal_path) as f:
        start = tuple(map(int, f.readline().strip().split(' ')))
        goal  = tuple(map(int, f.readline().strip().split(' ')))

    ax.add_patch(patches.Circle(start, facecolor='xkcd:bright green'))
    ax.add_patch(patches.Circle(goal, facecolor='xkcd:fuchsia'))

    return start, goal

# function that generates the ranomd configuration
def get_rand(pos):
    x = pos[0]
    y = pos[1]

    new_x = -1
    new_y = -1

    # add conditions so the new config is always inside the grid
    while ((new_x > xtop)|(new_y > ytop))|((new_x < xbottom)|(new_y < ybottom)):
        angle = random.uniform(0, 2*math.pi)
        new_x = x + step_size*math.cos(angle)
        new_y = y + step_size*math.sin(angle)

    return [new_x, new_y]

def build_rrt(q, n):
    T = Tree()
    T.add_vertex(q)

    for k in range(n):
        q_rand = random_state(q)
        extend(T, q_rand)

    return T

# TODO: write a function to progress the tree
# from the initial q point to a random point
# in free space
# selects nearest vertex in rrt to given point
def extend(T, q):
    q_near = nearest_neighbor(q, T)
    q_new = new_state(q, q_near)

    if q_new:
        T.add_vertex(q_new)
        T.add_edge(q_near, q_new)

        if q_new == q:
            return 'Reached'
        else:
            return 'Advanced'

    return 'Trapped'

def random_state(st):
    end = get_rand(st);
    theLine = Line((st[0], st[1]),(end[0],end[1]))

    draw = 0
    for l in obsLine:
        if (theLine.intersect(l))&(l.intersect(theLine)):
            draw = 1
            break
    if draw == 0:
        ax.add_patch(patches.Circle([end[0], end[1]], facecolor='xkcd:violet'))
        plt.plot([st[0], end[0]], [st[1], end[1]])
        # remove exploration nodes from path
        obsLine.append(theLine)

    return end

# TODO: implement k-d tree
# find closest neighbor of q in T
def nearest_neighbor(q, T):

    return q

# progress by step_size along straight line between
# q_near and q_rand (from q1 through q2 a distance set by step size)
def new_state(q1, q2):
    # protect against undefined slope
    if (q2[0] - q1[0]) == 0:
        new_y = q2[1] + step_size

        return q2[0], new_y
    # protect against slope of zero
    elif q2[1] - q1[1] == 0:
        new_x = q2[0] + step_size

        return new_x, q2[1]
    else:
        m = float(q2[1] - q1[1])/ (q2[0] - q1[0])
        b = -m*q2[1] + q2[0]
        angle = atan(q2[1]/float(q2[0]))

        x = step_size * cos(angle)
        y = step_size * sin(angle)

        return (x,y)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path',
                        help="File path for obstacle set")
    parser.add_argument('start_goal_path',
                        help="File path for obstacle set")
    args = parser.parse_args()

    fig, ax = plt.subplots()
    path = build_obstacle_course(args.obstacle_path, ax)
    start, goal = add_start_and_goal(args.start_goal_path, ax)

    obsLine = readObs("world_obstacles.txt")

    build_rrt(start, 200)
    #
    # st = start
    # for x in range(10000):
    #     end = get_rand(st);
    #     theLine = Line((st[0], st[1]),(end[0],end[1]))
    #
    #     draw = 0
    #     for l in obsLine:
    #         if (theLine.intersect(l))&(l.intersect(theLine)):
    #             draw = 1
    #             break
    #     if draw == 0:
    #         ax.add_patch(patches.Circle([end[0], end[1]], facecolor='xkcd:violet'))
    #         plt.plot([st[0], end[0]], [st[1], end[1]])
    #         st = end
    #         # remove exploration nodes from path
    #         obsLine.append(theLine)

    plt.show()
