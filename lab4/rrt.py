# Program to load obstacle course for Lab 4 - RRT
# usage:  python rrt.py obstacles_file start_goal_file
from __future__ import division
from line import Line
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import random, math

# global variables
step_size = 50.0

# grid bounds
xtop = 600
xbottom = 0

ytop = 600
ybottom = 0

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

    # print new_x, new_y
    # newLine = Line((x,y),(new_x, new_y))
    # ax.add_patch(patches.Circle([new_x, new_y], facecolor='xkcd:violet'))
    # plt.plot([x, new_x], [y, new_y])

    return [new_x, new_y]

    # print pow(new_x,2) + pow(new_y,2)
    # assert 1==2f

# function that reads the obstacle file and returns line of obstacles
def readObs(filename):
    # opening the file
    obsFile = open(filename, 'r')

    # dictionaray of obs in the file
    obstacles = []

    # first line of the file is the number of objects
    numObs = int(obsFile.readline());

    print "The number of obstacles is: ", numObs
    # iterating over all the obstaces
    for x in range(numObs):
        # firstline of each obstacle is the number of edges
        numEdges = int(obsFile.readline());
        print "the number of edges of:", x, "is", numEdges
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
    # assert 1 == 12

    st = start
    for x in range(200):
        end = get_rand(st);
        theLine = Line((st[0], st[1]),(end[0],end[1]))

        draw = 0
        for l in obsLine:
            if (theLine.intersect(l))&(l.intersect(theLine)):
                draw = 1
        if draw == 0:
            ax.add_patch(patches.Circle([end[0], end[1]], facecolor='xkcd:violet'))
            plt.plot([st[0], end[0]], [st[1], end[1]])
            st = end

    plt.show()
