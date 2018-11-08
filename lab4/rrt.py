# Program to load obstacle course for Lab 4 - RRT

# usage:  python rrt.py obstacles_file start_goal_file


from __future__ import division
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

# helper function to draw RRT search path
def draw_rand(start, rand):
    x = [start[0], rand[0]]
    y = [start[1], rand[1]]

    for i in range(0, len(x), 2):
        plt.plot(x[i:i+2], y[i:i+2])

    ax.add_patch(patches.Circle(rand, facecolor='xkcd:red'))
    plt.show()



# funciton that generates the ranomd configuration

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
    ax.add_patch(patches.Circle([new_x, new_y], facecolor='xkcd:violet'))
    plt.plot([x, new_x], [y, new_y])

    return [new_x, new_y]

    # print pow(new_x,2) + pow(new_y,2)
    # assert 1==2


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


    st = start
    for x in range(20):    
        st = get_rand(st);

    plt.show()

    # print "i will not be pritned"
    # plt.show()
