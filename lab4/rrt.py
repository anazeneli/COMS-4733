# Program to load obstacle course for Lab 4 - RRT
# usage:  python rrt.py obstacles_file start_goal_file
from __future__ import division
from line import Line
from tree import Tree
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import random, math
from math import cos, sin, atan
from itertools import cycle

# global variables
step_size = 75
# grid bounds
xtop = 600
xbottom = 0
ytop = 600
ybottom = 0

global start, goal

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

# boolean check if point is collision-free
def collision_free(st,end):
    theLine = Line((st[0], st[1]),(end[0],end[1]))

    for l in obsLine:
        if (theLine.intersect(l))&(l.intersect(theLine)):
            return False

    # remove exploration nodes from path
    if within_grid_bounds(end):
        obsLine.append(theLine)
        return True

def within_grid_bounds(pt):
    # grid bounds
    xtop    = 600
    xbottom = 0
    ytop    = 600
    ybottom = 0

    x, y = pt
    if ((x <= xtop) and (y <= ytop)) and ((x >= xbottom) and(y >= ybottom)):
        return True

    return False

def draw(st, end, color=None):
    ax.add_patch(patches.Circle([end[0], end[1]], facecolor='xkcd:violet'))
    plt.plot([st[0], end[0]], [st[1], end[1]])

    if color:
        plt.plot([st[0], end[0]], [st[1], end[1]], color)

    plt.pause(0.1)

# function that generates the ranomd configuration
def get_rand(range = None):
    if range:
        inc_x = random.randint(goal[0] - range, goal[0] + range)
        inc_y = random.randint(goal[0] - range, goal[0] + range)

    else:
        inc_x = random.randint(1,600)
        inc_y = random.randint(1,600)

    new_x = -1
    new_y = -1

    # add conditions so the new config is always inside the grid
    while ((new_x > xtop)|(new_y > ytop))|((new_x < xbottom)|(new_y < ybottom)):
        # angle = random.uniform(0, 2*math.pi)
        new_x = inc_x #*math.cos(angle)
        new_y = inc_y #*math.sin(angle)

    return round(new_x,2), round(new_y,2)

def distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    dx = x2 - x1
    dy = y2 - y1

    return math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))

# TODO: implement k-d tree
# find closest neighbor of q in T
def nearest_neighbor(q, T):
    best_pt   = q
    best_dist = xtop * 2

    for v in T.vertices:
        curr_dist = distance(q, v)
        if curr_dist < best_dist:
            best_pt   = v
            best_dist = curr_dist

    return best_pt

# progress by step_size along straight line between
# q_near and q_rand (from q1 through q2 a distance
# set by step size)
def new_state(q1, q2):
    eps = step_size/float(10)
    # protect against undefined slope
    if q2[0] - q1[0] == 0 :
        new_x = q1[0]
        new_y = q2[1] + step_size
    # protect against slope of zero
    elif q2[1] - q1[1] ==0 :
        new_x = q1[0] + step_size
        new_y = q2[1]
    else:
        m = q2[1] - q1[1]/ float(q2[0] - q1[0])
        b = -m*q2[1] + q2[0]
        angle = atan((q2[1]- q1[1])/ float((q2[0]-q1[0])))

        new_x = q1[0] + step_size * cos(angle)
        new_y = q1[1] + step_size * sin(angle)

    new_x = round(new_x, 2)
    new_y = round(new_y, 2)
    new = (new_x, new_y)

    if collision_free(q1, new):
        return new

def dijsktra(T, start, goal):
    dists = T.dists
    shortest_paths = {start: (None, 0)}
    v = start
    visited = set()

    while v != goal:
        visited.add(v)
        connected_nodes = T.edges[v]

        for node in connected_nodes:
            weight = dists[v, node] + shortest_paths[v][1]
            if node not in shortest_paths:
                shortest_paths[node] = (v, weight)
            else:
                curr_shortest_weight = shortest_paths[node][1]
                if curr_shortest_weight > weight:
                    shortest_paths[node] = (v, weight)

        next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}

        if not next_destinations:
            return "Route Not Possible"

        v = min(next_destinations, key=lambda k: next_destinations[k][1])


    # Work back through destinations in shortest path
    path = []
    while v is not None:
        path.append(v)
        next_node = shortest_paths[v][0]
        v = next_node
    # Reverse path
    path = path[::-1]
    return path

def combine_trees(t1, t2):
    for i in t2.edges:
        for j in t2.edges[i]:
            t1.add_edge(i,j)
            t1.fill_distances(i,j, t2.dists[(i,j)])

def bidirectional_rrt(ts, tg, n = 5000):
    alternate = cycle(range(2))

    # while(distance(q, goal) > step_size):
    # bias 5% of time toward goal
    bias_factor = n * 0.05
    divisor = n / bias_factor

    q_new = ts.start

    for k in range(n):
        # Alternate between trees to grow from
        toggle = alternate.next()
        if  toggle == 0:
            T = ts
            start =  ts.start
            goal  =  ts.goal
            color = 'xkcd:orange'
        else:
            T = tg
            start =  tg.start
            goal  =  tg.goal
            color = 'xkcd:blue'

        # add bias to goal
        if k % divisor == 0:
            q_rand = goal

        near1 =  nearest_neighbor(q_new, ts)
        near2 =  nearest_neighbor(q_new, tg)

        if distance(near1, near2) < step_size:
            if collision_free(near1, near2):
                T.add_edge(near1, near2)
                T.fill_distances(near1, near2, distance(near1, near2))
                draw(near1, near2, color)
                print "GOAL REACHED"

                if toggle == 0:
                    combine_trees(T,tg)
                else:
                    combine_trees(T,ts)

                break
            else:
                q_rand = get_rand()
        elif distance(near1, near2) < 3 * step_size:
            q_rand = get_rand(2 * step_size)
            # print "NEAR GOAL "
        else:
            q_rand = get_rand()

        ex = extend(T, q_rand, color)
        if ex:
            q_new = ex

    return ts

def build_rrt(T, q, goal, n=5000):
    print "STARTING AT ", q
    # while(distance(q, goal) > step_size):
    # bias 5% of time toward goal
    bias_factor = n * 0.05
    divisor = n / bias_factor
    q_new = q
    for k in range(n):
        # add bias to goal
        if k % divisor == 0:
            q_rand = goal

        near =  nearest_neighbor(q_new, T)
        if distance(near, goal) < step_size:
            if collision_free(near, goal):
                T.add_edge(near, goal)
                T.fill_distances(near, goal, distance(near, goal))
                draw(near, goal)
                # print "GOAL REACHED"
                break
            else:
                q_rand = get_rand()

        elif distance(near, goal) < 3 * step_size:
            q_rand = get_rand(2 * step_size)
            # print "NEAR GOAL "

        else:
            q_rand = get_rand()

        ex = extend(T, q_rand)
        if ex:
            q_new = ex

    return T

# extends tree in direction of passed q
# from nearest vertex in rrt
def extend(T, q, color):
    q_new = None
    # near_list = nearest_neighbor(q, T)
    q_near = nearest_neighbor(q, T)

    # if q_rand is within a step size, connect it to nearest node
    if distance(q_near, q) < step_size:
        if collision_free(q_near, q):
            T.add_edge(q_near, q)
            T.fill_distances(q_near, q, distance(q_near, q))
            draw(q_near, q, color)

            return q
    else:
        q_new = new_state(q_near, q)
        if q_new :
            if collision_free(q_near, q_new):
                T.add_edge(q_near, q_new)
                T.fill_distances(q_near, q_new, distance(q_near, q_new))

                draw(q_near, q_new, color)

                return q_new

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path',
                        help="File path for obstacle set")
    parser.add_argument('start_goal_path',
                        help="File path for obstacle set")

    parser.add_argument('-b', action='store_true',
                        help="Optional flag for bidirectional RRT (-b)")

    args = parser.parse_args()

    fig, ax = plt.subplots()
    path = build_obstacle_course(args.obstacle_path, ax)
    start, goal = add_start_and_goal(args.start_goal_path, ax)

    # collision detection helper function
    obsLine = readObs("world_obstacles.txt")

    if args.b:
        T1 = Tree(start, goal)
        T2 = Tree(goal, start)

        T = bidirectional_rrt(T1, T2)

    else:
        build_rrt(T, T.start, T.goal, 20000)


    shortest_path = dijsktra(T,start, goal)

    for i in range(len(shortest_path) -1):
        draw(shortest_path[i], shortest_path[i+1] ,'xkcd:yellow')

    plt.show()
