class Tree():
    def __init__(self, start, goal ):
        self.start = start
        self.goal  = goal
        self.vertices = []
        self.edges = []

        # self.edges = {}

    def add_edge(self, x_near, x_new):
        self.edges.append((x_near, x_new))
        # if x_near in self.edges.keys():
        #     self.edges[x_near].append(x_new)
        # else:
        #
        # self.vertices.append(x_new)

    def add_vertex(self, x):
        if x not in self.vertices:
            self.vertices.append(x)
