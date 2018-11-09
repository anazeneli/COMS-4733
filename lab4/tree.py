class Tree():
    def __init__(self):
        self.vertices = []
        self.edges = []

    def add_edge(self, x_near, x_new):
        self.edges.append((x_near, x_new))

    def add_vertex(self, x):
        self.vertices.append(x)
