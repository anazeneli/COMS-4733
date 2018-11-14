class Node():
    def __init__(self, pt):
        self.root = pt
        self.left = None
        self.right = None

class Tree():
    def __init__(self, start, goal ):
        self.start = start
        self.goal = goal
        self.vertices = []
        self.edges = []
        self.nodes = {}

    def check_expansion(self, x_near):
        can_add = 0

        if self.nodes[x_near] :
            root = self.nodes[x_near]

            if not root.left:
                can_add = 1

            elif not root.right:
                can_add = 1

        return can_add

    def add_node(self, x1, x2) :
        root = self.nodes[x1]
        if not root.left:
            root.left = x2

        elif not root.right:
            root.right = x2

    def add_edge(self, x_near, x_new):
        self.add_node(x_near, x_new)
        self.edges.append((x_near, x_new))

    def add_vertex(self, x):
        self.vertices.append(x)
        if x not in self.nodes.keys():
            self.nodes[x] = Node(x)

    #
    # def add_edge(self, x_near, x_new):
    #     self.edges.append((x_near, x_new))
    #
    # def add_vertex(self, x):
    #     if x not in self.vertices:
    #         self.vertices.append(x)
