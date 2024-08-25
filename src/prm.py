# src/prm.py

import numpy as np
import networkx as nx
from sklearn.neighbors import KDTree

class PRM:
    def __init__(self, env, num_samples=500, connection_radius=3, step_size=0.05, safe_distance=0.01, max_rep=10):
        self.env = env
        self.num_samples = num_samples
        self.connection_radius = connection_radius
        self.step_size = step_size
        self.safe_distance = safe_distance
        self.max_rep = max_rep

    def sample_nodes(self):
        nodes = []
        while len(nodes) < self.num_samples:
            x = np.random.rand() * self.env.size_x
            y = np.random.rand() * self.env.size_y
            if not self.env.check_collision(x, y):
                nodes.append((x, y))
        return nodes

    def build_roadmap(self, nodes):
        tree = KDTree(np.array(nodes))
        graph = nx.Graph()
        graph.add_nodes_from(nodes)

        for i, node1 in enumerate(nodes):
            for j, node2 in enumerate(nodes):
                if i != j and np.linalg.norm(np.array(node1) - np.array(node2)) <= self.connection_radius:
                    if self.is_path_collision_free(node1, node2):
                        graph.add_edge(node1, node2)
        return graph, tree

    def is_path_collision_free(self, node1, node2):
        x1, y1 = node1
        x2, y2 = node2
        for ob in self.env.obs:
            if self.is_line_segment_intersecting(x1, y1, x2, y2, ob.x0, ob.y0, ob.x1, ob.y1) or \
               self.is_line_segment_intersecting(x1, y1, x2, y2, ob.x1, ob.y1, ob.x2, ob.y2) or \
               self.is_line_segment_intersecting(x1, y1, x2, y2, ob.x2, ob.y2, ob.x0, ob.y0):
                return False
        if self.is_point_near_boundary(x1, y1) or self.is_point_near_boundary(x2, y2):
            return False
        return True

    def is_line_segment_intersecting(self, x1, y1, x2, y2, x3, y3, x4, y4):
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        A = (x1, y1)
        B = (x2, y2)
        C = (x3, y3)
        D = (x4, y4)
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

    def distance_to_nearest_obstacle(self, x, y):
        return min([np.linalg.norm(np.array([x, y]) - np.array([ob.x0, ob.y0])) for ob in self.env.obs])

    def add_start_goal(self, graph, tree, start, goal):
        graph.add_node(start)
        graph.add_node(goal)
        start_neighbors = tree.query_radius(np.array([start]), r=self.connection_radius)[0]
        goal_neighbors = tree.query_radius(np.array([goal]), r=self.connection_radius)[0]

        for neighbor in start_neighbors:
            node = self.nodes[neighbor]
            if self.is_path_collision_free(start, node):
                graph.add_edge(start, node)

        for neighbor in goal_neighbors:
            node = self.nodes[neighbor]
            if self.is_path_collision_free(goal, node):
                graph.add_edge(goal, node)

    def find_path(self, graph, start, goal):
        try:
            return nx.astar_path(graph, start, goal, heuristic=self.heuristic)
        except nx.NetworkXNoPath:
            print("No path found between start and goal!")
            return []

    def shortcut_path(self, path):
        for _ in range(self.max_rep):
            if len(path) < 3:
                continue
            i, j = np.random.choice(len(path), 2, replace=False)
            if i > j:
                i, j = j, i
            if j - i < 2:
                continue
            new_path = path[:i+1] + [path[j]] + path[j+1:]
            if self.is_segment_collision_free(path[i][0], path[i][1], path[j][0], path[j][1]):
                path = new_path
        return path

    def is_segment_collision_free(self, x1, y1, x2, y2):
        for i in range(0, 100, 5):
            x = x1 + i / 100.0 * (x2 - x1)
            y = y1 + i / 100.0 * (y2 - y1)
            if self.env.check_collision(x, y):
                return False
        return True

