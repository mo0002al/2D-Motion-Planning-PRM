# examples/prm_example.py

import sys
import os

# Add the 'src' directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

import numpy as np
import pylab as pl
from src.env.environment_2d import Environment
from src.prm import PRM

# Set up environment
np.random.seed(4)
env = Environment(10, 6, 5)
pl.figure(figsize=(12,8))
pl.clf()
env.plot()

# Generate random query (start and goal)
query = env.random_query()
if query is not None:
    x_start, y_start, x_goal, y_goal = query
    env.plot_query(x_start, y_start, x_goal, y_goal)

    # Initialize PRM with environment and parameters
    prm = PRM(env)
    
    # Step 1: Sample nodes
    nodes = prm.sample_nodes()
    
    # Step 2: Build roadmap
    graph, tree = prm.build_roadmap(nodes)
    
    # Step 3: Add start and goal to the graph
    prm.add_start_goal(graph, tree, (x_start, y_start), (x_goal, y_goal))
    
    # Step 4: Find the path using A*
    path = prm.find_path(graph, (x_start, y_start), (x_goal, y_goal))
    
    # Step 5: Apply path shortcutting
    if path:
        path = prm.shortcut_path(path)
        path_x, path_y = zip(*path)
        pl.plot(path_x, path_y, 'g', linewidth=2)

pl.show()
