Probabilistic Roadmap Planner (PRM) in 2D Environment
This repository contains an implementation of a Probabilistic Roadmap (PRM) path planner in a 2D environment. The PRM algorithm is widely used in robot motion planning for finding collision-free paths between a start and a goal point in a space with obstacles.

Features
Environment Setup: Generates a random 2D environment with obstacles.
PRM Generation: Constructs a roadmap by sampling random points in the environment and connecting them using a KDTree.
Pathfinding: Utilizes A* search algorithm to find the shortest path on the roadmap from the start to the goal point.
Path Shortcutting: Post-processes the found path to eliminate unnecessary detours, making the path more efficient.
Requirements
To run the code, you need to have the following Python packages installed:

numpy
pylab
networkx
scikit-learn
You can install the required packages using pip:

bash
Copy code
pip install numpy matplotlib networkx scikit-learn
How to Run
Clone the repository:

bash
Copy code
git clone https://github.com/yourusername/2d-prm-path-planner.git
Navigate to the project directory:

bash
Copy code
cd 2d-prm-path-planner
Ensure the environment_2d.py file is in the same directory as the main script. This file should contain the Environment class that manages the 2D space and obstacles.

Run the main script:

bash
Copy code
python main.py
The script will generate a random environment, create a PRM, and find a path from the start to the goal. The path will be plotted on the 2D environment, showing the initial path and the path after shortcutting.

Code Overview
Environment Setup:

env = Environment(...) initializes the 2D environment with random dimensions.
env.plot() visualizes the environment.
env.random_query() generates a random start and goal point for the path search.
PRM Construction:

Random nodes are generated and checked for collision.
A KDTree is used to efficiently find neighbors within a specified radius.
Nodes are connected in the roadmap (graph) if the path between them is collision-free.
Pathfinding:

The A* algorithm is used to find the shortest path from the start to the goal on the generated roadmap.
Path Shortcutting:

The found path is post-processed to remove unnecessary segments, producing a more efficient route.
Future Improvements
Environment Customization: Add functionality to manually set the environment's dimensions and obstacle layout.
Algorithm Tuning: Allow more flexibility in adjusting PRM parameters, such as node density and connection radius.
Optimization: Improve the efficiency of the collision detection and pathfinding steps.
Visualization: Enhance the visualization to show intermediate steps in the PRM generation and pathfinding process.

## Acknowledgments

- 'environment_2d' library used in this project is cloned from the [OS Robotics github repository](https://github.com/crigroup/osr_course_pkgs/blob/master/osr_examples/scripts/environment_2d.py)
