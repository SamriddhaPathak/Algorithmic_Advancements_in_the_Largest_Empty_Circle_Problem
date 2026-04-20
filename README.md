# DO-LEC: Dynamic Obstacle-Aware Largest Empty Circle

Fast and Adaptive Largest Empty Circle Computation for Dynamic Environments

---

## Overview

DO-LEC (Dynamic Obstacle-Aware Largest Empty Circle) is a computational geometry algorithm designed to compute the Largest Empty Circle (LEC) in environments with moving obstacles efficiently and in real time.

Traditional LEC algorithms assume static environments and require complete recomputation whenever obstacles move. DO-LEC overcomes this limitation through an incremental and adaptive approach that preserves geometric optimality while enabling real-time performance.

This repository contains the implementation and experimental validation of the DO-LEC algorithm.

---

## Key Features

- Real-time computation of Largest Empty Circle in dynamic environments  
- Geometrically optimal solutions without approximation  
- Incremental updates instead of full recomputation  
- Scalable to large problem sizes (up to n = 1000, m = 400)  
- Combines Voronoi diagrams and Delaunay triangulation  
- k-nearest neighbor pruning for efficiency  
- Early termination strategies for faster evaluation  

---

## Problem Definition

Given:
- A bounded 2D workspace  
- A set of static points (sites)  
- A set of dynamic circular obstacles  

The objective is to compute, at any time t, the largest circle that:

1. Lies entirely within the workspace  
2. Contains no site in its interior  
3. Does not intersect any dynamic obstacle  

---

## Algorithm Overview

The DO-LEC algorithm operates in three main phases:

### 1. Candidate Generation

- Delaunay triangulation to obtain circumcenters (Voronoi vertices)  
- Boundary sampling (corners and edge midpoints)  
- Perpendicular bisector intersections  
- k-nearest neighbor pruning (k approximately 8)  

### 2. Dynamic Evaluation

For each candidate point:
- Compute distance to nearest site  
- Compute clearance from nearest obstacle  
- Compute distance to workspace boundary  
- Radius is the minimum of the above  
- Early termination is applied when possible  

### 3. Selection

- The candidate with the maximum feasible radius is selected as the solution  

---

## Time Complexity

The algorithm achieves:

O(n log n + n k + m)

Where:
- n is the number of sites  
- k is a small constant (approximately 8)  
- m is the number of dynamic obstacles  

---

## Performance Summary

| Scale       | Sites (n) | Obstacles (m) | Runtime        |
|------------|----------|--------------|----------------|
| Small      | 10       | 4            | ~1.5 ms        |
| Medium     | 100      | 10           | ~14 ms         |
| Large      | 500      | 150          | ~102 ms        |
| Very Large | 1000     | 400          | ~290 ms        |

- Sub-15 ms performance for real-time scenarios (n ≤ 100)  
- Predictable overhead for dynamic updates  
- Memory complexity O(n + m)  

---

## Implementation Details

- Language: Python 3.x  
- Libraries:
  - NumPy  
  - SciPy  
  - Matplotlib  
  - Tkinter  

### Core Components

- compute_lec(sites, obstacles)  
  Computes the largest empty circle  

- get_current_obstacle_positions(time)  
  Updates obstacle positions via interpolation  

- draw()  
  Handles rendering and visualization  

- update_stats()  
  Displays real-time statistics  

---

## GUI Features

The project includes an interactive graphical interface with:

- Site and obstacle creation tools  
- Obstacle motion path configuration  
- Visualization toggles:
  - Voronoi diagram  
  - Convex hull  
  - Largest empty circle  
- Real-time animation and statistics  

## Getting Started

### Clone the repository


git clone https://github.com/SamriddhaPathak/Algorithmic_Solutions_to_the_LEC_Problem

cd Algorithmic_Solutions_to_the_LEC_Problem


### Install dependencies


pip install numpy scipy matplotlib


### Run the application


python main.py


---

## Experimental Setup

- Dataset types:
  - Uniform distribution  
  - Clustered distribution  
  - Grid-perturbed distribution  

- Obstacle configuration:
  - Piecewise linear motion paths  
  - Constant speed  
  - Ping-pong interpolation  

---

## Contributions

### Dynamic Largest Empty Circle Algorithm

- Efficient geometric candidate generation  
- Adaptive triangulation strategy  
- Early termination optimization  

### Dynamic Obstacle Integration

- Unified constraint-based formulation  
- Real-time obstacle position tracking  
- Seamless extension of static LEC to dynamic environments  

---

## Limitations

- Restricted to 2D environments  
- Supports only circular obstacles  
- Assumes perfect obstacle localization  
- Python implementation introduces performance overhead  

---

## Future Work

- Extension to 3D (largest empty sphere)  
- GPU acceleration for parallel computation  
- Integration with ROS2 and simulation frameworks  
- Support for non-circular obstacle shapes  
- Learning-based optimization techniques  

---

## Applications

- Autonomous robotics  
- Drone navigation  
- Warehouse automation  
- Emergency response planning  
- Spatial optimization systems  

---

## Acknowledgements

The authors acknowledge the guidance and technical support of Abiral Sangroula, Principal Engineer at QSystems AI.

---

## Citation

If you use this work, please cite:

### APA Style

Pathak, S., & Pokhrel, J. (2025). DO-LEC (Dynamic Obstacle-Aware Largest Empty Circle): Fast and adaptive LEC computation for navigating dynamic obstacle fields. International Journal on Engineering Technology, 3(1), 62–80.

### IEEE Style

S. Pathak and J. Pokhrel, "DO-LEC (Dynamic Obstacle-Aware Largest Empty Circle): Fast and Adaptive LEC Computation for Navigating Dynamic Obstacle Fields," International Journal on Engineering Technology, vol. 3, no. 1, pp. 62–80, Dec. 2025.

### BibTeX


@article{pathak2025dolec,
author = {Pathak, S. and Pokhrel, J.},
title = {DO-LEC (Dynamic Obstacle-Aware Largest Empty Circle): Fast and Adaptive LEC Computation for Navigating Dynamic Obstacle Fields},
journal = {International Journal on Engineering Technology},
volume = {3},
number = {1},
pages = {62--80},
month = dec,
year = {2025}
}


---

## Contact

Samriddha Pathak  
Email: samriddha.805421@cct.tu.edu.np
