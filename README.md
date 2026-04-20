# DO-LEC: Dynamic Obstacle-Aware Largest Empty Circle

> **Fast and Adaptive LEC Computation for Navigating Dynamic Obstacle Fields**

[![DOI](https://img.shields.io/badge/DOI-10.3126%2Finjet.v3i1.86978-blue)](https://doi.org/10.3126/injet.v3i1.86978)
[![Journal](https://img.shields.io/badge/Journal-InJET%20Vol.3%20No.1-green)](https://www.kec.edu.np/kec-journal/)
[![Python](https://img.shields.io/badge/Python-3.x-yellow)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Academic%20Use-lightgrey)](#license)

---

**Published in:** International Journal on Engineering Technology (InJET), Vol. 3, No. 1, Dec 2025, pp. 62–80  
**DOI:** [10.3126/injet.v3i1.86978](https://doi.org/10.3126/injet.v3i1.86978)  
**Received:** July 27, 2025 &nbsp;|&nbsp; **Accepted:** October 15, 2025

| Author | Affiliation | Contact |
| :--- | :--- | :--- |
| Samriddha Pathak *(corresponding)* | Central Campus of Technology, Dharan, Sunsari, Nepal | samriddha.805421@cct.tu.edu.np |
| Jyoti Pokhrel | Soch College of IT, Pokhara, Kaski, Nepal | jyoti2079@sochcollege.edu.np |

---

## Table of Contents

- [Abstract](#abstract)
- [Keywords](#keywords)
- [Problem Definition](#problem-definition)
- [Algorithm Overview](#algorithm-overview)
- [Complexity Analysis](#complexity-analysis)
- [Getting Started](#getting-started)
- [Implementation Details](#implementation-details)
- [Experimental Results](#experimental-results)
- [Optimization Strategies](#optimization-strategies)
- [Limitations and Future Work](#limitations-and-future-work)
- [Applications](#applications)
- [Acknowledgements](#acknowledgements)
- [Citation](#citation)
- [License](#license)

---

## Abstract

Autonomous systems navigating dynamic environments require continuous identification of maximum-clearance safe zones. Existing Largest Empty Circle (LEC) algorithms assume static obstacle configurations and necessitate complete recomputation when obstacles move — a fundamental limitation that prevents real-time deployment.

**DO-LEC** is the first practical algorithm to compute geometrically optimal safe zones in environments with moving obstacles while maintaining real-time performance. It integrates classical Voronoi-Delaunay geometric principles with adaptive candidate generation and incremental obstacle tracking, achieving efficient updates without full reconstruction. Comprehensive evaluation across diverse problem scales demonstrates sub-second computation for large-scale scenarios with predictable overhead characteristics essential for safety-critical systems. Unlike approximation-based methods that sacrifice optimality for speed, DO-LEC preserves geometric correctness while achieving practical real-time capability.

---

## Keywords

Largest Empty Circle &nbsp;·&nbsp; Computational Geometry &nbsp;·&nbsp; Dynamic Obstacle Avoidance &nbsp;·&nbsp; Voronoi Diagrams &nbsp;·&nbsp; Real-Time Path Planning &nbsp;·&nbsp; Delaunay Triangulation &nbsp;·&nbsp; Autonomous Navigation

---

## Problem Definition

Given a 2D bounded rectangular workspace **B ⊂ ℝ²** containing:

- A set of **static point sites** S = {s₁, s₂, ..., sₙ} ⊂ ℝ²
- A set of **dynamic circular obstacles** O = {o₁, ..., oₘ}, each with fixed radius rⱼ > 0 and a time-varying center Oⱼ(t) moving along a piecewise-linear path at constant speed

The goal is to compute, at any time *t*, the **Largest Empty Circle (LEC)** that:

1. Lies entirely within the workspace B
2. Contains no site in its interior
3. Does not intersect any dynamic obstacle

### Mathematical Formulation

**Feasible Region.** The set of valid circle centers at time *t*:

```
F(t) = { x ∈ B : for all j,  |x − Oj(t)| >= rj }
```

**Distance Functions.**

```
dist(x, S)    =  min  |x − si|                          (nearest site distance)
                  i

dist(x, O(t)) =  min  max(0, |x − Oj(t)| − rj)         (obstacle clearance)
                  j

dist(x, dB)   =  distance from x to the rectangular boundary
```

**Optimization Objective.** Find the center `c*(t)` that maximizes the minimum clearance:

```
c*(t) = argmax   min{ dist(x, S),  dist(x, O(t)),  dist(x, dB) }
         x in F(t)
```

**Optimal Radius.** The radius of the largest feasible empty circle:

```
R*(t) = min{ dist(c*(t), S),  dist(c*(t), O(t)),  dist(c*(t), dB) }
```

By construction, the circle of radius R\*(t) centered at c\*(t) lies entirely within B, contains no site in its interior, and does not intersect any obstacle.

---

## Algorithm Overview

DO-LEC operates in three sequential phases:

### Phase 1 — Candidate Generation

Candidate circle centers are drawn from geometrically privileged locations where the optimal LEC center is guaranteed to lie:

- **Delaunay circumcenters** — circumcenters of Delaunay triangles correspond directly to Voronoi vertices. Full triangulation is used for n ≤ 50; a √n sampled triangulation (every k-th point, k ≈ √n) is used for n > 50 to avoid O(n²) cost.
- **Boundary samples** — the four rectangle corners and four edge midpoints, since the optimal circle may lie flush against a workspace wall.
- **Perpendicular bisector intersections** — for each site, its k = min(8, n−1) nearest neighbors define bisectors whose intersections with the workspace boundary are computed analytically and added to the candidate pool.

Duplicate candidates are removed by hashing to a rounded grid (10⁻⁶ precision). Out-of-bounds points are discarded.

### Phase 2 — Dynamic Evaluation

Obstacle positions are updated for the current animation frame via ping-pong path interpolation in O(m) time.

For each candidate point `p = (x, y)`, the feasible radius is:

```
r(p) = min( d_bound,   min |p − si|,   min (|p − Oj| − Rj) )
                        i               j
```

where the boundary clearance is computed in O(1) as:

```
d_bound = min{ x − x_min,  x_max − x,  y − y_min,  y_max − y }
```

**Early termination** is applied whenever:
- A partial site distance drops below ~1.0 unit (radius already negligibly small)
- Any obstacle clearance is non-positive (circle would intersect an obstacle)
- The current candidate radius falls below the best radius found so far

### Phase 3 — Fallback and Selection

The candidate with the maximum feasible radius is selected as the LEC. If the best radius falls below a threshold δ, a fallback check at the workspace center is performed. The algorithm returns the result as `(center, radius)`.

### System Pipeline

```
┌───────────────────────────────────────────┐
│        PHASE 1: CANDIDATE GENERATION      │
│   • Boundary corners and edge midpoints   │
│   • Adaptive Delaunay triangulation       │
│   • Perpendicular bisector intersections  │
└─────────────────────┬─────────────────────┘
                      │
┌─────────────────────▼─────────────────────┐
│      PHASE 2: OPTIMAL CIRCLE SELECTION    │
│   • Update obstacle positions — O(m)      │
│   • Evaluate each candidate               │
│   • Track best (center, radius)           │
│   • Early termination on invalid cases    │
└─────────────────────┬─────────────────────┘
                      │
┌─────────────────────▼─────────────────────┐
│         PHASE 3: FALLBACK & RETURN        │
│   • Verify solution quality               │
│   • Apply workspace-center fallback       │
│   • Return optimal circle                 │
└───────────────────────────────────────────┘
```

---

## Complexity Analysis

| Component | Time Complexity | Notes |
| :--- | :---: | :--- |
| Voronoi / Delaunay construction | O(n log n) | Fortune's sweepline algorithm |
| Convex hull computation | O(n log n) | Graham scan |
| LEC candidate evaluation | O(n log n + nk) | Delaunay + k-NN pruning, k ≈ 8 |
| Dynamic obstacle position updates | O(m) | Incremental per-frame interpolation |
| Rendering and visualization | O(n + m) | Linear drawing operations |
| **DO-LEC total** | **O(n log n + nk + m)** | k ≈ 8 is a small fixed constant |

**Memory complexity:** O(n + m) — measured from 45 KB (n=5) to 2.1 MB (n=1000, m=400).

---

## Getting Started

### Prerequisites

- Python 3.x
- [NumPy](https://numpy.org/)
- [SciPy](https://scipy.org/)
- [Matplotlib](https://matplotlib.org/)
- Tkinter *(included with most standard Python distributions)*

### Installation

```bash
# Clone the repository
git clone https://github.com/SamriddhaPathak/Algorithmic_Solutions_to_the_LEC_Problem
cd Algorithmic_Solutions_to_the_LEC_Problem

# Install dependencies
pip install numpy scipy matplotlib
```

### Running the Application

```bash
python main.py
```

---

## Implementation Details

The system is implemented in Python 3.x and tested on Windows 11 (Ryzen 5 5500U, 24 GB RAM). All state and logic are encapsulated in the `DynamicLECVoronoi` class. The GUI is built with Tkinter, consisting of a left-hand control panel and a right-hand Matplotlib canvas (800×600 workspace, 50-unit margin).

### Core Methods

| Method | Description |
| :--- | :--- |
| `compute_lec(sites, obstacles)` | Generates all candidates, evaluates feasible radii, and returns the optimal `(center, radius)` |
| `get_current_obstacle_positions(time)` | Linearly interpolates each obstacle along its waypoint path; reflects at endpoints (ping-pong) |
| `draw()` | Clears and redraws the full scene — Voronoi diagram, convex hull, LEC circle, sites, obstacles, and paths |
| `update_stats()` | Refreshes the live statistics panel: site count, obstacle count, elapsed time |

### GUI Features

| Panel | Controls |
| :--- | :--- |
| Mode | Add Sites / Add Obstacles |
| Animation | Play / Reset / Adjustable speed slider |
| Display toggles | Voronoi Diagram, Convex Hull, Largest Empty Circle |
| Statistics | Real-time site count, obstacle count, elapsed time |
| Obstacle management | Path configuration, individual and bulk clearing |

### Dataset Configuration

Static sites are generated using three spatial distributions:

| Distribution | Cases | Method |
| :--- | :---: | :--- |
| Uniform random | 1–4, 10–16 | `numpy.random.uniform()` within B = [50,750] × [50,550] |
| Clustered | 5–7 | Gaussian mixture, 3–5 cluster centers, σ = 50 px |
| Grid-perturbed | 8–9 | Regular √n × √n grid + Normal(μ=0, σ=15) noise |

All distributions use reproducible seeding: `seed = 42 + case_number`.

Dynamic obstacle parameters: radii ∈ [10, 25] px · speed = 1.0 unit/s · 3–8 waypoints per path · minimum inter-waypoint distance = 100 px · ping-pong interpolation · m ≈ 0.4n for n ≥ 100.

---

## Experimental Results

All timing values are the mean of 10 independent runs. Standard deviations were below 5% of the mean in all cases, confirming stable and reproducible performance.

### Test Case Categories

| Category | Cases | n Range | m Range | Distribution | Primary Purpose |
| :--- | :---: | :---: | :---: | :--- | :--- |
| Micro | 1–4 | 5–10 | 1–4 | Uniform | Baseline validation |
| Small | 5–7 | 100–200 | 10–30 | Clustered | Interactive robotics |
| Medium | 8–10 | 250–400 | 50–100 | Grid-perturbed | Simulation environments |
| Large | 11–13 | 500–700 | 150–250 | Uniform | Batch processing |
| Stress | 14–16 | 800–1000 | 300–400 | Mixed | Scalability limits |

### Full Runtime Performance (DO-LEC)

| Case | n | m | Time w/o LEC | Time w/ LEC |
| :---: | :---: | :---: | ---: | ---: |
| 1 | 5 | 1 | 1.0 ms | 1.8 ms |
| 2 | 7 | 2 | 0.8 ms | 1.5 ms |
| 3 | 8 | 3 | 0.7 ms | 1.4 ms |
| 4 | 10 | 4 | 0.7 ms | 1.5 ms |
| 5 | 100 | 10 | 1.5 ms | 14.0 ms |
| 6 | 150 | 20 | 2.1 ms | 22.8 ms |
| 7 | 200 | 30 | 2.8 ms | 32.4 ms |
| 8 | 250 | 50 | 3.6 ms | 42.8 ms |
| 9 | 300 | 75 | 4.5 ms | 54.2 ms |
| 10 | 400 | 100 | 6.2 ms | 76.8 ms |
| 11 | 500 | 150 | 8.1 ms | 102.3 ms |
| 12 | 600 | 200 | 10.2 ms | 130.7 ms |
| 13 | 700 | 250 | 12.6 ms | 163.2 ms |
| 14 | 800 | 300 | 15.3 ms | 200.1 ms |
| 15 | 900 | 350 | 18.4 ms | 242.6 ms |
| 16 | 1000 | 400 | 21.8 ms | 290.8 ms |

### DO-LEC vs. Static LEC Baseline

The Static LEC baseline applies Fortune's sweepline algorithm from scratch at every frame, with no temporal coherence between steps. Both algorithms share identical candidate generation and evaluation logic; the sole difference is incremental maintenance (DO-LEC) versus full reconstruction (Static LEC).

| n | m | Static LEC | DO-LEC | Overhead (ms) | Overhead (%) | Perf. Ratio |
| :---: | :---: | ---: | ---: | ---: | ---: | :---: |
| 5 | 1 | 1.3 ms | 1.8 ms | +0.5 | +38.5% | 0.72× |
| 10 | 4 | 1.1 ms | 1.5 ms | +0.4 | +36.4% | 0.73× |
| 100 | 10 | 9.8 ms | 14.0 ms | +4.2 | +42.9% | 0.70× |
| 200 | 30 | 21.1 ms | 32.4 ms | +11.3 | +53.6% | 0.65× |
| 400 | 100 | 45.2 ms | 76.8 ms | +31.6 | +69.9% | 0.59× |
| 500 | 150 | 58.4 ms | 102.3 ms | +43.9 | +75.2% | 0.57× |
| 700 | 250 | 88.7 ms | 163.2 ms | +74.5 | +84.0% | 0.54× |
| 1000 | 400 | 148.3 ms | 290.8 ms | +142.5 | +96.1% | 0.51× |

> The overhead percentage **stabilizes at 95–96% for n ≥ 500**, confirming that dynamic obstacle tracking imposes bounded and predictable cost. As problem size grows 100× (n=10 → n=1000), runtime grows 193×, closely tracking the theoretical O(n log n + nk + m) bound.

### Dynamic Overhead Breakdown

| Problem Size | n | m | Obstacle Updates | k-factor Cost | State Management | Total Overhead |
| :--- | :---: | :---: | ---: | ---: | ---: | ---: |
| Small | 10 | 4 | 0.1 ms | 0.2 ms | 0.1 ms | 0.4 ms |
| Medium | 100 | 10 | 0.3 ms | 2.8 ms | 1.1 ms | 4.2 ms |
| Large | 500 | 150 | 4.2 ms | 28.4 ms | 11.3 ms | 43.9 ms |
| Very Large | 1000 | 400 | 12.0 ms | 85.2 ms | 45.3 ms | 142.5 ms |

*k-factor Cost: computational cost of candidate generation through k-NN pruning, including distance computations and visibility queries.*

### Memory and Accuracy

| Metric | Result |
| :--- | :--- |
| Memory complexity | O(n + m) |
| Memory range | 45 KB (n=5) → 2.1 MB (n=1000, m=400) |
| LEC identification accuracy (n ≤ 15, verified vs. brute-force) | **100%** |
| Computed radius error bound | ≤ 10⁻¹² relative error |

### Performance at a Glance

| Scenario | n | Runtime | Practical Use |
| :--- | :---: | ---: | :--- |
| Micro-scale | 5 | 1.8 ms | Embedded / constrained systems |
| Interactive robotics | 100 | 14.0 ms | Real-time control (≥ 60 Hz) |
| Simulation | 500 | 102.3 ms | Physics simulation loops |
| Large-scale analysis | 1000 | 290.8 ms | Offline batch processing |

---

## Optimization Strategies

DO-LEC achieves O(n log n + nk + m) through four compounding strategies:

**1. √n Sampled Triangulation**
For n > 50, a representative subset (every k-th point, k ≈ √n) is triangulated instead of the full site set. This reduces triangulation overhead by **65–78%** while generating a candidate pool that preserves solution quality.

**2. Early Termination**
Candidate evaluation halts as soon as a geometric infeasibility is detected — obstacle intersection, boundary violation, or site proximity below 1.0 unit — or when the partial radius already falls below the current global best. This eliminates **85–92%** of full candidate evaluations in practice.

**3. k-Nearest Neighbor Pruning**
Perpendicular bisectors are computed only between each site and its k ≈ 8 nearest neighbors, matching the structure of the Delaunay graph. This reduces candidate generation by approximately **89%** while maintaining solution optimality, since geometrically distant bisectors cannot yield the global maximum clearance.

**4. Incremental Obstacle Updates**
Obstacle positions are recomputed via lightweight path interpolation in O(m) per frame. The Delaunay triangulation and candidate pool are preserved across frames — only the distance evaluation step is re-executed with updated obstacle centers, avoiding the full O(n log n) reconstruction cost of static methods.

> These four strategies together yield a **2–3 order-of-magnitude** improvement over brute-force enumeration while preserving geometric optimality.

---

## Limitations and Future Work

### Current Limitations

| Limitation | Impact |
| :--- | :--- |
| Restricted to 2D workspaces | Cannot be directly applied to aerial or volumetric planning |
| Circular obstacles only | Non-circular real-world shapes require preprocessing or approximation |
| Perfect obstacle localization assumed | No model for sensor noise or positional uncertainty |
| Single-threaded Python implementation | Interpreter overhead becomes significant at large n |
| Evaluated on synthetic datasets only | Real sensor data may introduce different statistical properties |

### Future Research Directions

- **3D extension** — Generalize to the Largest Empty Sphere for aerial robotics and volumetric motion planning
- **Non-circular obstacles** — Support arbitrary shapes via convex decomposition or signed distance fields
- **GPU acceleration** — Parallel candidate evaluation for order-of-magnitude throughput improvements
- **Compiled implementation** — C++ or Rust port to eliminate interpreter overhead and enable SIMD vectorization
- **ROS 2 / Gazebo / CARLA integration** — Validation under realistic sensor noise and physics simulation
- **Probabilistic uncertainty models** — Robust LEC computation over obstacle position distributions rather than deterministic point estimates
- **Learning-based adaptive pruning** — ML-guided candidate validity prediction to reduce per-frame evaluation cost in structured environments

---

## Applications

DO-LEC is designed for any domain requiring real-time identification of maximum-clearance safe zones amid moving obstacles:

- Autonomous ground robot navigation and drone flight planning
- Warehouse automation with mobile human workers
- Emergency response vehicle routing through dynamic traffic
- Human-robot collaboration in shared workspaces
- Healthcare robotics navigation in hospital environments
- Adaptive facility location and spatial optimization systems

---

## Acknowledgements

The authors express sincere gratitude to **Mr. Abiral Sangroula**, Principal Engineer at QSystems AI, for his technical guidance, optimization expertise, and valuable insights throughout this research. His contributions significantly influenced the refinement of the DO-LEC algorithm and the overall quality of this work.

---

## Citation

If you use DO-LEC or build upon this work, please cite the original publication:

**APA**
> Pathak, S., & Pokhrel, J. (2025). DO-LEC (Dynamic Obstacle-Aware Largest Empty Circle): Fast and adaptive LEC computation for navigating dynamic obstacle fields. *International Journal on Engineering Technology*, *3*(1), 62–80. https://doi.org/10.3126/injet.v3i1.86978

**IEEE**
> S. Pathak and J. Pokhrel, "DO-LEC (Dynamic Obstacle-Aware Largest Empty Circle): Fast and Adaptive LEC Computation for Navigating Dynamic Obstacle Fields," *International Journal on Engineering Technology*, vol. 3, no. 1, pp. 62–80, Dec. 2025. doi: 10.3126/injet.v3i1.86978.

**BibTeX**
```bibtex
@article{pathak2025dolec,
  author  = {Pathak, Samriddha and Pokhrel, Jyoti},
  title   = {{DO-LEC} ({Dynamic Obstacle-Aware Largest Empty Circle}):
             Fast and Adaptive {LEC} Computation for Navigating
             Dynamic Obstacle Fields},
  journal = {International Journal on Engineering Technology},
  volume  = {3},
  number  = {1},
  pages   = {62--80},
  month   = dec,
  year    = {2025},
  doi     = {10.3126/injet.v3i1.86978},
  issn    = {3021-940X}
}
```

---

## Repository

All experimental datasets, test case generation scripts, raw performance measurements, and the complete GUI source code are publicly available:

**[https://github.com/SamriddhaPathak/Algorithmic_Solutions_to_the_LEC_Problem](https://github.com/SamriddhaPathak/Algorithmic_Solutions_to_the_LEC_Problem)**

---

## License

This software accompanies a peer-reviewed open-access publication (ISSN: 3021-940X). It is made available for academic and non-commercial research use. For commercial licensing inquiries, please contact the corresponding author at samriddha.805421@cct.tu.edu.np.
