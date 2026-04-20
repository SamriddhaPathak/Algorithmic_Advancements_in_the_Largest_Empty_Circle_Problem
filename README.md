# DO-LEC: Dynamic Obstacle-Aware Largest Empty Circle

> Fast and Adaptive LEC Computation for Navigating Dynamic Obstacle Fields

**Published in:** International Journal on Engineering Technology (InJET), Vol. 3, No. 1, Dec 2025, pp. 62–80  
**DOI:** [10.3126/injet.v3i1.86978](https://doi.org/10.3126/injet.v3i1.86978)  
**Authors:** Samriddha Pathak¹, Jyoti Pokhrel²

¹ Central Campus of Technology, Dharan, Sunsari, Nepal — samriddha.805421@cct.tu.edu.np  
² Soch College of IT, Pokhara, Kaski, Nepal — jyoti2079@sochcollege.edu.np

---

## Abstract

Autonomous systems navigating dynamic environments require continuous identification of maximum-clearance safe zones. Existing Largest Empty Circle (LEC) algorithms assume static obstacle configurations and necessitate complete recomputation when obstacles move — a fundamental limitation that prevents real-time deployment.

**DO-LEC** is the first practical algorithm to compute geometrically optimal safe zones in environments with moving obstacles while maintaining real-time performance. It integrates classical Voronoi-Delaunay geometric principles with adaptive candidate generation and incremental obstacle tracking, achieving efficient updates without full reconstruction. Unlike approximation-based methods that sacrifice optimality for speed, DO-LEC preserves geometric correctness while achieving practical real-time capability.

---

## Table of Contents

- [Problem Definition](#problem-definition)
- [Algorithm Overview](#algorithm-overview)
- [Complexity Analysis](#complexity-analysis)
- [Getting Started](#getting-started)
- [Experimental Results](#experimental-results)
- [Optimization Strategies](#optimization-strategies)
- [Limitations and Future Work](#limitations-and-future-work)
- [Applications](#applications)
- [Citation](#citation)

---

## Problem Definition

Given a 2D bounded rectangular workspace **B ⊂ ℝ²** containing:

- A set of **static point sites** S = {s₁, s₂, ..., sₙ} ⊂ ℝ²
- A set of **dynamic circular obstacles** O = {o₁, ..., oₘ}, each with fixed radius rⱼ > 0 and a time-varying center Oⱼ(t) moving along a piecewise-linear path

The goal is to compute, at any time *t*, the **Largest Empty Circle (LEC)** that:

1. Lies entirely within the workspace B
2. Contains no site in its interior
3. Does not intersect any dynamic obstacle

### Mathematical Formulation

The feasible region at time *t* is defined as:

**F(t) = { x ∈ B : ∀j, |x − Oⱼ(t)| ≥ rⱼ }**

The optimal LEC center is:

**c\*(t) = argmax over x∈F(t) of min{ dist(x, S), dist(x, O(t)), dist(x, ∂B) }**

Where:
- dist(x, S) = minᵢ |x − sᵢ| — distance to nearest site
- dist(x, O(t)) = minⱼ max(0, |x − Oⱼ(t)| − rⱼ) — clearance to nearest obstacle
- dist(x, ∂B) — distance to workspace boundary

The resulting optimal radius is:

**R\*(t) = min{ dist(c\*(t), S), dist(c\*(t), O(t)), dist(c\*(t), ∂B) }**

---

## Algorithm Overview

DO-LEC operates in three phases:

### Phase 1 — Candidate Generation

Candidate circle centers are identified from geometrically privileged locations:

- **Delaunay triangulation circumcenters** — which correspond to Voronoi vertices (full triangulation for n ≤ 50; √n sampled triangulation for n > 50)
- **Boundary samples** — four rectangle corners and four edge midpoints
- **Perpendicular bisector intersections** — each site's k nearest neighbors (k = min(8, n−1)) are used to compute bisector–boundary intersections

Duplicate candidates are removed via grid hashing (10⁻⁶ precision), and out-of-bounds points are discarded.

### Phase 2 — Dynamic Evaluation

For each candidate point p = (x, y), the feasible radius is:

**r(p) = min( d_bound, minᵢ |p − sᵢ|, minⱼ (|p − Oⱼ| − Rⱼ) )**

Where boundary distance is computed in O(1) as:

**d_bound = min{ x − x_min, x_max − x, y − y_min, y_max − y }**

**Early termination** is applied when:
- A partial site distance drops below a minimum threshold (~1.0 unit)
- Any obstacle clearance is non-positive (circle would intersect obstacle)
- Current candidate radius falls below the best known radius

Obstacle positions are updated each frame via **ping-pong path interpolation** in O(m) time.

### Phase 3 — Fallback and Selection

The candidate with the maximum feasible radius is selected. A fallback check using the workspace center is applied when the best radius falls below threshold δ. The algorithm returns the circle (center, radius) with the globally optimal clearance.

### System Pipeline

```
┌─────────────────────────────────────┐
│     PHASE 1: CANDIDATE GENERATION   │
│  - Boundary sampling                │
│  - Adaptive triangulation           │
│  - Bisector intersections           │
└──────────────────┬──────────────────┘
                   │
┌──────────────────▼──────────────────┐
│  PHASE 2: OPTIMAL CIRCLE SELECTION  │
│  - Evaluate each candidate          │
│  - Track best solution              │
│  - Early termination                │
└──────────────────┬──────────────────┘
                   │
┌──────────────────▼──────────────────┐
│     PHASE 3: FALLBACK & RETURN      │
│  - Verify solution quality          │
│  - Apply fallback if needed         │
│  - Return optimal circle            │
└─────────────────────────────────────┘
```

---

## Complexity Analysis

| Component | Time Complexity | Description |
|---|---|---|
| Voronoi diagram construction | O(n log n) | Fortune's sweepline algorithm |
| Convex hull computation | O(n log n) | Graham scan with optimizations |
| LEC computation | O(n log n + nk) | Delaunay triangulation with k-NN pruning |
| Dynamic obstacle updates | O(m) | Incremental position tracking per frame |
| Rendering / visualization | O(n + m) | Linear drawing operations |
| **DO-LEC overall** | **O(n log n + nk + m)** | k ≈ 8 (small constant) |

---

## Getting Started

### Prerequisites

- Python 3.x
- NumPy
- SciPy
- Matplotlib
- Tkinter (bundled with most Python distributions)

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

The system is implemented in Python 3.x (tested on Windows 11, Ryzen 5 5500U, 24 GB RAM). All logic is encapsulated in the `DynamicLECVoronoi` class. The GUI is built with Tkinter and contains a left-hand control panel and a right-hand Matplotlib drawing canvas (800×600 workspace with a 50-unit margin).

### Core Methods

| Method | Description |
|---|---|
| `compute_lec(sites, obstacles)` | Generates candidates and returns the largest feasible circle (center, radius) |
| `get_current_obstacle_positions(time)` | Linearly interpolates each obstacle along its waypoint path with ping-pong reflection |
| `draw()` | Clears and redraws the scene: Voronoi diagram, convex hull, LEC, sites, and obstacles |
| `update_stats()` | Updates real-time statistics display (site count, obstacle count, elapsed time) |

### GUI Features

- **Mode selection** — Add Sites / Add Obstacles
- **Animation controls** — Play / Reset with adjustable speed
- **Display toggles** — Voronoi Diagram, Convex Hull, Largest Empty Circle
- **Real-time statistics** — site count, obstacle count, elapsed time
- **Obstacle management** — path configuration and clearing

---

## Experimental Results

### Test Case Categories

| Category | Cases | n Range | m Range | Distribution | Purpose |
|---|---|---|---|---|---|
| Micro | 1–4 | 5–10 | 1–4 | Uniform | Baseline validation |
| Small | 5–7 | 100–200 | 10–30 | Clustered | Interactive robotics |
| Medium | 8–10 | 250–400 | 50–100 | Grid-perturbed | Simulation environments |
| Large | 11–13 | 500–700 | 150–250 | Uniform | Batch processing |
| Stress | 14–16 | 800–1000 | 300–400 | Mixed | Scalability limits |

Obstacle configuration: radii ∈ [10, 25] pixels, speed = 1.0 unit/second, ping-pong interpolation, m ≈ 0.4n for n ≥ 100. All timing values are the mean of 10 independent runs; standard deviations were <5% of mean values.

### Runtime Performance

| Scale | Sites (n) | Obstacles (m) | Time w/o LEC | Time w/ LEC |
|---|---|---|---|---|
| Micro | 5 | 1 | 1.0 ms | 1.8 ms |
| Micro | 10 | 4 | 0.7 ms | 1.5 ms |
| Small | 100 | 10 | 1.5 ms | 14.0 ms |
| Small | 200 | 30 | 2.8 ms | 32.4 ms |
| Medium | 300 | 75 | 4.5 ms | 54.2 ms |
| Medium | 400 | 100 | 6.2 ms | 76.8 ms |
| Large | 500 | 150 | 8.1 ms | 102.3 ms |
| Large | 700 | 250 | 12.6 ms | 163.2 ms |
| Stress | 900 | 350 | 18.4 ms | 242.6 ms |
| Stress | 1000 | 400 | 21.8 ms | 290.8 ms |

### DO-LEC vs. Static LEC

The Static LEC baseline uses Fortune's sweepline algorithm recomputed from scratch each frame with no temporal coherence. Both algorithms use identical candidate generation and evaluation logic.

| n | m | Static LEC | DO-LEC | Overhead | Performance Ratio |
|---|---|---|---|---|---|
| 5 | 1 | 1.3 ms | 1.8 ms | +38.5% | 0.72× |
| 100 | 10 | 9.8 ms | 14.0 ms | +42.9% | 0.70× |
| 500 | 150 | 58.4 ms | 102.3 ms | +75.2% | 0.57× |
| 1000 | 400 | 148.3 ms | 290.8 ms | +96.1% | 0.51× |

The overhead percentage stabilizes at **95–96% for n ≥ 500**, confirming that dynamic obstacle tracking introduces bounded, predictable computational cost.

### Dynamic Overhead Breakdown

| Problem Size | n | m | Obstacle Updates | k-factor Cost | State Management | Total Overhead |
|---|---|---|---|---|---|---|
| Small | 10 | 4 | 0.1 ms | 0.2 ms | 0.1 ms | 0.4 ms |
| Medium | 100 | 10 | 0.3 ms | 2.8 ms | 1.1 ms | 4.2 ms |
| Large | 500 | 150 | 4.2 ms | 28.4 ms | 11.3 ms | 43.9 ms |
| Very Large | 1000 | 400 | 12.0 ms | 85.2 ms | 45.3 ms | 142.5 ms |

### Memory and Accuracy

- **Memory:** O(n + m); measured from 45 KB (n=5) to 2.1 MB (n=1000, m=400)
- **Accuracy:** 100% LEC identification rate verified against brute-force for n ≤ 15; computed radii match ground truth within ≤ 10⁻¹² relative error

---

## Optimization Strategies

DO-LEC achieves its efficiency through four key strategies:

1. **√n Sampling** — For n > 50, a subset triangulation (every k-th point, k ≈ √n) is used instead of full Delaunay, reducing triangulation overhead by **65–78%** while generating a representative candidate set.

2. **Early Termination** — Evaluation of a candidate stops as soon as its partial radius falls below the current best. Obstacle intersection and site proximity filters eliminate **85–92%** of full evaluations.

3. **k-NN Pruning** — Perpendicular bisectors are computed only for each site's k ≈ 8 nearest neighbors, reducing candidate generation by approximately **89%** while preserving solution optimality.

4. **Incremental Updates** — Obstacle positions are updated via path interpolation in O(m) per frame. Geometric structures (Delaunay triangulation, candidate set) are reused across frames rather than rebuilt, avoiding full recomputation.

Together these strategies achieve **2–3 orders of magnitude** improvement over brute-force approaches.

---

## Limitations and Future Work

### Current Limitations

- Restricted to 2D environments
- Supports only circular obstacles
- Assumes perfect obstacle localization (no sensor noise model)
- Single-threaded Python introduces interpreter overhead at large scale

### Future Directions

- **3D extension** — Largest Empty Sphere for aerial robotics and volumetric planning
- **Non-circular obstacles** — Shape decomposition or distance field representations
- **GPU acceleration** — Parallel clearance evaluation for order-of-magnitude speedup
- **Compiled implementation** — C++ or Rust to eliminate interpreter overhead and enable SIMD
- **ROS 2 integration** — Validation under realistic sensor models in Gazebo / CARLA
- **Learning-based pruning** — ML-predicted candidate validity to optimize the efficiency–rigor tradeoff

---

## Applications

- Autonomous ground robotics and drone navigation
- Warehouse automation with mobile workers
- Emergency response facility deployment
- Human-robot collaborative environments
- Healthcare robotics and hospital navigation
- Adaptive spatial optimization systems

---

## Acknowledgements

The authors express sincere gratitude to **Mr. Abiral Sangroula**, Principal Engineer at QSystems AI, for his technical guidance, optimization expertise, and valuable insights throughout this research.

---

## Citation

If you use DO-LEC in your work, please cite:

**APA**
> Pathak, S., & Pokhrel, J. (2025). DO-LEC (Dynamic Obstacle-Aware Largest Empty Circle): Fast and adaptive LEC computation for navigating dynamic obstacle fields. *International Journal on Engineering Technology*, *3*(1), 62–80. https://doi.org/10.3126/injet.v3i1.86978

**IEEE**
> S. Pathak and J. Pokhrel, "DO-LEC (Dynamic Obstacle-Aware Largest Empty Circle): Fast and Adaptive LEC Computation for Navigating Dynamic Obstacle Fields," *International Journal on Engineering Technology*, vol. 3, no. 1, pp. 62–80, Dec. 2025.

**BibTeX**
```bibtex
@article{pathak2025dolec,
  author  = {Pathak, S. and Pokhrel, J.},
  title   = {DO-LEC (Dynamic Obstacle-Aware Largest Empty Circle): Fast and Adaptive LEC Computation for Navigating Dynamic Obstacle Fields},
  journal = {International Journal on Engineering Technology},
  volume  = {3},
  number  = {1},
  pages   = {62--80},
  month   = dec,
  year    = {2025},
  doi     = {10.3126/injet.v3i1.86978}
}
```

---

## License

This project is made available for academic and research use. All experimental datasets, test case generation code, raw performance measurements, and GUI source code are publicly available at:

**[https://github.com/SamriddhaPathak/Algorithmic_Solutions_to_the_LEC_Problem](https://github.com/SamriddhaPathak/Algorithmic_Solutions_to_the_LEC_Problem)**
