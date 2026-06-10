# CARLA Autonomous Navigation — Custom Dijkstra & A\* Path Planning

A self-contained point-to-point navigation system for the [CARLA simulator](https://carla.org/) (version 0.9.x). The project implements **two graph-search planners from scratch — Dijkstra's algorithm and A\* (A-Star)** — over a road graph extracted directly from the simulator map, then drives a vehicle along the computed route with a custom PID-based controller while a chase camera follows the car.

It deliberately does **not** use CARLA's built-in `GlobalRoutePlanner`. The topological graph, the search algorithms, the controller, and the camera logic are all implemented in plain Python.

---

## 🎥 Demo Videos

> **TODO:** replace the placeholder links below with your own YouTube URLs.

| Run | Video |
|-----|-------|
| Dijkstra planner — full A → B run in CARLA | ▶️ [Watch on YouTube](https://youtu.be/CWJg42tZ8CI)
| A\* planner — full A → B run in CARLA | ▶️ [Watch on YouTube](https://youtu.be/e8b5mMvgPIM)|

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Repository Structure](#repository-structure)
4. [Prerequisites](#prerequisites)
5. [Installation](#installation)
6. [Usage](#usage)
7. [How It Works](#how-it-works)
8. [Algorithm Comparison](#algorithm-comparison)
9. [Troubleshooting](#troubleshooting)
10. [Project Report](#project-report)
11. [Authors](#authors)
12. [License](#license)

---

## Overview

Global route planning is framed here as a **shortest-path search over a weighted directed graph**. The graph is built at run time from the simulator's road network, both search algorithms operate on the *same* graph abstraction, and the resulting waypoint route is tracked by a longitudinal + lateral controller. The two planners are provided as independent, runnable scripts that share an identical command-line interface, which makes it easy to run and compare them on the same start/destination pair.

## Features

- **Custom graph construction** from `map.get_topology()` — nodes are road-segment endpoints quantized to whole-meter coordinates; edges store the densified waypoint geometry and are weighted by true road arc-length.
- **Two from-scratch planners:** Dijkstra (uniform-cost) and A\* (Euclidean-heuristic guided), each with correct forward route reconstruction.
- **Route visualization** drawn as a continuous green line slightly above the road via `world.debug.draw_line`.
- **PID-based vehicle control** with separate longitudinal and lateral loops, a **speed-dependent look-ahead** for smooth tracking, automatic **slow-down before curves**, and real braking on overspeed.
- **Third-person chase camera** that follows the vehicle, positioned behind it using the forward vector multiplied by a negative distance (raised and pitched down).
- **Synchronous fixed-delta simulation** (0.05 s) for deterministic control.
- **Robust setup/cleanup:** clear connection diagnostics, an optional collision sensor for feedback, and a `try/finally` block that destroys all actors and restores the original world settings on completion, error, or `Ctrl-C`.
- **Configurable from the command line:** host, port, start/destination spawn indices, cruise speed, and vehicle blueprint.

## Repository Structure

```
carla-autonomous-navigation/
├── README.md
├── LICENSE
├── requirements.txt
├── .gitignore
├── src/
│   ├── carla_astar_navigation.py       # A* planner + controller + camera (single file)
│   ├── carla_dijkstra_navigation.py    # Dijkstra planner + controller + camera (single file)
│   └── START.py                        # convenience launcher (forwards args to a planner)
└── docs/
    └── AStar_vs_Dijkstra_Project_Report.docx   # academic comparison report
```

## Prerequisites

- **CARLA simulator 0.9.x** installed and running (tested against 0.9.13–0.9.15 era APIs).
- **Python 3.7+** matching the version supported by your CARLA build.
- The **`carla` Python API** that ships with your simulator (the `.egg`/wheel must match the server version).
- **`numpy`**.

## Installation

```bash
# 1. Clone the repository
git clone https://github.com/orior11/carla-autonomous-navigation.git
cd carla-autonomous-navigation

# 2. (Recommended) create a virtual environment
python -m venv .venv
source .venv/bin/activate        # on Windows: .venv\Scripts\activate

# 3. Install dependencies
pip install -r requirements.txt
```

> If the `carla` pip package does not match your simulator, install the wheel/egg bundled with your CARLA download instead, or pin the exact version in `requirements.txt`.

## Usage

First, start the CARLA server, e.g.:

```bash
./CarlaUE4.sh                       # Linux
# or  CarlaUE4.exe                  # Windows
```

Then run a planner. Both scripts expose the same flags.

**A\* (recommended):**
```bash
python src/carla_astar_navigation.py --host 127.0.0.1 --port 2000 --a 0 --b 100
```

**Dijkstra:**
```bash
python src/carla_dijkstra_navigation.py --a 0 --b 120 --speed 25
```

**Via the launcher:**
```bash
python src/START.py astar --a 0 --b 100
python src/START.py dijkstra --a 0 --b 120
```

### Command-line options

| Flag | Default | Description |
|------|---------|-------------|
| `--host` | `127.0.0.1` | CARLA server host. |
| `--port` | `2000` | CARLA RPC port. **Must match the server** (see Troubleshooting). |
| `--a` | `0` | Start spawn-point index (Point A). |
| `--b` | `100` | Destination spawn-point index (Point B). |
| `--speed` | `22.0` | Cruise speed in km/h on straight segments. |
| `--vehicle` | `vehicle.tesla.model3` | Vehicle blueprint id. |
| `--no-draw` | off | Disable drawing the route line in the world. |

Point A and Point B are chosen by **spawn-point index** into `map.get_spawn_points()`. Indices wrap around the available spawn points, so out-of-range values are reduced modulo the count.

## How It Works

### 1. Graph construction from topology
`map.get_topology()` returns sparse `(entry, exit)` directed road segments. The planner:
- Assigns each endpoint a **node id** equal to its location rounded to whole meters, so endpoints shared by adjacent segments collapse onto one node and junctions become connected.
- **Densifies** every segment by walking the real lane geometry with `waypoint.next(resolution)`, storing the dense waypoint list on the edge and using its accumulated **arc-length** as the edge weight (not straight-line distance).

### 2. Search
- **Dijkstra** orders its frontier purely by cost-so-far `g(n)` and expands nodes in concentric (uninformed) order until the goal is finalized.
- **A\*** orders its frontier by `f(n) = g(n) + h(n)`, where `h(n)` is the **Euclidean straight-line distance to the goal**. Because road distance is always ≥ straight-line distance, the heuristic is admissible and consistent, so A\* returns a cost-optimal path while expanding far fewer nodes. The A\* script prints the number of nodes it expanded for each query.

Both planners reconstruct the route **forward** (start → goal), de-duplicating shared junction nodes and preserving the start and goal points.

### 3. Vehicle control
A longitudinal PID tracks a target speed (braking when overspeed), and a lateral PID steers toward a **look-ahead waypoint** whose distance scales with speed (clamped to a small range so the car follows the lane tightly instead of cutting corners). Target speed is automatically reduced ahead of curves based on the upcoming path curvature.

### 4. Chase camera
Every simulation step the spectator is placed behind the car by taking `vehicle.get_forward_vector()` and multiplying it by a **negative** distance (≈6 m back), raised ≈3 m, and pitched slightly downward — so you watch the car drive the route in real time.

### 5. Simulation lifecycle
The world runs in **synchronous mode** at a fixed delta of 0.05 s. A `try/finally` block guarantees that all spawned actors (vehicle and collision sensor) are destroyed and the original world settings are restored on normal completion, on exceptions, or on keyboard interrupt.

## Algorithm Comparison

Both algorithms return a route of identical, provably optimal cost. Their difference is computational: Dijkstra explores the map isotropically, whereas A\*'s admissible heuristic focuses exploration toward the goal and expands a strict subset of the nodes Dijkstra must expand. For single-source, single-goal point-to-point queries, **A\* is therefore the preferred planner** — same optimal path, materially less work. A full, formal analysis is provided in [`docs/AStar_vs_Dijkstra_Project_Report.docx`](docs/AStar_vs_Dijkstra_Project_Report.docx).

## Troubleshooting

- **`Could not reach the CARLA server` / connection timeout** — the simulator is not running, or `--port` does not match the server's RPC port. CARLA listens on **2000** by default; if you started it with `-carla-rpc-port=3000`, run the script with `--port 3000`.
- **Vehicle fails to spawn** — the chosen start spawn point may be occupied; try a different `--a` index. The scripts also retry once at a slightly raised position.
- **No path found** — the start and destination may lie in disconnected parts of the road graph; pick different spawn indices.
- **`carla` import error** — the installed `carla` API version does not match your server; install the wheel/egg that ships with your simulator.
- **Version compatibility** — the API is stable across 0.9.x, but minor method signatures can differ between point releases; this project targets the 0.9.13–0.9.15 generation.

## Project Report

A comprehensive academic write-up — covering the graph-construction methodology, the mechanics of both algorithms, a comparative analysis of their node-expansion behaviour, and a formal justification for selecting A\* — is included as a Word document in [`docs/`](docs/).

## Authors

| Name | ID |
|------|----|
| Ori Zarfaty (אורי צרפתי) | 208213678 |
| Romi Yosef (רומי יוסף) | 209363274 |
| Natan Beer (נתן BEER) | 208788356 |

## License

Released under the [MIT License](LICENSE).
