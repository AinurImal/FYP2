# BCM Evacuation Simulation — A* + PSO

MATLAB simulation of crowd evacuation in the Borneo Cultures Museum Level 2,
using **A* pathfinding** for routes and **Particle Swarm Optimization** for
agent motion. Final Year Project, UNIMAS.

---

## What This Simulates

80 agents are spawned across the BCM Level 2 floor plan and must evacuate
through one of four green exit zones. Each agent finds an optimal route with
A* and follows it under PSO velocity dynamics.

---

## Files

| File                  | Purpose                                                  |
|-----------------------|----------------------------------------------------------|
| `untitled5.m`   | Main simulation script (run this)                        |
| `BCM-L2-SIM-2.jpg`    | Floor plan image — must be in MATLAB working directory   |

---

## How to Run

1. Open MATLAB.
2. Set the working directory to the folder containing both files above.
3. In the MATLAB Command Window:
   ```matlab
   untitled5
   ```
4. A figure window opens with the simulation map (left), live evacuation
   graph (top-right), and results panel (bottom-right). After a 3-second
   countdown the evacuation begins.

Console output prints map dimensions, walkable pixel count, A* planning
time, periodic progress updates, and a final results summary.

---

## Algorithm

### A* Pathfinding (per agent, run once at startup)

- 8-connected coarse grid (4-px cells with 1-px halo) — guarantees no
  waypoint can land on or adjacent to a wall.
- **Octile heuristic:** `h(n) = D(dx+dy) + (D2−2D)·min(dx, dy)`, with
  `D = 1`, `D2 = √2`. Admissible (never overestimates) and informative
  (`h > 0` everywhere except at the goal).
- Binary min-heap priority queue ordered by `f = g + h`.
- Goal test on heap extraction → guarantees optimal path (A* optimality
  theorem, Hart-Nilsson-Raphael 1968).
- Diagonal corner-cut prevention: both flanking cells must be walkable.
- Path then smoothed via line-of-sight on the full pixel grid.

### PSO Motion (per agent, every iteration)

The agent's canonical state is **`progress(i)`** — a scalar arc-length
along its A* path. Pixel position is *derived* from progress via
interpolation. This makes wall collision structurally impossible.

Each iteration, for every non-arrived agent:

1. Get current position and path tangent at `progress(i)`.
2. Compute look-ahead pursuit point ~25 px further along path.
3. PSO 2D velocity update:
   `vel = w·vel + c1·r1·(pbest − pos) + c2·r2·social_pull`
   where `social_pull = 0.5·(look_ahead − pos) + 0.5·(tangent · 5·speed)`.
4. Project velocity onto path tangent — forward component only, with a
   minimum-step guarantee.
5. Advance progress, update position via interpolation.
6. Check arrival.
7. Stuck mechanism: if `progress` fails to advance ≥ 0.3 px over 12
   iterations, force-jump 17.6 px forward along the (wall-safe) path.

PSO uses a **social-only variant** (`c1 = 0`). With forward-only motion,
`pbest = pos` always, so the cognitive term contributes nothing useful;
disabling it eliminates the backward-pull failure mode.

### Why the A* + PSO Coupling Works

A* provides the deterministic wall-safe path. PSO provides the stochastic
swarm dynamics (inertia, social attraction). The two are coupled by
projecting the PSO 2D velocity onto the A* path tangent — only the forward
component is realised as motion. This guarantees agents respect walls
without explicit collision detection.

---

## Tunable Parameters

All in `untitled5.m`. Search for the section number to find each.

### Section 2 — Wall detection
| Parameter     | Default | Effect                                                     |
|---------------|---------|------------------------------------------------------------|
| `wall_thresh` | `40`    | Pixel intensity below this counts as wall. Increase if thin black lines are missed; decrease if coloured rooms become walls. |

### Section 3 — Exit positions
4 green-zone centres in pixel coordinates. Edit the `exits` matrix to add,
move, or remove exits. Block an exit at runtime with `exit_blocked(2) = true;`.

### Section 4 — A* planning grid
| Parameter   | Default | Effect                                                       |
|-------------|---------|--------------------------------------------------------------|
| `cell_size` | `4`     | Coarse-cell size in pixels. Larger = faster A*, coarser paths. Smaller = slower, finer paths. |

### Section 6 — Agent placement
| Parameter     | Default | Effect                                            |
|---------------|---------|---------------------------------------------------|
| `N`           | `80`    | Number of agents.                                 |
| `min_spacing` | `10`    | Minimum spawn separation. Decrease if not enough agents fit. |

### Section 7 — Movement physics
| Parameter            | Default | Justification                       |
|----------------------|---------|-------------------------------------|
| `walking_speed_ms`   | `1.47`  | Standard SFM walking speed (m/s).   |
| `agent_radius_m`     | `0.2`   | Standard SFM agent radius (m).      |
| `map_scale_px_per_m` | `30`    | Calibration: pixels per real-world metre on the BCM floor plan. |
| `step_duration_s`    | `0.05`  | Simulated seconds per iteration.    |

### Section 8 — PSO weights
| Parameter | Default | Tuning range  | Notes                       |
|-----------|---------|---------------|-----------------------------|
| `w_pso`   | `0.4`   | 0.3 – 0.7     | Inertia.                    |
| `c1`      | `0.0`   | keep at 0     | Cognitive (disabled).       |
| `c2`      | `3.5`   | 2.5 – 4.0     | Social pull strength.       |

### Section 10 — Stuck mechanism
| Parameter          | Default | Effect                                              |
|--------------------|---------|-----------------------------------------------------|
| `stuck_window`     | `12`    | Iterations of low progress before recovery fires.   |
| `min_step_advance` | `0.3`   | Below this px/iteration, count toward stuck.        |
| `jump_distance`    | `17.6`  | How far forward along the path to jump on recovery. |
| `min_fwd`          | `0.4·agent_speed` | Minimum forward step every iteration. Guarantees motion. |

### Section 13 — Loop control
| Parameter   | Default | Effect                                  |
|-------------|---------|-----------------------------------------|
| `max_iter`  | `5000`  | Hard iteration cap.                     |
| Timeout     | `300 s` | Sim ends after 300 s wall-clock seconds.|

---

## Output

### Console
```
╔══════════════════════════════════════════════╗
║          EVACUATION RESULTS                  ║
╚══════════════════════════════════════════════╝
  Total time         : XX.XX sec
  Evacuated          : N / N (100.0%)
  Fastest            : X.XX sec
  Slowest            : XX.XX sec
  Average            : X.XX sec
  Std dev            : X.XX sec
```

### Figure
- **Left:** floor plan with cyan agents and green exits.
- **Top-right:** live count of evacuated agents vs iteration.
- **Bottom-right:** final results panel.

---

## Switching Maps

To run with a different BCM floor plan (e.g. `BCM-L2-SIM-4.jpg`):

1. Place the new image in the working directory.
2. In Section 1, change `imread('BCM-L2-SIM-2.jpg')` to the new filename.
3. In Section 3, update the `exits` matrix to match the new map's green
   zones (use MATLAB's Data Cursor on the loaded image to find pixel
   coordinates).

---

## Common Issues

| Symptom                                        | Fix                                                          |
|------------------------------------------------|--------------------------------------------------------------|
| `error: BCM-L2-SIM-2.jpg not found`            | File not in MATLAB working directory.                        |
| Many "unplannable agents" reported             | Spawn region isolated by walls. Lower `wall_thresh` (Sec 2). |
| Agents clip through thin walls                 | Increase wall dilation in Sec 2 from `disk(1)` to `disk(2)`. |
| Coloured rooms wrongly detected as walls       | Lower `wall_thresh` from 40 toward 30.                       |
| Doorways look impassable                       | Decrease wall dilation; verify `cell_size` is small enough.  |
| Sim too slow                                   | Comment out `pause(0.05)` in Section 13 main loop.           |



## Reference

Hart, P. E., Nilsson, N. J., & Raphael, B. (1968).
*A Formal Basis for the Heuristic Determination of Minimum Cost Paths.*
IEEE Transactions on Systems Science and Cybernetics, 4(2), 100–107.
