# Examples

Each file is a tiny, self-contained script. Run any of them with:

```bash
julia --project=. examples/<name>.jl
```

### Single-DOF motions
- `forward.jl` — pure surge (Fx).
- `yaw.jl` — pure yaw (τz).
- `hover.jl` — pure heave (Fz) to hold depth.
- `roll.jl` — pure roll (τx).

### Failures & under-actuation
- `failed_thruster.jl` — kill a thruster, compare control authority before/after.
- `underactuated.jl` — the 4-thruster quad can't do heave/roll/pitch.
- `rank_loss.jl` — watch `rank(B)` fall as thrusters fail one by one.

### Numerical analysis
- `condition_number.jl` — how thruster spacing changes conditioning.
- `diagnostics.jl` — full SVD-based design report.

### Solvers
- `weighted_solver.jl` — penalise a weak thruster.
- `minimum_power.jl` — minimum-power vs minimum-norm allocation.
- `qp_solver.jl` — bounded (saturation-aware) allocation.
- `solver_comparison.jl` — run every method on one command, side by side.
- `compare_methods.jl` — the built-in solver-comparison table.

### Design analysis & optimisation
- `reachability.jl` — can the vehicle even produce this wrench?
- `failure_analysis.jl` — rank each thruster by how critical it is.
- `monte_carlo.jl` — robustness to thruster misalignment / position noise.
- `optimize_layout.jl` — re-aim thrusters to improve κ / manipulability.

### Beyond AUVs
- `spacecraft_reaction_wheels.jl` — the same pipeline for a satellite.

### Plotting (needs `using Plots`)
- `plot_layout.jl` — render the thruster layout to a PNG.
- `plot_ellipsoid.jl` — draw the 3-D manipulability ellipsoid.
