# monte_carlo.jl — robustness to manufacturing error.
# Perturb every thruster's pointing direction by ±misalignment and resample the
# design thousands of times: how often is a DOF lost? what's the spread of κ?
using ThrusterLab

vehicle = bluerov_vehicle()

println("Tight build (±2° misalignment): degrades conditioning, keeps full rank")
report(monte_carlo(vehicle; misalignment_deg=2.0, samples=5000, seed=1))

println("\nWith a 10%-per-thruster failure rate: now DOF loss becomes likely")
report(monte_carlo(vehicle; misalignment_deg=5.0, failure_prob=0.10, samples=5000, seed=1))
