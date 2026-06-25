# qp_solver.jl — bounded (saturation-aware) allocation.
# Demand more force than the thrusters can deliver. The :qp solver finds the
# best achievable wrench *within* the ±max_thrust box, instead of returning a
# command that would clip.
using ThrusterHelper

vehicle = bluerov_vehicle()                 # thrusters limited to ±1 N
# Max surge inside the ±1 box is ~2.83 N; ask for more so it cannot be met.
τ = [4.0, 0.0, 0.0, 0.0, 0.0, 0.0]

println("Minimum-norm (ignores limits, would saturate):")
report(allocate(vehicle, τ; method=:minimum_norm); actuators=vehicle.actuators)

println("\nQP (optimal within ±max_thrust):")
report(allocate(vehicle, τ; method=:qp); actuators=vehicle.actuators)
