# optimize_layout.jl — from *analyse my AUV* to *design my AUV*.
# Start from a cramped, poorly-conditioned layout and let the optimiser re-aim
# the thrusters to minimise the condition number (then maximise manipulability).
using ThrusterHelper

# Short arms → weak torque authority → high condition number.
vehicle = bluerov_vehicle(; arm=0.1)
println("Starting design:")
report(diagnostics(vehicle))

println("\nOptimise thruster orientations for condition number:")
res = optimize_layout(vehicle; objective=:condition_number, restarts=3, iterations=3000)
report(res)

println("\nSame, but maximising manipulability:")
report(optimize_layout(vehicle; objective=:manipulability, restarts=3, iterations=3000))
