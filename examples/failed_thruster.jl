# failed_thruster.jl — what does losing a thruster cost?
# Compare the allocation and control authority for "forward + yaw" before and
# after two thrusters die.
using ThrusterHelper

thr = bluerov_heavy()
B   = allocation_matrix(thr)
τ   = [1.0, 0.0, 0.0, 0.0, 0.0, 0.5]

println("=== Healthy ===")
report(allocate(B, τ); actuators=thr)

failed = [1, 5]
println("\n=== After losing ", [thr[i].name for i in failed], " ===")
Bf = apply_failures(B, failed)
report(allocate(Bf, τ); actuators=thr)

println()
report(diagnostics(Bf))     # still rank 6? which DOF got weakest?
