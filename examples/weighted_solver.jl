# weighted_solver.jl — discourage one (weak / nearly-failed) thruster by giving
# it a large weight. The solver routes effort to the others.
using ThrusterLab

thr = bluerov_heavy()
B   = allocation_matrix(thr)
τ   = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]

println("Unweighted:")
report(allocate(B, τ); actuators=thr)

w = ones(8); w[1] = 50.0       # thruster 1 is suspect → penalise it
println("\nWeighted (penalise ", thr[1].name, "):")
report(allocate(B, τ; method=:weighted, weights=w); actuators=thr)
