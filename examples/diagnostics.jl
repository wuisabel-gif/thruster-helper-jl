# diagnostics.jl — the full SVD-based design report for a vehicle.
using ThrusterLab

vehicle = bluerov_vehicle()
describe(vehicle)
println()
report(diagnostics(vehicle))

# The raw LinearAlgebra primitives are re-exported, so you can dig further:
B = allocation_matrix(vehicle)
println("\nSingular values: ", round.(svdvals(B); digits=4))
println("Null-space dim : ", size(nullspace(B), 2), "  (redundant control)")
