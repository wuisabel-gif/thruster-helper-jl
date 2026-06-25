# solver_comparison.jl — run every allocation method on one command and compare.
# This is the "framework for comparing allocation algorithms" view of the
# project in a single screen.
using ThrusterHelper
using Printf

thr = bluerov_heavy()
B   = allocation_matrix(thr)
τ   = [1.0, 0.3, 0.0, 0.0, 0.0, 0.4]

@printf("%-15s %10s %10s %10s %12s\n", "method", "‖f‖₂", "total|f|", "power", "residual")
for (m, kw) in ((:minimum_norm, ()),
                (:weighted,      (weights = [4,1,1,1,1,1,1,1.0],)),  # penalise thruster 1
                (:minimum_power, ()),
                (:qp,            (bounds = 1.0,)))
    r = allocate(B, τ; method=m, kw...)
    l2 = sqrt(sum(abs2, r.commands))
    @printf("%-15s %10.4f %10.4f %10.4f %12.2e\n",
            m, l2, sum(abs.(r.commands)),
            total_power(r.commands), maximum(abs.(r.residual)))
end
println("\nminimum_norm minimises ‖f‖₂; minimum_power lowers total draw; weighted")
println("steers effort off thruster 1; qp matches min_norm here (within bounds).")
