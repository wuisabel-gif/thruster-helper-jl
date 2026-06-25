# minimum_power.jl — minimum-power allocation vs plain minimum-norm.
# Power ∝ |thrust|^1.5, so minimising Σ|f|^1.5 (IRLS) can beat the least-‖f‖₂
# solution on total electrical draw.
using ThrusterHelper
using Printf

thr = bluerov_heavy()
B   = allocation_matrix(thr)
τ   = [1.0, 0.5, 0.0, 0.0, 0.0, 0.4]

for m in (:minimum_norm, :minimum_power)
    r = allocate(B, τ; method=m)
    @printf("%-15s  total|f| = %.4f   est.power = %.4f W   residual = %.2e\n",
            m, sum(abs.(r.commands)), total_power(r.commands), maximum(abs.(r.residual)))
end
