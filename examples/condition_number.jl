# condition_number.jl — geometry affects how *evenly* a vehicle can be driven.
# A poorly-conditioned allocation matrix means some directions need far more
# thrust than others.
#
# The force rows of B have magnitude ~1, but the torque rows scale with the
# lever arm. Shrink the arm and the torque DOFs get weak relative to the force
# DOFs → κ(B) blows up. Sweep `arm` and watch the conditioning.
using ThrusterHelper
using Printf

@printf("%-10s %8s %12s %14s\n", "arm [m]", "rank", "cond κ", "manipulability")
for arm in (0.05, 0.10, 0.22, 0.40, 0.80)
    d = diagnostics(bluerov_heavy(; arm=arm))
    @printf("%-10.2f %8d %12.3f %14.4g\n", arm, d.rank, d.condition_number, d.manipulability)
end
println("\nA short arm (cramped thrusters) → weak torque authority → large κ.")
println("Note: the vertical `span` does NOT help — vertical thrusters push along")
println("z, so a z-offset adds no roll/pitch torque. Only the x/y arm does.")
