# rank_loss.jl — watch control authority collapse as thrusters fail one by one.
using ThrusterLab
using Printf

thr = bluerov_heavy()
B   = allocation_matrix(thr)

@printf("%-28s %5s %12s\n", "after failing", "rank", "redundancy")
failed = Int[]
for i in 0:6
    i > 0 && push!(failed, i)
    d = diagnostics(apply_failures(B, failed))
    name = i == 0 ? "(none)" : thr[i].name
    @printf("%-28s %5d %12d\n", name, d.rank, d.redundancy)
end
println("\nRank stays at 6 while the vehicle is over-actuated; once enough")
println("thrusters are gone, DOFs become uncontrollable and rank drops.")
