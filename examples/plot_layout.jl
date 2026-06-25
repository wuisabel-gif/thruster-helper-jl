# plot_layout.jl — graphical force-vector plot (requires Plots).
#   julia --project=. -e 'using Pkg; Pkg.add("Plots")'   # one-time
#   julia --project=. examples/plot_layout.jl
using ThrusterHelper
using Plots                                   # triggers the plotting extension

vehicle = bluerov_vehicle()
r = allocate(vehicle, [1.0, 0.0, 0.0, 0.0, 0.0, 0.5]; method=:qp)

plt = plot_vehicle(vehicle; commands=r.commands, failed=[1, 5], view=:xy)
savefig(plt, joinpath(@__DIR__, "thruster_layout.png"))
println("wrote examples/thruster_layout.png")
