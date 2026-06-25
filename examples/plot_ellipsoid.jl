# plot_ellipsoid.jl — the 3-D manipulability ellipsoid (requires Plots).
#   julia --project=. -e 'using Pkg; Pkg.add("Plots")'   # one-time
#   julia --project=. examples/plot_ellipsoid.jl
using ThrusterLab
using Plots

vehicle = bluerov_vehicle()
plt = plot_manipulability(vehicle; block=:force)   # force-wrench ellipsoid
savefig(plt, joinpath(@__DIR__, "manipulability.png"))
println("wrote examples/manipulability.png")
