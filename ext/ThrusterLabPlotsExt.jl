module ThrusterLabPlotsExt

# Package extension: loaded automatically when both ThrusterLab and Plots are
# present in the session. Provides the graphical `plot_thrusters` / `plot_vehicle`.

using ThrusterLab
using ThrusterLab: AbstractActuator, Thruster, ReactionWheel, Vehicle
import Plots

# Implementation lives in src/plotting.jl; it defines
# ThrusterLab.plot_thrusters and ThrusterLab.plot_vehicle.
include(joinpath(@__DIR__, "..", "src", "plotting.jl"))

end # module
