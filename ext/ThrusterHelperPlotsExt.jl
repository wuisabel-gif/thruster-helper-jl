module ThrusterHelperPlotsExt

# Package extension: loaded automatically when both ThrusterHelper and Plots are
# present in the session. Provides the graphical `plot_thrusters` / `plot_vehicle`.

using ThrusterHelper
using ThrusterHelper: AbstractActuator, Thruster, ReactionWheel, Vehicle
import Plots

# Implementation lives in src/plotting.jl; it defines
# ThrusterHelper.plot_thrusters and ThrusterHelper.plot_vehicle.
include(joinpath(@__DIR__, "..", "src", "plotting.jl"))

end # module
