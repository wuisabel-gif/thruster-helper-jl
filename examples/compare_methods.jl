# compare_methods.jl — one call runs every solver and tabulates the trade-offs
# (residual, ‖f‖₂, power, saturated thrusters, time). This is the researcher's
# view of the project.
using ThrusterHelper

vehicle = bluerov_vehicle()
τ = [1.0, 0.3, 0.0, 0.0, 0.0, 0.4]
report(compare_methods(vehicle, τ))
