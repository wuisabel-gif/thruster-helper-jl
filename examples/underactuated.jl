# underactuated.jl — the 4-thruster quad controls surge/sway/yaw only.
# Asking for heave leaves a large residual.
using ThrusterLab

vehicle = quad_vehicle()
report(diagnostics(vehicle))

println("\nAsk for heave (Fz) it cannot produce:")
r = allocate(vehicle, [0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
report(r; actuators=vehicle.actuators)
