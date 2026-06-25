# hover.jl — pure heave (hold depth against buoyancy/weight).
using ThrusterHelper

vehicle = bluerov_vehicle()
τ = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]      # Fz only → the 4 vertical thrusters
report(allocate(vehicle, τ); actuators=vehicle.actuators)
