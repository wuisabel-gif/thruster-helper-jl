# roll.jl — pure roll (τx), produced by the vertical thrusters pushing opposite
# sides up/down.
using ThrusterLab

vehicle = bluerov_vehicle()
τ = [0.0, 0.0, 0.0, 0.3, 0.0, 0.0]      # τx only
report(allocate(vehicle, τ); actuators=vehicle.actuators)
