# yaw.jl — pure yaw (rotate about the vertical axis).
using ThrusterLab

vehicle = bluerov_vehicle()
τ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.5]      # τz only
report(allocate(vehicle, τ); actuators=vehicle.actuators)
