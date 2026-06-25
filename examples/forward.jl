# forward.jl — pure surge (move straight ahead).
using ThrusterLab

vehicle = bluerov_vehicle()
τ = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]      # Fx only
report(allocate(vehicle, τ); actuators=vehicle.actuators)
