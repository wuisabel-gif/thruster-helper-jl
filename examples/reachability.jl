# reachability.jl — can the vehicle even produce this wrench?
# Allocation alone returns *some* command; reachable() respects the saturation
# limits and tells you the closest achievable wrench when the answer is "no".
using ThrusterHelper

vehicle = bluerov_vehicle()      # thrusters limited to ±1 N

println("Modest forward push:")
report(reachable(vehicle, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

println("\nToo much forward push (asks 4 N, vehicle tops out near 2.83 N):")
report(reachable(vehicle, [4.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
