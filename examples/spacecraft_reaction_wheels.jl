# spacecraft_reaction_wheels.jl — the allocation math is not underwater-specific.
# A satellite with 4 reaction wheels in a pyramid produces pure torque; the same
# allocation_matrix / allocate / diagnostics pipeline applies. Reaction wheels
# only actuate the 3 torque DOFs, so force DOFs are (correctly) uncontrollable.
using ThrusterLab

β = deg2rad(54.7)   # classic pyramid half-angle
wheels = [
    ReactionWheel("rw1", [ cos(β),  0.0, sin(β)]),
    ReactionWheel("rw2", [-cos(β),  0.0, sin(β)]),
    ReactionWheel("rw3", [ 0.0,  cos(β), sin(β)]),
    ReactionWheel("rw4", [ 0.0, -cos(β), sin(β)]),
]
sat = Vehicle("CubeSat", wheels; mass=4.0)

describe(sat)
println()
report(diagnostics(sat))      # τx, τy, τz controllable; forces are not

println("\nCommand a yaw torque:")
report(allocate(sat, [0.0, 0.0, 0.0, 0.0, 0.0, 0.1]); actuators=sat.actuators)
