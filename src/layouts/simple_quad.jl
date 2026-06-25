# layouts/simple_quad.jl
# A minimal four horizontal-thruster vehicle: surge / sway / yaw only.
# Deliberately under-actuated (no heave/roll/pitch) — useful for teaching and
# for demonstrating rank loss and control-authority analysis.

"""
    simple_quad(; arm=0.2, max_thrust=1.0) -> Vector{Thruster}

Four horizontal thrusters at the corners, angled at 45°. Controls surge (Fx),
sway (Fy) and yaw (τz); heave/roll/pitch are uncontrollable, so `rank(B) == 3`.
"""
function simple_quad(; arm::Real=0.2, max_thrust::Real=1.0)
    a = float(arm); c = sqrt(2) / 2; mt = float(max_thrust)
    return [
        Thruster("FR", [ a, -a, 0.0], [ c,  c, 0.0]; max_thrust=mt),
        Thruster("FL", [ a,  a, 0.0], [ c, -c, 0.0]; max_thrust=mt),
        Thruster("BR", [-a, -a, 0.0], [-c,  c, 0.0]; max_thrust=mt),
        Thruster("BL", [-a,  a, 0.0], [-c, -c, 0.0]; max_thrust=mt),
    ]
end

"""
    quad_vehicle(; kwargs...) -> Vehicle

The [`simple_quad`](@ref) thrusters wrapped in a [`Vehicle`](@ref).
"""
quad_vehicle(; mass::Real=5.0, kwargs...) =
    Vehicle("Simple Quad", simple_quad(; kwargs...); mass=mass)
