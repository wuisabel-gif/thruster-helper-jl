# geometry.jl
# Low-level geometric primitives shared by the actuator types and diagnostics.
#
# Coordinate convention used by the built-in layouts: x forward, y left, z up;
# positions are body-frame, expressed about the centre of mass. A thruster's
# contribution splits cleanly into a force and a torque:
#
#     force  = direction
#     torque = position × direction
#
# and the two stacked form its column in the 6×N allocation matrix.

"""
    skew(v) -> 3×3 Matrix

Skew-symmetric ("cross-product") matrix `[v]ₓ` such that `skew(v) * w == v × w`.
Handy when assembling allocation matrices in bulk or differentiating geometry.
"""
function skew(v::AbstractVector)
    length(v) == 3 || throw(ArgumentError("skew expects a 3-vector"))
    return [   0.0   -v[3]   v[2]
             v[3]    0.0   -v[1]
            -v[2]   v[1]    0.0]
end

"""
    force_contribution(t::Thruster) -> SVec3

The body-frame force a unit command on thruster `t` produces (its push
direction).
"""
force_contribution(t::Thruster) = copy(t.direction)

"""
    torque_contribution(t::Thruster) -> SVec3

The body-frame torque a unit command on thruster `t` produces, `position ×
direction`.
"""
torque_contribution(t::Thruster) = cross(t.position, t.direction)

"""
    column(a::AbstractActuator) -> Vector{Float64}

The 6-element wrench column `[Fx, Fy, Fz, τx, τy, τz]` produced by a unit
command to actuator `a`. Alias for [`wrench_column`](@ref); kept because it
reads naturally when talking about "columns of `B`".
"""
column(a::AbstractActuator) = wrench_column(a)
