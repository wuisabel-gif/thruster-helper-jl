# allocation_matrix.jl
# Assemble the 6×N allocation matrix B and apply the forward map τ = B f.

"""
    allocation_matrix(actuators) -> Matrix{Float64}  (6 × N)
    allocation_matrix(vehicle)   -> Matrix{Float64}

Stack the per-actuator wrench columns into the allocation matrix `B`, so the
body wrench produced by command vector `f` is `τ = B f`:

    τ = [Fx, Fy, Fz, τx, τy, τz]   (force then torque, body frame)
    f = [f1, …, fN]               (signed command per actuator)

Column `i` is `wrench_column(actuators[i])`.
"""
function allocation_matrix(actuators::AbstractVector{<:AbstractActuator})
    n = length(actuators)
    n > 0 || throw(ArgumentError("need at least one actuator"))
    B = Matrix{Float64}(undef, 6, n)
    @inbounds for i in 1:n
        col = wrench_column(actuators[i])
        length(col) == 6 || throw(ArgumentError("actuator $i produced a non-6 wrench column"))
        B[:, i] = col
    end
    return B
end

allocation_matrix(v::Vehicle) = allocation_matrix(v.actuators)

"""
    wrench(B, f) -> Vector{Float64}

Forward map: the 6-DOF wrench `τ = B f` produced by commands `f`.
"""
wrench(B::AbstractMatrix, f::AbstractVector) = B * f
wrench(v::Vehicle, f::AbstractVector) = allocation_matrix(v) * f

"""
    command_bounds(actuators) -> (lo, hi)

Per-actuator command limits as two vectors, gathered from each actuator's
`command_limits`. Used by the constrained (`:qp`) solver and saturation tools.
"""
function command_bounds(actuators::AbstractVector{<:AbstractActuator})
    lo = Float64[]; hi = Float64[]
    for a in actuators
        l, h = command_limits(a)
        push!(lo, l); push!(hi, h)
    end
    return lo, hi
end
command_bounds(v::Vehicle) = command_bounds(v.actuators)

"Per-actuator maximum |command| (saturation limits)."
max_thrusts(actuators::AbstractVector{<:AbstractActuator}) = [a.max_thrust for a in actuators]
max_thrusts(v::Vehicle) = max_thrusts(v.actuators)
