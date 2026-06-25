# constraints.jl
# Hardware constraints applied to / around the solver: saturation limits,
# direction-preserving scaling, failure injection, and the power model.

"""
    saturate(f, limits) -> (f_sat, saturated)

Hard-clamp each command `f[i]` to `[-limits[i], +limits[i]]` (`limits` may be a
scalar or per-actuator vector). Returns the clamped command and a `Bool` mask of
which actuators hit their limit.

Clamping changes the *direction* of the produced wrench; when that matters,
prefer [`scale_to_limits`](@ref) (preserves direction) or the `:qp` solver
(optimal within bounds).
"""
function saturate(f::AbstractVector, limits)
    lim = limits isa Number ? fill(float(limits), length(f)) : collect(Float64, limits)
    length(lim) == length(f) ||
        throw(ArgumentError("limits length $(length(lim)) ≠ commands length $(length(f))"))
    f_sat = clamp.(f, -lim, lim)
    saturated = abs.(f) .> lim .+ 1e-12
    return f_sat, saturated
end

"""
    scale_to_limits(f, limits) -> (f_scaled, factor)

If any `|f[i]| > limits[i]`, scale the *whole* command vector by the single
worst-case factor so the largest command sits exactly at its limit. Preserves
the direction of the produced wrench (you get less of it, undistorted). Returns
the scaled command and the applied `factor ∈ (0, 1]`.
"""
function scale_to_limits(f::AbstractVector, limits)
    lim = limits isa Number ? fill(float(limits), length(f)) : collect(Float64, limits)
    length(lim) == length(f) ||
        throw(ArgumentError("limits length $(length(lim)) ≠ commands length $(length(f))"))
    worst = maximum(abs.(f) ./ lim)
    factor = worst > 1 ? 1 / worst : 1.0
    return f .* factor, factor
end

"""
    apply_failures(B, failed) -> B_failed

Zero the columns of `B` for failed actuators (a dead actuator produces no wrench
however it is commanded). `failed` is a vector of indices (`[3, 5]`) or a `Bool`
mask of length N. Returns a copy.
"""
function apply_failures(B::AbstractMatrix, failed)
    Bf = copy(Matrix{Float64}(B))
    mask = _failure_mask(failed, size(B, 2))
    @inbounds for i in axes(B, 2)
        mask[i] && (Bf[:, i] .= 0.0)
    end
    return Bf
end

"""
    failed_indices(failed, n) -> Vector{Int}

Normalise a failure spec (index vector or length-`n` `Bool` mask) to a sorted
vector of failed actuator indices.
"""
failed_indices(failed, n::Integer) = findall(_failure_mask(failed, n))

function _failure_mask(failed, n::Integer)
    if eltype(failed) == Bool && length(failed) == n
        return collect(Bool, failed)
    else
        mask = falses(n)
        for i in failed
            (1 <= i <= n) || throw(ArgumentError("failed index $i out of range 1:$n"))
            mask[i] = true
        end
        return mask
    end
end

# ---------------------------------------------------------------------------
# Power model
# ---------------------------------------------------------------------------

"""
    estimate_power(f; k=1.0, p=1.5, idle=0.0) -> Vector{Float64}

Rough per-actuator electrical power (watts). Marine thrusters draw roughly
`power ∝ |thrust|^1.5` (thrust ∝ rpm², power ∝ rpm³), so

    power[i] = idle + k * |f[i]|^p

Tune `k`, `p` to a thruster's bench data. Sum for total draw.
"""
estimate_power(f::AbstractVector; k::Real=1.0, p::Real=1.5, idle::Real=0.0) =
    idle .+ k .* abs.(f) .^ p

"Total estimated electrical power for a command vector."
total_power(f::AbstractVector; kwargs...) = sum(estimate_power(f; kwargs...))
