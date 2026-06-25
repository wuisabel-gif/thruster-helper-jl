# solver.jl
# The allocation solvers. `allocate(...; method=...)` selects between several
# algorithms so the package is a *framework for comparing allocation methods*,
# not a single pinv wrapper.
#
#   :minimum_norm / :pinv  minimise ‖f‖₂        s.t. Bf = τ      (Moore–Penrose)
#   :weighted              minimise ‖W f‖₂      s.t. Bf = τ      (penalise actuators)
#   :minimum_power         minimise Σ|fᵢ|ᵖ      s.t. Bf = τ      (IRLS, p≈1.5)
#   :qp                    minimise ‖Bf-τ‖₂²    s.t. lo ≤ f ≤ hi (bounded LS / QP)
#
# All are dependency-free (FISTA / IRLS built on LinearAlgebra only).

const ALLOCATION_METHODS = (:minimum_norm, :pinv, :weighted, :minimum_power, :qp)

"""
    allocate(B, τ; method=:minimum_norm, kwargs...) -> AllocationResult
    allocate(actuators, τ; ...)        # builds B, default bounds from limits
    allocate(vehicle, τ; ...)          # builds B, default bounds from limits

Solve for the actuator commands `f` that produce the desired wrench `τ` (6-vec),
using the chosen `method` (see [`ALLOCATION_METHODS`](@ref)).

Keyword arguments
- `weights`  : length-N positive costs for `:weighted` (larger ⇒ discourage).
- `bounds`   : command limits for `:qp`, as `(lo, hi)` vectors, a per-actuator
               vector `u` (⇒ `±u`), or a scalar. Defaults to the actuators'
               `max_thrust` when called on `actuators`/`vehicle`.
- `p`        : exponent for `:minimum_power` (default `1.5`; marine thrusters).
- `λ`        : optional Tikhonov damping added to the least-squares methods,
               useful on ill-conditioned geometries.
- `maxiter`, `tol` : iteration controls for the iterative methods.
"""
function allocate(B::AbstractMatrix, τ::AbstractVector;
                  method::Symbol=:minimum_norm,
                  weights=nothing, bounds=nothing,
                  p::Real=1.5, λ::Real=0.0,
                  maxiter::Integer=5000, tol::Real=1e-10)
    size(B, 1) == 6 || throw(ArgumentError("B must have 6 rows, got $(size(B,1))"))
    length(τ) == 6 || throw(ArgumentError("desired wrench must have 6 elements, got $(length(τ))"))
    d = collect(Float64, τ)
    n = size(B, 2)

    f = if method === :minimum_norm || method === :pinv
        _min_norm(B, d, λ)
    elseif method === :weighted
        weights === nothing && throw(ArgumentError(":weighted requires `weights`"))
        w = _check_weights(weights, n)
        _weighted_min_norm(B, d, w, λ)
    elseif method === :minimum_power
        _minimum_power(B, d, p, λ, maxiter, tol)
    elseif method === :qp
        lo, hi = _resolve_bounds(bounds, n)
        _bounded_lstsq(B, d, lo, hi, λ, maxiter, tol)
    else
        throw(ArgumentError("unknown method :$method; choose one of $(ALLOCATION_METHODS)"))
    end

    achieved = B * f
    return AllocationResult(method, f, d, achieved, achieved .- d)
end

function allocate(actuators::AbstractVector{<:AbstractActuator}, τ;
                  method::Symbol=:minimum_norm, bounds=nothing, kwargs...)
    B = allocation_matrix(actuators)
    if method === :qp && bounds === nothing
        bounds = command_bounds(actuators)        # default QP box from hardware limits
    end
    return allocate(B, τ; method=method, bounds=bounds, kwargs...)
end

allocate(v::Vehicle, τ; kwargs...) = allocate(v.actuators, τ; kwargs...)

# ---------------------------------------------------------------------------
# Solver implementations
# ---------------------------------------------------------------------------

# Minimum-norm (Moore–Penrose), optionally Tikhonov-damped:
#   λ = 0 : f = pinv(B) τ
#   λ > 0 : f = Bᵀ (B Bᵀ + λI)⁻¹ τ      (well-defined even at rank loss)
function _min_norm(B, τ, λ)
    if λ <= 0
        return pinv(B) * τ
    else
        return B' * ((B * B' + λ * I) \ τ)
    end
end

# Weighted minimum norm: minimise ‖W f‖₂ s.t. Bf = τ, W = diag(w).
function _weighted_min_norm(B, τ, w, λ=0.0)
    Winv = Diagonal(1 ./ w)
    Bw = B * Winv
    if λ <= 0
        return Winv * (pinv(Bw) * τ)
    else
        return Winv * (Bw' * ((Bw * Bw' + λ * I) \ τ))
    end
end

# Minimum p-norm via Iteratively Reweighted Least Squares.
# Minimise Σ|fᵢ|ᵖ s.t. Bf = τ by repeatedly solving a weighted min-norm with
# weights wᵢ = |fᵢ|^((p-2)/2). For 1 < p < 2 this favours spreading the load and
# approximates minimum electrical power (power ∝ |f|^1.5).
function _minimum_power(B, τ, p, λ, maxiter, tol)
    f = _min_norm(B, τ, λ)
    (1 < p < 2) || return f                      # p=2 is just min-norm
    ε = 1e-6
    for _ in 1:min(maxiter, 100)
        w = max.(abs.(f), ε) .^ ((p - 2) / 2)
        fnew = _weighted_min_norm(B, τ, w, λ)
        norm(fnew - f) <= tol * (norm(fnew) + ε) && (f = fnew; break)
        f = fnew
    end
    return f
end

# Bounded-variable least squares (the QP):
#   minimise 0.5‖Bf - τ‖² + 0.5λ‖f‖²   s.t.   lo ≤ f ≤ hi
# Solved with FISTA (accelerated projected gradient). Dependency-free; converges
# for this convex problem. With infinite bounds and λ=0 it reproduces the
# minimum-norm solution, so :qp degrades gracefully to :pinv when unconstrained.
function _bounded_lstsq(B, τ, lo, hi, λ, maxiter, tol)
    n = size(B, 2)
    BtB = B' * B
    Btτ = B' * τ
    L = opnorm(B)^2 + λ + 1e-12                  # Lipschitz const of the gradient
    step = 1 / L
    grad(f) = BtB * f - Btτ + λ .* f

    x = clamp.(zeros(n), lo, hi)
    y = copy(x); t = 1.0
    for _ in 1:maxiter
        xnew = clamp.(y .- step .* grad(y), lo, hi)
        tnew = (1 + sqrt(1 + 4t^2)) / 2
        y = xnew .+ ((t - 1) / tnew) .* (xnew .- x)
        if norm(xnew - x) <= tol * (norm(xnew) + 1e-12)
            return xnew
        end
        x = xnew; t = tnew
    end
    return x
end

# ---------------------------------------------------------------------------
# Argument helpers
# ---------------------------------------------------------------------------

function _check_weights(weights, n)
    w = collect(Float64, weights)
    length(w) == n || throw(ArgumentError("weights must have $n elements, got $(length(w))"))
    all(>(0), w) || throw(ArgumentError("weights must be strictly positive"))
    return w
end

# Normalise the many `bounds` spellings into (lo, hi) vectors.
function _resolve_bounds(bounds, n)
    if bounds === nothing
        return fill(-Inf, n), fill(Inf, n)
    elseif bounds isa Tuple
        lo = bounds[1] isa Number ? fill(float(bounds[1]), n) : collect(Float64, bounds[1])
        hi = bounds[2] isa Number ? fill(float(bounds[2]), n) : collect(Float64, bounds[2])
        (length(lo) == n && length(hi) == n) ||
            throw(ArgumentError("bounds vectors must have length $n"))
        return lo, hi
    elseif bounds isa Number
        return fill(-float(bounds), n), fill(float(bounds), n)
    else                                            # per-actuator symmetric vector
        u = collect(Float64, bounds)
        length(u) == n || throw(ArgumentError("bounds vector must have length $n"))
        return -abs.(u), abs.(u)
    end
end
