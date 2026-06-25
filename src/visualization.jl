# visualization.jl
# Text / ASCII reporting (dependency-free). Graphical force-vector plots live in
# the optional Plots extension (ext/ThrusterLabPlotsExt.jl), exposed as
# `plot_thrusters` / `plot_vehicle`.

"""
    describe(actuators; io=stdout)
    describe(vehicle;   io=stdout)

Print a table of the actuator geometry: name, position, push direction and
limit.
"""
function describe(actuators::AbstractVector{<:AbstractActuator}; io::IO=stdout)
    println(io, "Actuators (", length(actuators), "):")
    @printf(io, "  %-3s %-22s %-22s %-22s %8s\n", "#", "name", "position [m]", "direction", "limit")
    for (i, a) in enumerate(actuators)
        pos = a isa Thruster ? string(round.(a.position; digits=3)) : "—"
        dir = a isa Thruster ? string(round.(a.direction; digits=3)) :
              a isa ReactionWheel ? string(round.(a.axis; digits=3)) * " (torque)" : "—"
        @printf(io, "  %-3d %-22s %-22s %-22s %8.2f\n", i, label(a), pos, dir, a.max_thrust)
    end
    return nothing
end

function describe(v::Vehicle; io::IO=stdout)
    println(io, "Vehicle: \"", v.name, "\"")
    isnan(v.mass) || @printf(io, "  mass = %.3f kg\n", v.mass)
    describe(v.actuators; io=io)
    return nothing
end

"""
    bar(value, scale; width=20) -> String

A small signed ASCII bar, centred on zero, with `scale` mapped to the half-width.
"""
function bar(value::Real, scale::Real; width::Integer=20)
    scale == 0 && (scale = 1)
    frac = clamp(value / scale, -1, 1)
    cells = round(Int, abs(frac) * width)
    return value >= 0 ?
        string(" "^width, "|", "█"^cells, " "^(width - cells)) :
        string(" "^(width - cells), "█"^cells, "|", " "^width)
end

"""
    report(result; actuators=nothing, B=nothing, io=stdout)

Pretty-print an [`AllocationResult`](@ref): the desired-vs-achieved wrench, a
signed bar chart of the per-actuator commands, and a summary block (residual,
power, most-loaded actuator). If `actuators` (or a matrix `B`) is supplied, the
rank and condition number of the design are included.
"""
function report(result::AllocationResult; actuators=nothing, B=nothing, io::IO=stdout)
    println(io, "Allocation report  (method = :", result.method, ")")
    println(io, "─"^52)
    println(io, "Desired vs achieved wrench:")
    @printf(io, "  %-4s %12s %12s %12s\n", "DOF", "desired", "achieved", "residual")
    for k in 1:6
        @printf(io, "  %-4s %12.4f %12.4f %12.4f\n",
                _DOF_LABELS[k], result.desired[k], result.achieved[k], result.residual[k])
    end
    rn = norm(result.residual)
    @printf(io, "  residual norm: %.5g  %s\n", rn,
            rn < 1e-6 ? "(fully achievable)" : "(under-actuated / clipped)")

    f = result.commands
    scale = maximum(abs.(f); init=0.0)
    println(io, "\nActuator commands:")
    for i in eachindex(f)
        name = actuators === nothing ? "T$i" : label(actuators[i])
        flag = (actuators !== nothing && abs(f[i]) > actuators[i].max_thrust + 1e-9) ?
               "  ⚠ SATURATED" : ""
        @printf(io, "  %-22s %8.4f  %s%s\n", name, f[i], bar(f[i], scale), flag)
    end

    # summary block
    Bmat = B !== nothing ? B : (actuators !== nothing ? allocation_matrix(actuators) : nothing)
    imax = isempty(f) ? 0 : argmax(abs.(f))
    most = imax == 0 ? "—" : (actuators === nothing ? "T$imax" : label(actuators[imax]))
    println(io, "\nSummary:")
    @printf(io, "  total |command|     : %.4f\n", sum(abs.(f)))
    @printf(io, "  est. power          : %.4f W\n", total_power(f))
    @printf(io, "  most-loaded actuator: %s (%.4f)\n", most, imax == 0 ? 0.0 : f[imax])
    if Bmat !== nothing
        d = diagnostics(Bmat)
        @printf(io, "  rank / condition #  : %d/6  ,  κ = %.4g\n", d.rank, d.condition_number)
    end
    return nothing
end

"""
    report(d::AllocationDiagnostics; io=stdout)

Print the SVD-based design diagnostics: rank, redundancy, condition number,
manipulability, per-DOF control authority, singular values and the weakest
wrench direction.
"""
function report(d::AllocationDiagnostics; io::IO=stdout)
    println(io, "Design diagnostics  (", d.n_dof, " DOF × ", d.n_actuators, " actuators)")
    println(io, "─"^52)
    @printf(io, "  rank               : %d / 6  %s\n", d.rank,
            d.rank == 6 ? "(fully actuated)" : "(UNDER-ACTUATED)")
    @printf(io, "  redundancy (null)  : %d\n", d.redundancy)
    @printf(io, "  condition number κ : %.4g  %s\n", d.condition_number,
            d.condition_number < 5 ? "(well-conditioned)" :
            d.condition_number < 50 ? "(moderate)" : "(ill-conditioned)")
    @printf(io, "  manipulability     : %.4g\n", d.manipulability)
    println(io, "  control authority  :")
    for k in 1:6
        @printf(io, "      %-3s : %s\n", _DOF_LABELS[k], d.controllable[k] ? "ok" : "LOST")
    end
    println(io, "  singular values    : ", round.(d.singular_values; digits=4))
    @printf(io, "  weakest direction  : %s-dominated, gain σ_min = %.4g\n",
            dominant_dof(d.weakest_direction), d.weakest_gain)
    return nothing
end
