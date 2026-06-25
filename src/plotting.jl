# plotting.jl
# Implementation of the graphical force-vector visualisation.
#
# NOT included by the core module (which is dependency-free). Loaded by the
# package extension ext/ThrusterLabPlotsExt.jl once the user runs `using Plots`,
# and defines ThrusterLab.plot_thrusters / ThrusterLab.plot_vehicle.

"""
    plot_thrusters(actuators; commands=nothing, failed=Int[], view=:xy, kwargs...)

Plot the actuator layout and force vectors (requires `using Plots`).

- `commands` : optional command per actuator; arrows scale with magnitude and
               are coloured by sign (green = forward/+, red = reverse/−). With
               no commands, unit push directions are drawn.
- `failed`   : indices drawn greyed with an ✕.
- `view`     : `:xy` (top-down), `:xz` (side) or `:yz` (rear).

Only `Thruster`s are drawn (they have a position); other actuators are skipped.
Returns the `Plots.Plot`.
"""
function ThrusterLab.plot_thrusters(actuators::AbstractVector{<:AbstractActuator};
                                    commands=nothing, failed=Int[],
                                    view::Symbol=:xy, arrowscale::Real=0.15, kwargs...)
    ax = view === :xy ? (1, 2, "x [m]", "y [m]") :
         view === :xz ? (1, 3, "x [m]", "z [m]") :
         view === :yz ? (2, 3, "y [m]", "z [m]") :
         throw(ArgumentError("view must be :xy, :xz or :yz, got :$view"))
    ix, iy, xl, yl = ax

    n = length(actuators)
    failmask = ThrusterLab.failed_indices(failed, n)

    plt = Plots.plot(; xlabel=xl, ylabel=yl, aspect_ratio=:equal,
                     legend=false, title="ThrusterLab — $(view) view", kwargs...)
    Plots.scatter!(plt, [0.0], [0.0]; marker=:cross, markersize=8, color=:black)

    for (i, a) in enumerate(actuators)
        a isa Thruster || continue
        px, py = a.position[ix], a.position[iy]
        isfailed = i in failmask
        mag = commands === nothing ? 1.0 : commands[i]
        dx = a.direction[ix] * mag * arrowscale
        dy = a.direction[iy] * mag * arrowscale
        col = isfailed ? :gray : (mag >= 0 ? :seagreen : :crimson)
        Plots.scatter!(plt, [px], [py]; markersize=6, color=col,
                       marker=(isfailed ? :xcross : :circle))
        if !isfailed && (abs(dx) + abs(dy)) > 1e-9
            Plots.quiver!(plt, [px], [py]; quiver=([dx], [dy]), color=col, linewidth=2)
        end
        Plots.annotate!(plt, px, py, Plots.text("  $(a.name)", 7, :left))
    end
    return plt
end

"""
    plot_vehicle(vehicle; kwargs...)

Plot a [`Vehicle`](@ref)'s actuators (forwards to [`plot_thrusters`](@ref)).
"""
ThrusterLab.plot_vehicle(v::Vehicle; kwargs...) =
    ThrusterLab.plot_thrusters(v.actuators; kwargs...)

"""
    plot_manipulability(vehicle; block=:force, kwargs...)

Draw the 3-D **manipulability ellipsoid** of the allocation matrix (requires
`using Plots`). The ellipsoid is the set of wrenches reachable with a unit
command ball; its principal axes are the singular vectors of `B`, scaled by the
singular values. A long axis = a cheap-to-produce direction; the short axis is
the design's weak spot (drawn as a red arrow).

`block` selects the 3×3 sub-block to visualise: `:force` (rows Fx,Fy,Fz) or
`:torque` (rows τx,τy,τz). Returns the `Plots.Plot`.
"""
function ThrusterLab.plot_manipulability(v::Vehicle; block::Symbol=:force, kwargs...)
    B = ThrusterLab.allocation_matrix(v)
    rows = block === :force ? (1:3) : block === :torque ? (4:6) :
           throw(ArgumentError("block must be :force or :torque"))
    labels = block === :force ? ("Fx", "Fy", "Fz") : ("τx", "τy", "τz")
    M = B[rows, :]
    F = svd(M)                                    # svd re-exported by ThrusterLab
    U, S = F.U, F.S

    # unit sphere → ellipsoid via U * diag(S)
    nu, nv = 24, 24
    us = range(0, 2π; length=nu); vs = range(0, π; length=nv)
    xs = zeros(nu, nv); ys = similar(xs); zs = similar(xs)
    for i in 1:nu, j in 1:nv
        p = [cos(us[i])*sin(vs[j]), sin(us[i])*sin(vs[j]), cos(vs[j])]
        q = U * (S .* p)
        xs[i, j], ys[i, j], zs[i, j] = q[1], q[2], q[3]
    end

    plt = Plots.surface(xs, ys, zs; alpha=0.45, color=:viridis, legend=false,
                        xlabel=labels[1], ylabel=labels[2], zlabel=labels[3],
                        title="Manipulability ellipsoid ($(block))", kwargs...)
    # principal axes; weakest (smallest σ) in red
    for k in 1:3
        ax = U[:, k] .* S[k]
        col = k == 3 ? :red : :black
        Plots.plot!(plt, [-ax[1], ax[1]], [-ax[2], ax[2]], [-ax[3], ax[3]];
                    color=col, linewidth=(k == 3 ? 3 : 2))
    end
    return plt
end
