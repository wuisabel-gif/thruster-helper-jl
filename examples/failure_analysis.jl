# failure_analysis.jl — rank every thruster by how critical it is.
# For a redundant vehicle, single failures are survivable; the quad shows where
# a pair of failures actually costs a degree of freedom.
using ThrusterLab

println("BlueROV Heavy — single-thruster failures (over-actuated):")
report_failures(rank_failures(bluerov_vehicle()))

println("\nSimple Quad — pairs of failures (redundancy = 1):")
report_failures(rank_failures(quad_vehicle(); pairs=true))
