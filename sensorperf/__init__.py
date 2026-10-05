"""
sensorperf: code for the VSX3000 Resolution, Area-Fidelity, Detectability, and
Noise Characterization Procedure.

Part I of the procedure (data acquisition) is supported by the acquisition and
cli modules: station planning with logged randomization, the capture manifest,
the quick-look check, and the robot-to-sensor registration solve. Part II (data
analysis) is implemented in the analysis modules, one per procedure letter
(A noise, B resolution, C area, D detection, E boundary bias). The simulate
module renders synthetic captures of the same targets so every tool and
analysis can be exercised without a sensor.

Every arbitrary constant of the procedure is a named parameter in
sensorperf.parameters. See docs/design/code_design.md for the module map, the
frame conventions, and the interfaces.
"""
