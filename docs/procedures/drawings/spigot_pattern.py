"""Shared mounting constants for the VSX3000 performance-testing fixtures.

One steel spigot (PT-02) bolts to the BACK face of whichever plate carries the
target to the robot, and PT-03 standoffs hold the front part above the back
plate.  The drawings of the carrying plates (PT-04 to PT-07) take the hole
pattern from this module so that the spigot, the plates and the standoffs
cannot disagree.

Target frame: origin at the plate center, x to the right and y up when the plate
faces the sensor.  The spigot pattern is seen from the BACK, so it is mirrored in
the front view (the hole angles below are measured in the target frame, from +x,
counterclockwise seen from the sensor).
"""

# ---------------------------------------------------------------------------
# Spigot interface (PT-02)
# ---------------------------------------------------------------------------
SPIGOT_PCD_MM = 60.0  # pitch circle diameter of the four spigot screws
SPIGOT_SCREW = "M5"  # spigot screw thread
SPIGOT_SCREW_DIAMETER_MM = 5.0  # nominal diameter of that thread
SPIGOT_HOLE_ANGLES_DEG = (45.0, 135.0, 225.0, 315.0)  # screw angles from +x
SPIGOT_FLANGE_DIAMETER_MM = 80.0  # diameter of the spigot flange (on the back face)
SPIGOT_DOWEL_DIRECTION = "+y"  # the spigot's radial orientation dowel points along +y

# Engraved orientation arrow on the back face, pointing along SPIGOT_DOWEL_DIRECTION.
# (Chosen here, not fixed by the brief: the arrow starts just outside the flange so
# it stays visible with the spigot mounted.)
ORIENTATION_MARK_WIDTH_MM = 6.0  # arrow width (the width of its head)
ORIENTATION_MARK_DEPTH_MM = 2.0  # engraving depth
ORIENTATION_MARK_LENGTH_MM = 20.0  # arrow length, tail to tip
ORIENTATION_MARK_CLEARANCE_MM = 4.0  # gap between the flange edge and the arrow tail

# ---------------------------------------------------------------------------
# Standoffs (PT-03)
# ---------------------------------------------------------------------------
STANDOFF_STUD = "M5"  # male stud thread at both ends
STANDOFF_STUD_BACK_LENGTH_MM = 8.0  # stud length at the back-plate end (through-tapped 8 mm plate, flush with its back face)
STANDOFF_BODY_DIAMETER_MM = 10.0  # body diameter; body length equals the gap

# Tapped holes that receive a standoff stud or a bracket screw.
M5_PITCH_MM = 0.8  # M5 coarse pitch
# The stud at the FRONT end of a standoff is shorter than the 8 mm back-end stud: in a 6 mm
# front plate (M5 through-tapped) it ends 6 - 5.5 = 0.5 mm below the front face.
STANDOFF_STUD_FRONT_LENGTH_MM = 5.5  # stud length at the front-part end (back-plate end: STANDOFF_STUD_BACK_LENGTH_MM)
