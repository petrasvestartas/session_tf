"""Example: visualize all FloorBuilder public properties."""

from session_py.session import Session
from session_py.session_config import SESSION_CONFIG
from session_tf.floor import FloorBuilder
from session_compas.session import view



session = Session()

builder = FloorBuilder(
    size=3000,
    height=650,
    rise=453,
    oculus=1000,
    thick=40,
    beam_w=200,
)

# ------------------------------------------------------------------ #
#  Floor plan geometry (2D)
# ------------------------------------------------------------------ #

# Four points whose distance is sqrt(2) * oculus
session.set_layer("oculus_points")
for p in builder.oculus_points:
    session.add_point(p)

# The main polygon of the floor
session.set_layer("quarter_polygon")
session.add_polyline(builder.quarter_polygon)

# The bottom left corner
session.set_layer("corner_point")
session.add_point(builder.corner_point)

# Points along quarter boundary
session.set_layer("boundary_points")
for p in builder.boundary_points:
    session.add_point(p)

# ------------------------------------------------------------------ #
#  Axes & ribs (3D curves along the vault)
# ------------------------------------------------------------------ #

# Lines of the ribs, with a small offset for the two central axes
session.set_layer("axes")
for line in builder.axes:
    session.add_line(line)

# The point where the two boundary axes intersect
session.set_layer("corner_axis_point")
session.add_point(builder.corner_axis_point)

# Two boundary parabolas for first and last axes
session.set_layer("boundary_parabolas")
for polyline in builder.boundary_parabolas:
    session.add_polyline(polyline)

# Planes from lines and z-axis
session.set_layer("target_planes")
for plane in builder.target_planes:
    plane.width = 200
    session.add_plane(plane)

# Four parabolas at each axis
session.set_layer("rib_parabolas")
for polyline in builder.rib_parabolas:
    session.add_polyline(polyline)

# ------------------------------------------------------------------ #
#  Column head (transition from ribs to column)
# ------------------------------------------------------------------ #

# Top ring and bottom ring of the column head
session.set_layer("column_head_points")
pts, pts_bottom = builder.column_head_points
for i in range(len(pts)):
    session.add_point(pts[i])
    session.add_point(pts_bottom[i])

# Cut planes for the column head
session.set_layer("cut_planes")
for plane in builder.cut_planes:
    plane.width = 200
    session.add_plane(plane)

# ------------------------------------------------------------------ #
#  Corner block & end planes
# ------------------------------------------------------------------ #

# Points defining the top block of the column head
session.set_layer("top_corner_block_points")
for p in builder.top_corner_block_points:
    session.add_point(p)

# Planes at midpoints of corner block edges
session.set_layer("top_end_planes")
for plane in builder.top_end_planes:
    plane.width = 200
    session.add_plane(plane)

# End planes for each rib
session.set_layer("end_planes")
for plane in builder.end_planes:
    plane.width = 200
    session.add_plane(plane)

# Diagonal cut plane at the corner
session.set_layer("end_diagonal_plane")
plane = builder.end_diagonal_plane
plane.width = 200
session.add_plane(plane)

SESSION_CONFIG.scale_factor = 0.001  # mm → m
view(session)
