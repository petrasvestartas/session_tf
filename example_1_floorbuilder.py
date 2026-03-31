"""Example: create a box mesh using session_py."""

from session_py.session import Session
from session_py.session_config import SESSION_CONFIG
from session_py import Mesh
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

# for p in builder.oculus_points:
#     session.add_point(p)

session.add_polyline(builder.quarter_polygon)
# session.add_point(builder.corner_point)

for p in builder.boundary_points:
    session.add_point(p)

# session.add_polyline(builder.offset_polygon(builder.quarter_polygon, builder.thick))

for line in builder.axes:
    session.add_line(line)

# box = Mesh.create_box(2.0, 3.0, 1.5)
# session.add_mesh(box)

session.pb_dump("build_model.pb")
SESSION_CONFIG.scale_factor = 0.001  # mm → m
view("build_model.pb")

