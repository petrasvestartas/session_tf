from session_py import Line
from session_py import Plane
from session_py import Point
from session_py import Polyline # Need polyline offset
from session_py import TOLERANCE
from session_py import Xform
from session_py import Vector
from session_py import NurbsCurve
from session_py.intersection import line_line, line_plane, polyline_plane, plane_plane_plane




class FloorBuilder:
    """Provides geometry parameters for building column heads and edge beams."""

    def __init__(self, size=3000, height=650, rise=453, oculus=1000, thick=40, beam_w=200, column_head_scale=460, column_head_inclination=180, head_h=500, head_b=100, head_o=150):
        self.size = size
        self.height = height
        self.rise = rise
        self.static_h = height - rise
        self.oculus = oculus
        self.beam_w = beam_w
        self.thick = thick
        self.head_h = head_h
        self.head_b = head_b
        self.head_o = head_o
        self._oculus_pts = None
        self._q1_poly = None
        self._axes = None
        self._bound_parabolas = None
        self._rib_parabolas = None
        self._target_planes = None
        self._cut_planes = None
        self._corner_pts = None
        self._end_planes = None
        self._top_end_planes = None
        self._top_corner_block_points = None
        self._column_head_scale = column_head_scale
        self._column_head_inclination = column_head_inclination

    @property
    def oculus_points(self):
        if self._oculus_pts is None:
            self._oculus_pts = [
                Point(0, -self.oculus, 0),
                Point(self.oculus, 0, 0),
                Point(0, self.oculus, 0),
                Point(-self.oculus, 0, 0),
            ]
        return self._oculus_pts

    @property
    def quarter_polygon(self):
        if self._q1_poly is None:
            self._q1_poly = Polyline([
                Point(-self.size, -self.size, 0), # 0
                Point(0, -self.size, 0), # 1
                self.oculus_points[0], # 2
                self.oculus_points[3], # 3
                Point(-self.size, 0, 0), # 4
                Point(-self.size, -self.size, 0), # 0
            ])
        return self._q1_poly

    @property
    def corner_point(self):
        return self.quarter_polygon.points[0]

    @property
    def boundary_points(self):
        """Points along quarter boundary: edge -> oculus -> oculus -> edge."""
        q1 = self.quarter_polygon
        return [q1[i] for i in [1, 2, 3, 4]]
    
    @staticmethod
    def offset_polygon(polygon, distance, baseplane=None):
        """Offset a polygon by a distance.

        Parameters
        ----------
        polygon : :class:`compas.geometry.Polygon` or list
            Polygon to offset.
        distance : float
            Offset distance (positive = outward).
        baseplane : :class:`compas.geometry.Plane`, optional
            Reference plane. Defaults to world XY.

        Returns
        -------
        :class:`compas.geometry.Polygon`
            Offset polygon.
        """
        if baseplane is None:
            baseplane = Plane.xy_plane()
    
        planes = []
        pts = list(polygon) #  + [polygon[0]]
        for line in Polyline(pts).lines:
            zaxis = Vector.z_axis().cross(line.to_vector())
            zaxis.normalize_self()
            planes.append(Plane.from_point_normal(zaxis * distance + line.center(), zaxis))

        points = []
        for i in range(len(planes)):
            a = planes[(len(planes) + i - 1) % len(planes)]
            b = planes[i]
            c = baseplane
            result = plane_plane_plane(a, b, c)
            if not result:
                raise Exception(f"No intersection at offset_polygon. Index: {i}")
            points.append(result)

        return Polyline(points + [points[0]])

    @property
    def axes(self):
        if self._axes is None:
            polygon = self.quarter_polygon
            offset_polygon_full = self.offset_polygon(polygon, self.thick)
            offset_polygon_half = self.offset_polygon(polygon, self.thick * 0.5)
            axes_lines = list(offset_polygon_half.lines)

            corner = offset_polygon_half.points[0]
            axes_lines.insert(1, Line.from_points(corner, offset_polygon_full.points[2]))
            axes_lines.insert(2, Line.from_points(corner, offset_polygon_full.points[3]))

            extended_axes = []
            for j in range(len(axes_lines)):
                p0, p1 = axes_lines[j].start(), axes_lines[j].end()
                direction = (p1 - p0).normalized()
                line = Line.from_points(p1 + direction * self.thick * 4, p0 + -direction * self.thick * 4)

                intersection_points = []
                for k in range(len(polygon)):
                    seg = Line.from_points(polygon[k], polygon[(k + 1) % len(polygon)])
                    pt = line_line(line, seg, TOLERANCE.absolute)
                    if pt is not None:
                        intersection_points.append(pt)

                line = Line.from_points(intersection_points[0], intersection_points[1])
                ax_start = axes_lines[j].start()
                d0 = ax_start.distance(line.start())
                d1 = ax_start.distance(line.end())
                if d0 > d1:
                    line = Line.from_points(line.end(), line.start())
                extended_axes.append(line)

            # OPTIONAL: Move axes outward a bit to avoid double cut on the plate
            direction0 = Vector.z_axis().cross(extended_axes[1].to_direction()).normalized()*self.thick*0.5
            extended_axes[1] -= direction0
            direction1 = Vector.z_axis().cross(extended_axes[2].to_direction()).normalized()*self.thick*0.5
            extended_axes[2] += direction1
            self._axes = extended_axes

        return self._axes

    # @property
    # def corner_axis_point(self):
    #     return line_line(self.axes[0], self.axes[-1])[0]

    # @property
    # def boundary_parabolas(self):
    #     """Two boundary parabolas for Q1 (first and last axes)."""
    #     if self._bound_parabolas is None:
    #         divisions = 7
    #         q1_parabolas = []

    #         for j in [0, len(self.axes) - 1]:
    #             if j == 0:
    #                 p0 = Vector(0, 0, -self.height) + self.axes[j].start
    #                 p1 = Vector(0, 0, -self.static_h) + self.axes[j].midpoint
    #                 p2 = Vector(0, 0, -self.static_h) + self.axes[j].end
    #             else:
    #                 p0 = Vector(0, 0, -self.static_h) + self.axes[j].start
    #                 p1 = Vector(0, 0, -self.static_h) + self.axes[j].midpoint
    #                 p2 = Vector(0, 0, -self.height) + self.axes[j].end

    #             bezier = BezierCurve.quadratic_points(p0, p1, p2, divisions)
    #             # # self.head_h = abs(bezier[-2][2]) * 0.845 - 3.5
    #             # self.head_h = abs(bezier[-2][2]) * 0.84 - 3.5
    #             # self.head_h = 500
                
    #             # # self.head_h += 100
                
    #             q1_parabolas.append(bezier)

    #         self._bound_parabolas = q1_parabolas
    #         # self.head_h *= 1.5
    #     return self._bound_parabolas

    # @property
    # def target_planes(self):
    #     """Planes perpendicular to axes[0:4], at axis start points."""
    #     if self._target_planes is None:
    #         axes = self.axes
    #         self._target_planes = [Plane(axes[i].start, Vector.z_axis().cross(axes[i].direction)) for i in range(4)]
    #     return self._target_planes

    # @property
    # def rib_parabolas(self):
    #     """Four rib parabolas for Q1."""
    #     if self._rib_parabolas is None:
    #         axes = self.axes
    #         proj_dir0 = Vector.z_axis().cross(axes[0].direction)
    #         proj_dir3 = Vector.z_axis().cross(axes[-1].direction)

    #         target_planes = self.target_planes
    #         xform10 = Projection.from_plane_and_direction(target_planes[1], proj_dir0)
    #         xform20 = Projection.from_plane_and_direction(target_planes[2], proj_dir3)

    #         parabola0 = self.boundary_parabolas[0]
    #         parabola1 = parabola0.transformed(xform10)
    #         parabola3 = self.boundary_parabolas[1]
    #         parabola2 = parabola3.transformed(xform20)
    #         parabola2.points.reverse()
    #         parabola3.points.reverse()

    #         self._rib_parabolas = [parabola0, parabola1, parabola2, parabola3]
    #     return self._rib_parabolas
    
    # def compute_column_head_points(self, scale=None, inclination=None):
    #     """Compute corner profile points without caching.

    #     Parameters
    #     ----------
    #     scale : float, optional
    #         Defaults to self._column_head_scale.
    #     inclination : float, optional
    #         Defaults to self._column_head_inclination.

    #     Returns
    #     -------
    #     tuple[list[Point], list[Point]]
    #         (pts, pts_offset)
    #     """
    #     scale = scale if scale is not None else self._column_head_scale
    #     inclination = inclination if inclination is not None else self._column_head_inclination

    #     # Find corner intersection and create 4 points/planes along axes
    #     points, axis_planes = [], []
    #     for i in range(4):
    #         point = Point(*(self.axes[i].direction * scale + self.corner_axis_point))
    #         points.append(point)
    #         plane = Plane(point, self.axes[i].direction.cross(Vector.z_axis()))

    #         # OPTIONAL: second check is need when axis are offset:
    #         if i == 0 or i == 3:
    #             axis_planes.append(plane.offset(self.thick * (-0.5 if i > 1 else 0.5)))
    #         else:
    #             axis_planes.append(plane.offset(self.thick * (-1.0 if i > 1 else 1.0)))

    #     # Find corner profile points via middle_line intersections
    #     middle_line = Line(points[1], points[2])
    #     pts = [
    #         axis_planes[0].closest_point(Point(*line_plane(middle_line, axis_planes[1]))),
    #         Point(*line_plane(middle_line, axis_planes[1])),
    #         Point(*line_plane(middle_line, axis_planes[2])),
    #         axis_planes[3].closest_point(Point(*line_plane(middle_line, axis_planes[2]))),
    #     ]

    #     # Offset points: move along axis direction, then down by height
    #     pts_offset = [self.axes[i].direction * inclination + pts[i] for i in range(4)]
    #     pts_offset[0] = axis_planes[0].closest_point(pts_offset[1])
    #     pts_offset[3] = axis_planes[3].closest_point(pts_offset[2])
    #     pts_offset = [-Vector.z_axis() * self.height + p for p in pts_offset]

    #     return pts, pts_offset

    # @property
    # def column_head_points(self):
    #     """Cached version using default parameters."""
    #     if self._corner_pts is None:
    #         self._corner_pts = self.compute_column_head_points()
    #     return self._corner_pts

    # def compute_cut_planes(self, scale=None, inclination=None):
    #     """Compute cut planes without caching.

    #     Parameters
    #     ----------
    #     scale : float, optional
    #     inclination : float, optional

    #     Returns
    #     -------
    #     list[Plane]
    #         6 planes: 3 boundary offsets + 3 corner planes.
    #     """
    #     pts, pts_offset = self.compute_column_head_points(scale, inclination)

    #     # Build cut planes: 3 boundary offsets + 3 corner planes
    #     boundary_planes = [Plane(self.axes[j].midpoint, Vector.z_axis().cross(self.axes[j].direction)) for j in range(3, len(self.axes) - 1)]
    #     offset_boundary_planes = [plane.offset(self.thick * 0.5) for plane in boundary_planes]
    #     normals = [-(pts[i + 1] - pts[i]).cross(pts_offset[i] - pts[i]) for i in range(3)]
    #     corner_planes = [Plane((pts[i] + pts[i + 1]) * 0.5, normals[i]) for i in range(3)]
    #     return offset_boundary_planes + corner_planes

    # @property
    # def cut_planes(self):
    #     """Cached version using default parameters."""
    #     if self._cut_planes is None:
    #         self._cut_planes = self.compute_cut_planes()
    #     return self._cut_planes

    # def set_column_head_params(self, scale=None, inclination=None):
    #     """Update parameters and invalidate caches.

    #     Parameters
    #     ----------
    #     scale : float, optional
    #         New column head scale value.
    #     inclination : float, optional
    #         New column head inclination value.
    #     """
    #     if scale is not None:
    #         self._column_head_scale = scale
    #     if inclination is not None:
    #         self._column_head_inclination = inclination
    #     self._corner_pts = None
    #     self._cut_planes = None

    # @property
    # def top_corner_block_points(self):
    #     """Points defining the top block of the column head."""
    #     if self._top_corner_block_points is None:
    #         polyline = Polyline(self.quarter_polygon.points+[self.quarter_polygon.points[0]])
    #         result = polyline_plane(polyline, self.end_diagonal_plane, 2)
    #         p0 = Point(*result[0])
    #         p1 = Point(*result[1])
    #         if p1[1] < p0[1]:
    #             p1, p0 = p0, p1
            
    #         p2 = p1 + Vector(-1, 0, 0) * self.beam_w
    #         p3 = Point(-self.beam_w-self.size, -self.beam_w-self.size, 0)
    #         p4 = p0 + Vector(0, -1, 0) * self.beam_w
    #         self._top_corner_block_points = [p4, p0, p1, p2, p3]
    #     return self._top_corner_block_points
    
    # @property
    # def top_end_planes(self):
    #     if self._top_end_planes is None:

    #         p0 = (self.top_corner_block_points[0] + self.top_corner_block_points[1]) * 0.5
    #         p1 = (self.top_corner_block_points[1] + self.top_corner_block_points[2]) * 0.5
    #         p2 = (self.top_corner_block_points[2] + self.top_corner_block_points[3]) * 0.5
            
    #         n0 = Vector.z_axis().cross(self.top_corner_block_points[1] - self.top_corner_block_points[0])
    #         n1 = Vector.z_axis().cross(self.top_corner_block_points[2] - self.top_corner_block_points[1])
    #         n2 = Vector.z_axis().cross(self.top_corner_block_points[3] - self.top_corner_block_points[2])

    #         plane0 = Plane(p0, n0)
    #         plane1 = Plane(p1, n1)
    #         plane2 = Plane(p2, n2)
        
    #         self._top_end_planes = [plane0, plane1, plane2]
    #     return self._top_end_planes

    # @property
    # def end_planes(self):

    #     import math
    #     """End planes for each rib (needed for column head)."""


    #     if self._end_planes is None:
    #         # NOTE: Accessing rib_parabolas triggers boundary_parabolas which computes head_h.
    #         # Do NOT remove - head_h computation depends on this side effect.
    #         _ = self.rib_parabolas  # noqa: F841
    #         planes = []
    #         for i in range(4):
    #             id = i if i < 3 else -1
    #             p0 = Point(self.axes[id].start[0], self.axes[id].start[1], 0)
    #             p1 = Point(self.axes[id].end[0], self.axes[id].end[1], 0)
    #             if i == 3:
    #                 p0, p1 = p1, p0
    #             planes.append(Plane(p0, p0 - p1).offset(-self.thick * 10/math.sqrt(2)))
    #         self._end_planes = planes
    #     return self._end_planes

    # @property
    # def end_diagonal_plane(self):
    #     # plane = Plane(self.corner_point, Vector(-1,-1,0)).offset(-self.thick * 2.27 )
    #     plane = Plane(self.corner_point, Vector(-1,-1,0)).offset(-self.thick * 1.87)
    #     return plane
    
