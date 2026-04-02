import math
from session_py import Line
from session_py import Plane
from session_py import Point
from session_py import Polyline
from session_py import TOLERANCE
from session_py import Xform
from session_py import Vector
from session_py.intersection import line_line, line_plane, polyline_plane, plane_plane_plane


def _project_point_to_plane(point, plane, direction):
    """Project a point onto a plane along a direction vector."""
    o = plane.origin
    n = plane.z_axis
    denom = direction[0]*n[0] + direction[1]*n[1] + direction[2]*n[2]
    if abs(denom) < 1e-14:
        return point
    t = ((o[0]-point[0])*n[0] + (o[1]-point[1])*n[1] + (o[2]-point[2])*n[2]) / denom
    return Point(point[0]+t*direction[0], point[1]+t*direction[1], point[2]+t*direction[2])


def _project_polyline_to_plane(polyline, plane, direction):
    """Project all points of a polyline onto a plane along a direction."""
    return Polyline([_project_point_to_plane(p, plane, direction) for p in polyline])


def _closest_point_on_plane(plane, point):
    """Orthogonal projection of a point onto a plane."""
    n = plane.z_axis
    o = plane.origin
    dist = (point[0]-o[0])*n[0] + (point[1]-o[1])*n[1] + (point[2]-o[2])*n[2]
    return Point(point[0]-dist*n[0], point[1]-dist*n[1], point[2]-dist*n[2])




class FloorBuilder:
    """Provides geometry parameters for building column heads and edge beams.

    Parameters — floor shape
        size        half-span of the square floor plan (mm)
        height      total depth from top slab to lowest point (mm)
        rise        parabola rise within that depth (mm)
        oculus      radius of the central opening (mm)
        thick       rib / shell thickness (mm)
        beam_w      edge beam width (mm)

    Parameters — column head
        column_head_scale          how far the head extends along each axis (mm)
        column_head_inclination    splay distance of the bottom ring (mm)
        head_h, head_b, head_o     head height, base, overhang (mm)
    """

    def __init__(
        self,
        # --- floor shape ---
        size=3000,
        height=650,
        rise=453,
        oculus=1000,
        thick=40,
        beam_w=200,
        # --- column head ---
        column_head_scale=460,
        column_head_inclination=180,
        head_h=500,
        head_b=100,
        head_o=150,
    ):
        # floor shape
        self.size = size
        self.height = height
        self.rise = rise
        self.static_h = height - rise
        self.oculus = oculus
        self.beam_w = beam_w
        self.thick = thick

        # column head
        self.head_h = head_h
        self.head_b = head_b
        self.head_o = head_o
        self._column_head_scale = column_head_scale
        self._column_head_inclination = column_head_inclination

        # caches (invalidated lazily)
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

    # ------------------------------------------------------------------ #
    #  Floor plan geometry (2D)
    # ------------------------------------------------------------------ #

    @property
    def oculus_points(self):
        """Four points whole distance is sqrt(2) * oculus"""
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
        """The main polygon of the floor."""
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
        """The bottom left corner."""
        return self.quarter_polygon.points[0]

    @property
    def boundary_points(self):
        """Points along quarter boundary"""
        q1 = self.quarter_polygon
        return [q1[i] for i in [1, 2, 3, 4]]
    
    def __offset_polygon(self, polygon, distance, baseplane=None):
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

    # ------------------------------------------------------------------ #
    #  Axes & ribs (3D curves along the vault)
    # ------------------------------------------------------------------ #

    @property
    def axes(self):
        """ Lines of the rib
        There is a small offset for the two central axes."""
        if self._axes is None:
            polygon = self.quarter_polygon
            offset_polygon_full = self.__offset_polygon(polygon, self.thick)
            offset_polygon_half = self.__offset_polygon(polygon, self.thick * 0.5)
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

    @property
    def corner_axis_point(self):
        """The point where the two boundary axes intersect."""
        return line_line(self.axes[0], self.axes[-1], TOLERANCE.absolute)

    @property
    def boundary_parabolas(self):
        """TEMP Two boundary parabolas for first and last axes."""
        if self._bound_parabolas is None:
            divisions = 7
            q1_parabolas = []

            for j in [0, len(self.axes) - 1]:
                if j == 0:
                    p0 = self.axes[j].start() + Vector(0, 0, -self.height)
                    p1 = self.axes[j].center() + Vector(0, 0, -self.static_h)
                    p2 = self.axes[j].end() + Vector(0, 0, -self.static_h)
                else:
                    p0 = self.axes[j].start() + Vector(0, 0, -self.static_h)
                    p1 = self.axes[j].center() + Vector(0, 0, -self.static_h)
                    p2 = self.axes[j].end() + Vector(0, 0, -self.height)

                pts = []
                for k in range(divisions):
                    t = k / (divisions - 1)
                    pt = p0 * (1 - t) ** 2 + p1 * 2 * (1 - t) * t + p2 * t ** 2
                    pts.append(Point(pt[0], pt[1], pt[2]))
                polyline = Polyline(pts)
                q1_parabolas.append(polyline)


            self._bound_parabolas = q1_parabolas
            # self.head_h *= 1.5
        return self._bound_parabolas

    @property
    def target_planes(self):
        """Planes from lines and zaxis."""
        if self._target_planes is None:
            axes = self.axes
            self._target_planes = [Plane.from_point_normal(axes[i].center(), Vector.z_axis().cross(axes[i].to_direction())) for i in range(len(axes))]
        return self._target_planes

    @property
    def rib_parabolas(self):
        """Four parabolas at each axis.."""
        if self._rib_parabolas is None:
            axes = self.axes
            proj_dir0 = Vector.z_axis().cross(axes[0].to_direction())
            proj_dir3 = Vector.z_axis().cross(axes[-1].to_direction())

            target_planes = self.target_planes
            parabola0 = self.boundary_parabolas[0]
            parabola1 = _project_polyline_to_plane(parabola0, target_planes[1], proj_dir0)
            parabola3 = self.boundary_parabolas[1]
            parabola2 = _project_polyline_to_plane(parabola3, target_planes[2], proj_dir3)
            parabola2.points.reverse()
            parabola3.points.reverse()

            self._rib_parabolas = [parabola0, parabola1, parabola2, parabola3]
        return self._rib_parabolas

    # ------------------------------------------------------------------ #
    #  Column head (transition from ribs to column)
    # ------------------------------------------------------------------ #

    def __compute_column_head_points(self, scale=None, inclination=None):
        """Compute corner profile points without caching."""
        scale = scale if scale is not None else self._column_head_scale
        inclination = inclination if inclination is not None else self._column_head_inclination

        points, axis_planes = [], []
        for i in range(4):
            v = self.axes[i].to_direction() * scale + self.corner_axis_point
            point = Point(v[0], v[1], v[2])
            points.append(point)
            plane = Plane.from_point_normal(point, self.axes[i].to_direction().cross(Vector.z_axis()))

            if i == 0 or i == 3:
                axis_planes.append(plane.translate_by_normal(self.thick * (-0.5 if i > 1 else 0.5)))
            else:
                axis_planes.append(plane.translate_by_normal(self.thick * (-1.0 if i > 1 else 1.0)))

        middle_line = Line.from_points(points[1], points[2])
        lp1 = line_plane(middle_line, axis_planes[1], False)
        lp2 = line_plane(middle_line, axis_planes[2], False)
        pts = [
            _closest_point_on_plane(axis_planes[0], lp1),
            lp1,
            lp2,
            _closest_point_on_plane(axis_planes[3], lp2),
        ]

        pts_offset = [Point(*(self.axes[i].to_direction() * inclination + pts[i])) for i in range(4)]
        pts_offset[0] = _closest_point_on_plane(axis_planes[0], pts_offset[1])
        pts_offset[3] = _closest_point_on_plane(axis_planes[3], pts_offset[2])
        pts_offset = [Point(*(-Vector.z_axis() * self.height + p)) for p in pts_offset]


        return pts, pts_offset

    @property
    def column_head_points(self):
        """Cached version using default parameters."""
        if self._corner_pts is None:
            self._corner_pts = self.__compute_column_head_points()
        return self._corner_pts

    def __compute_cut_planes(self, scale=None, inclination=None):
        """Compute cut planes without caching."""
        pts, pts_offset = self.__compute_column_head_points(scale, inclination)

        boundary_planes = [Plane.from_point_normal(self.axes[j].center(), Vector.z_axis().cross(self.axes[j].to_direction())) for j in range(3, len(self.axes) - 1)]
        offset_boundary_planes = [plane.translate_by_normal(self.thick * 0.5) for plane in boundary_planes]
        normals = [-(pts[i + 1] - pts[i]).cross(pts_offset[i] - pts[i]) for i in range(3)]
        corner_planes = [Plane.from_point_normal((pts[i] + pts[i + 1]) * 0.5, normals[i]) for i in range(3)]
        return offset_boundary_planes + corner_planes

    @property
    def cut_planes(self):
        """Cached version using default parameters."""
        if self._cut_planes is None:
            self._cut_planes = self.__compute_cut_planes()
        return self._cut_planes

    def set_column_head_params(self, scale=None, inclination=None):
        """Update parameters and invalidate caches."""
        if scale is not None:
            self._column_head_scale = scale
        if inclination is not None:
            self._column_head_inclination = inclination
        self._corner_pts = None
        self._cut_planes = None

    # ------------------------------------------------------------------ #
    #  Corner block & end planes
    # ------------------------------------------------------------------ #

    @property
    def top_corner_block_points(self):
        """Points defining the top block of the column head."""
        if self._top_corner_block_points is None:
            result = polyline_plane(self.quarter_polygon, self.end_diagonal_plane)
            pts_list = result[0]
            p0 = pts_list[0]
            p1 = pts_list[1]
            if p1[1] < p0[1]:
                p1, p0 = p0, p1

            p2 = p1 + Vector(-1, 0, 0) * self.beam_w
            p3 = Point(-self.beam_w - self.size, -self.beam_w - self.size, 0)
            p4 = p0 + Vector(0, -1, 0) * self.beam_w
            self._top_corner_block_points = [p4, p0, p1, p2, p3]
        return self._top_corner_block_points

    @property
    def top_end_planes(self):
        """Planes at midpoints of corner block edges."""
        if self._top_end_planes is None:
            p0 = (self.top_corner_block_points[0] + self.top_corner_block_points[1]) * 0.5
            p1 = (self.top_corner_block_points[1] + self.top_corner_block_points[2]) * 0.5
            p2 = (self.top_corner_block_points[2] + self.top_corner_block_points[3]) * 0.5

            n0 = Vector.z_axis().cross(self.top_corner_block_points[1] - self.top_corner_block_points[0])
            n1 = Vector.z_axis().cross(self.top_corner_block_points[2] - self.top_corner_block_points[1])
            n2 = Vector.z_axis().cross(self.top_corner_block_points[3] - self.top_corner_block_points[2])

            self._top_end_planes = [
                Plane.from_point_normal(p0, n0),
                Plane.from_point_normal(p1, n1),
                Plane.from_point_normal(p2, n2),
            ]
        return self._top_end_planes

    @property
    def end_planes(self):
        """End planes for each rib (needed for column head)."""
        if self._end_planes is None:
            _ = self.rib_parabolas
            planes = []
            for i in range(4):
                idx = i if i < 3 else -1
                s = self.axes[idx].start()
                e = self.axes[idx].end()
                p0 = Point(s[0], s[1], 0)
                p1 = Point(e[0], e[1], 0)
                if i == 3:
                    p0, p1 = p1, p0
                planes.append(Plane.from_point_normal(p0, p0 - p1).translate_by_normal(-self.thick * 10 / math.sqrt(2)))
            self._end_planes = planes
        return self._end_planes

    @property
    def end_diagonal_plane(self):
        return Plane.from_point_normal(self.corner_point, Vector(-1, -1, 0)).translate_by_normal(-self.thick * 1.87)
