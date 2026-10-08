"""Turn the flood model's per-frame output into a flood-risk map.

The flood model (`detect_flood` in basilrari/edge-ai-modelserver, reached
through edge-ai-gateway as `flood_seg`/`flood_class`) does not return its
segmentation mask. It returns the water fraction of each cell of a 4x4 grid
laid over the frame (`grid.cell_ratios`, row 0 = top of the image), or no
grid at all when the frame is dry or no cell reaches 0.3 water. Its own GPS
fields cannot be used: as of 2026-10-08 the model server's drone position is
hardcoded and its altitude is random, so every result is marked
"simulated". The drone's pose therefore comes from us (PX4 telemetry at
the moment the frame was requested), not from the response.

Camera model, matching the model server's own geolocation code
(core/gps_locator.py there): a pinhole camera with no distortion, GoPro
Linear 87 deg horizontal FOV at 1920x1080, mounted on a gimbal that keeps
it level - pointing straight down by default (pitch -90 deg) with the top
of the image toward the drone's nose - over flat ground. Drone roll and
pitch are ignored because the gimbal cancels them.

Each map cell (coverage.py's 1m grid in the shared frame: x north, y east)
whose centre lands inside the frame takes the water fraction of the image
grid cell it lands in. Repeated observations are averaged, so overlapping
frames taken from different places sharpen the 4x4 blocks into the real
shape of the water. The average is written out as the cell's risk, in
coverage.load_risk_map's format.
"""

import math

GRID_SIZE = 4
DEFAULT_HFOV_DEG = 87.0
DEFAULT_WIDTH = 1920
DEFAULT_HEIGHT = 1080
NADIR_PITCH_DEG = -90.0
# Below this mean water fraction a cell is left out of the written map
# (missing = risk 0 in coverage.py, so dropping it changes nothing).
MIN_WRITTEN_RISK = 0.01


class Camera:

    def __init__(self, hfov_deg=DEFAULT_HFOV_DEG, width=DEFAULT_WIDTH,
                 height=DEFAULT_HEIGHT, pitch_deg=NADIR_PITCH_DEG):
        self.width = width
        self.height = height
        self.fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        self.fy = self.fx
        self.cx = width / 2.0
        self.cy = height / 2.0
        self.pitch = math.radians(pitch_deg)

    def project(self, ground_xy, drone_xy, altitude_m, yaw_rad):
        """Pixel (u, v) at which the ground point (x north, y east) appears
        from a drone at drone_xy, altitude_m above the ground, heading
        yaw_rad (PX4 convention: 0 = north, positive toward east). None if
        the point is behind the camera."""
        dn = ground_xy[0] - drone_xy[0]
        de = ground_xy[1] - drone_xy[1]
        dd = altitude_m  # the ground is below the drone (NED: +down)
        # Into the drone's body axes: forward, right, down.
        fwd = dn * math.cos(yaw_rad) + de * math.sin(yaw_rad)
        right = -dn * math.sin(yaw_rad) + de * math.cos(yaw_rad)
        # Camera axes: optical axis pitched down from forward, image right =
        # body right, image down = optical x right.
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        depth = fwd * cp - dd * sp
        if depth <= 1e-9:
            return None
        img_down = fwd * sp + dd * cp
        return (self.cx + self.fx * right / depth,
                self.cy + self.fy * img_down / depth)

    def ground_point(self, pixel, drone_xy, altitude_m, yaw_rad):
        """Inverse of project(): the ground point (x north, y east) seen at
        pixel (u, v), or None if that pixel looks at or above the horizon."""
        x = (pixel[0] - self.cx) / self.fx
        y = (pixel[1] - self.cy) / self.fy
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        fwd = cp + y * sp
        down = -sp + y * cp
        if down <= 1e-9:
            return None
        t = altitude_m / down
        fwd, right = t * fwd, t * x
        return (drone_xy[0] + fwd * math.cos(yaw_rad) - right * math.sin(yaw_rad),
                drone_xy[1] + fwd * math.sin(yaw_rad) + right * math.cos(yaw_rad))

    def grid_cell(self, pixel, grid_size=GRID_SIZE):
        """(row, col) of the image grid cell containing pixel, or None if
        it is outside the frame."""
        if pixel is None:
            return None
        u, v = pixel
        if not (0 <= u < self.width and 0 <= v < self.height):
            return None
        return (int(v * grid_size // self.height),
                int(u * grid_size // self.width))

    def ground_half_extent(self, altitude_m):
        """Distance from the drone within which every visible ground point
        lies, nadir or not (bounds the cells observe() has to check)."""
        corner = math.hypot(self.width / 2.0 / self.fx,
                            self.height / 2.0 / self.fy)
        tilt = abs(self.pitch - math.radians(NADIR_PITCH_DEG))
        reach = math.atan(corner) + tilt
        if reach >= math.radians(80):
            reach = math.radians(80)  # horizon: cap instead of infinity
        return altitude_m * math.tan(reach)


def cell_ratios_from_response(response):
    """The 4x4 water fractions to use from one detect_flood response, or
    None if the response says nothing about this frame.

    - grid present: its cell_ratios.
    - no grid, classifier says Non-Flooded: dry everywhere.
    - no grid but Flooded: the model found water but no cell reached 0.3,
      so spread the whole-frame flood_ratio evenly - it is under 0.3 too.
    - error, or skipped because the model was busy: no observation.
    """
    if not response or response.get('error') or response.get('skipped'):
        return None
    grid = response.get('grid') or {}
    ratios = grid.get('cell_ratios')
    if ratios:
        return [[float(r) for r in row] for row in ratios]
    label = (response.get('classification') or {}).get('label')
    if label == 'Non-Flooded':
        return [[0.0] * GRID_SIZE for _ in range(GRID_SIZE)]
    if label == 'Flooded':
        ratio = float((response.get('segmentation') or {}).get(
            'flood_ratio', 0.0))
        return [[ratio] * GRID_SIZE for _ in range(GRID_SIZE)]
    return None


class FloodRiskAccumulator:

    def __init__(self, camera=None, cell_size=1.0):
        self.camera = camera or Camera()
        self.cell_size = cell_size
        self._sum = {}    # cell -> summed water fraction
        self._count = {}  # cell -> number of frames that saw it

    def observe(self, cell_ratios, drone_xy, altitude_m, yaw_rad):
        """Add one frame. Returns how many map cells it covered."""
        if cell_ratios is None or altitude_m <= 0:
            return 0
        grid_size = len(cell_ratios)
        reach = self.camera.ground_half_extent(altitude_m)
        cs = self.cell_size
        i0 = math.floor((drone_xy[0] - reach) / cs)
        i1 = math.floor((drone_xy[0] + reach) / cs)
        j0 = math.floor((drone_xy[1] - reach) / cs)
        j1 = math.floor((drone_xy[1] + reach) / cs)
        covered = 0
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                centre = ((i + 0.5) * cs, (j + 0.5) * cs)
                rc = self.camera.grid_cell(
                    self.camera.project(centre, drone_xy, altitude_m, yaw_rad),
                    grid_size)
                if rc is None:
                    continue
                cell = (i, j)
                self._sum[cell] = self._sum.get(cell, 0.0) + cell_ratios[rc[0]][rc[1]]
                self._count[cell] = self._count.get(cell, 0) + 1
                covered += 1
        return covered

    def risk(self):
        """cell -> mean observed water fraction (0-1)."""
        return {c: self._sum[c] / n for c, n in self._count.items()}

    def to_risk_map(self, source='detect_flood'):
        """JSON-ready dict in coverage.load_risk_map's format."""
        return {
            'cell_size': self.cell_size,
            'source': source,
            'cells': [[i, j, round(r, 3)]
                      for (i, j), r in sorted(self.risk().items())
                      if r >= MIN_WRITTEN_RISK],
            'observed_cells': len(self._count),
        }
