"""Floor-plane fit and sensor pose from one 8x8 (or 4x4) depth frame.

The VL53L7 reports distance along the optical axis (z), not along each zone's
ray: on real data a plane fits 2-4x better that way. A zone's 3D point is then
P = z * (tan ax, tan ay, 1), and for points on a plane c.P = 1 the quantity
1/z is linear in (tan ax, tan ay, 1), so the fit is a linear least squares
with iterative outlier rejection (objects, drop-offs).

ZONE_DEG is the angular pitch of one zone: 60 deg / 8 per the datasheet. It
cannot be recovered from a plane alone (it just scales the fitted tilt), so
calibrate it with the sensor at a measured height and tilt if pose accuracy
matters.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

ZONE_DEG_8X8 = 7.5


def ray_tans(side: int = 8, zone_deg: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    """tan of each zone centre's angle off axis: (tx by column, ty by row), shape (side, side)."""
    if zone_deg is None:
        zone_deg = ZONE_DEG_8X8 * 8 / side
    a = np.tan((np.arange(side) - (side - 1) / 2) * np.radians(zone_deg))
    ty, tx = np.meshgrid(a, a, indexing="ij")
    return tx, ty


def points(z: np.ndarray, zone_deg: float | None = None) -> np.ndarray:
    """3D points in the sensor frame, shape (side, side, 3); NaN where z is NaN."""
    tx, ty = ray_tans(z.shape[0], zone_deg)
    return np.stack([tx * z, ty * z, z], -1)


@dataclass
class FloorFit:
    coef: np.ndarray  # plane c.P = 1 in the sensor frame
    inliers: np.ndarray  # bool (side, side)
    rms_mm: float  # depth residual on inliers

    @property
    def height_mm(self) -> float:
        """Sensor distance from the plane."""
        return float(1 / np.linalg.norm(self.coef))

    @property
    def up(self) -> np.ndarray:
        """Unit plane normal pointing from the floor towards the sensor."""
        return -self.coef / np.linalg.norm(self.coef)

    @property
    def tilt_deg(self) -> float:
        """Angle of the optical axis below the floor plane."""
        return float(np.degrees(np.arcsin(-self.up[2])))

    @property
    def roll_deg(self) -> float:
        """Rotation about the optical axis relative to the floor: 0 when the sensor
        rows are parallel to the floor. Rows run near (row 0) to far, so the
        floor normal projects onto +y in the image."""
        u = self.up
        return float(np.degrees(np.arctan2(u[0], u[1])))

    def heights(self, P: np.ndarray) -> np.ndarray:
        """Height of points above the floor, mm (negative = below)."""
        return self.height_mm + P @ self.up

    def floor_frame(self, P: np.ndarray) -> np.ndarray:
        """Points as (forward, left, up) mm, origin on the floor under the sensor."""
        u = self.up
        f = np.array([0.0, 0.0, 1.0]) - u[2] * u  # optical axis projected onto the floor
        f /= np.linalg.norm(f)
        left = np.cross(u, f)
        return np.stack([P @ f, P @ left, self.height_mm + P @ u], -1)


def fit_floor(z: np.ndarray, mask: np.ndarray | None = None, zone_deg: float | None = None,
              min_points: int = 8) -> FloorFit | None:
    """Robust plane fit to a depth frame (NaN = no return). None if too few points."""
    tx, ty = ray_tans(z.shape[0], zone_deg)
    A = np.stack([tx, ty, np.ones_like(tx)], -1)
    m = np.isfinite(z) & (z > 0)
    if mask is not None:
        m &= mask
    for _ in range(8):
        if m.sum() < min_points:
            return None
        coef, *_ = np.linalg.lstsq(A[m], 1 / z[m], rcond=None)
        with np.errstate(divide="ignore", invalid="ignore"):
            pred = 1 / (A @ coef)
            res = np.where(pred > 0, z - pred, np.inf)  # rays that never meet the plane
        mad = 1.4826 * np.median(np.abs(res[m])) + 1.0
        new = np.isfinite(z) & (np.abs(res) < max(3 * mad, 8.0))
        if mask is not None:
            new &= mask
        if (new == m).all():
            break
        m = new
    rms = float(np.sqrt(np.mean(res[m] ** 2))) if m.any() else float("nan")
    return FloorFit(coef, m, rms)
