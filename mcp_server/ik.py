"""Inverse kinematics for a MeArm-geometry 4-DOF arm (3 position joints +
claw). Direct port of the official MeArm-Arduino solver
(github.com/MeArm/MeArm-Arduino, ik.cpp, MIT licensed, Nick Moriarty 2014),
adapted to Python with configurable link lengths.

Coordinate frame (same as the original): origin is directly above the base
rotation axis, at shoulder height. y = forward (mm), x = sideways (mm),
z = up from shoulder height (mm). Angles returned are in radians.

Geometry (MeArm v3.0 defaults, see MeArm/MeArm-Arduino Geometry.md):
    L1 = shoulder-to-elbow length
    L2 = elbow-to-wrist length
    L3 = base-to-shoulder forward offset + wrist-to-claw-tip forward offset
         (combined, since both point the same direction)
These are close to correct for AliExpress MeArm clones (same laser-cut
design) but should be verified against the physical arm if precision
matters - see README calibration section.
"""
import math
from dataclasses import dataclass

L1 = 80.0
L2 = 80.0
L3 = 22.0


@dataclass
class JointAngles:
    base: float      # a0, radians
    shoulder: float  # a1, radians
    elbow: float     # a2, radians


def cart2polar(a: float, b: float) -> tuple[float, float]:
    r = math.sqrt(a * a + b * b)
    if r == 0:
        return 0.0, 0.0
    c = max(-1.0, min(1.0, a / r))
    s = max(-1.0, min(1.0, b / r))
    theta = math.acos(c)
    if s < 0:
        theta *= -1
    return r, theta


def cosangle(opp: float, adj1: float, adj2: float) -> float | None:
    """Law of cosines: angle between adj1 and adj2, given the opposite side."""
    den = 2 * adj1 * adj2
    if den == 0:
        return None
    c = (adj1 * adj1 + adj2 * adj2 - opp * opp) / den
    if c > 1 or c < -1:
        return None
    return math.acos(c)


def solve(x: float, y: float, z: float,
          l1: float = L1, l2: float = L2, l3: float = L3) -> JointAngles | None:
    """Solve for joint angles reaching (x, y, z). Returns None if unreachable."""
    r, th0 = cart2polar(y, x)
    r -= l3  # account for the wrist/claw forward offset

    R, ang_p = cart2polar(r, z)

    b = cosangle(l2, l1, R)
    if b is None:
        return None
    c = cosangle(R, l1, l2)
    if c is None:
        return None

    a0 = th0
    a1 = ang_p + b
    a2 = c + a1 - math.pi

    return JointAngles(base=a0, shoulder=a1, elbow=a2)


def forward(angles: JointAngles, l1: float = L1, l2: float = L2, l3: float = L3) -> tuple[float, float, float]:
    """Forward kinematics, for sanity-checking solve() round-trips.

    Note: a2 (elbow) is an ABSOLUTE angle from horizontal, not relative to
    a1 - MeArm's parallelogram linkage mechanically keeps the forearm's
    angle independent of the shoulder's, so the elbow servo command maps
    directly to the forearm's angle in space.
    """
    a0, a1, a2 = angles.base, angles.shoulder, angles.elbow
    # elbow position in the arm plane, relative to shoulder
    ex = l1 * math.cos(a1)
    ez = l1 * math.sin(a1)
    # wrist position in the arm plane (a2 is absolute, not relative to a1)
    wx = ex + l2 * math.cos(a2)
    wz = ez + l2 * math.sin(a2)
    r = wx + l3
    x = r * math.sin(a0)
    y = r * math.cos(a0)
    return x, y, wz
