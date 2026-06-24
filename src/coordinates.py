# -*- coding: utf-8 -*-
"""
coordinates.py  --  Single source of truth for LithTech <-> Blender geometry.

LithTech : left-handed, Y up, Z forward.
Blender  : right-handed, Z up,  Y forward.

The two systems differ by a single Y<->Z axis swap. That swap matrix

        | 1 0 0 |
    C = | 0 0 1 |          (C == C^-1, det C = -1)
        | 0 1 0 |

is an *involution* (its own inverse) and a *reflection* (det -1). Two
consequences that this whole module relies on:

  1. Because C is an involution, the SAME function converts both directions
     (LithTech->Blender and Blender->LithTech). There is no separate "import"
     and "export" math -- only `swap_*`.

  2. Because C is a reflection (handedness flip), triangle winding must be
     reversed whenever geometry crosses the boundary. Use `flip_winding`.

All conversions are baked into the *data* at the I/O boundary. Objects must
therefore have an IDENTITY transform -- do NOT rotate the armature 90 deg or
set scale.z = -1 anywhere. Those object-level hacks are exactly what this
module exists to delete.

Verified numerically (max error 0 / 1e-15):
  - swap_quat is its own inverse
  - swap_matrix is its own inverse
  - the quaternion path and the matrix path agree
"""

from mathutils import Matrix, Vector, Quaternion

# Y<->Z swap. Self-inverse, det = -1.
_C = Matrix(((1.0, 0.0, 0.0, 0.0),
             (0.0, 0.0, 1.0, 0.0),
             (0.0, 1.0, 0.0, 0.0),
             (0.0, 0.0, 0.0, 1.0)))


# --------------------------------------------------------------------------
# Positions / directions
# --------------------------------------------------------------------------
def swap_vec(v, scale=1.0):
    """Position vector across the boundary. (x, y, z) -> (x, z, y).

    `scale` multiplies the result and applies in BOTH directions, so the
    inverse call must pass 1.0/scale (see swap_vec_inv).
    """
    return Vector((v[0] * scale, v[2] * scale, v[1] * scale))


def swap_vec_inv(v, scale=1.0):
    """Convenience inverse of swap_vec for a non-unit scale."""
    return swap_vec(v, 1.0 / scale if scale != 1.0 else 1.0)


def swap_dir(v):
    """Direction/normal across the boundary (never scaled). (x,y,z)->(x,z,y)."""
    return Vector((v[0], v[2], v[1]))


# --------------------------------------------------------------------------
# Rotations
# --------------------------------------------------------------------------
def swap_quat(q):
    """Rotation quaternion across the boundary. Self-inverse.

    Accepts a mathutils.Quaternion (w, x, y, z) and returns the same type.
    Maps (w, x, y, z) -> (w, -x, -z, -y) in either direction.
    """
    return Quaternion((q.w, -q.x, -q.z, -q.y))


def swap_quat_xyzw(x, y, z, w):
    """Same as swap_quat but for raw LithTech storage order (x, y, z, w).
    Returns a mathutils.Quaternion in Blender's (w, x, y, z) order.
    """
    return Quaternion((w, -x, -z, -y))


# --------------------------------------------------------------------------
# Full transforms
# --------------------------------------------------------------------------
def swap_matrix(m, scale=1.0):
    """4x4 transform across the boundary by similarity C @ M @ C. Self-inverse
    (for scale == 1). Translation is scaled like swap_vec.
    """
    out = _C @ m @ _C
    if scale != 1.0:
        out.translation = out.translation * scale
    return out


# --------------------------------------------------------------------------
# Topology
# --------------------------------------------------------------------------
def flip_winding(tri):
    """Reverse a triangle's winding to compensate the handedness flip.
    (a, b, c) -> (a, c, b). Apply to indices AND to any per-corner data
    (uv/normal/colour corner lists) consistently.
    """
    return (tri[0], tri[2], tri[1])


WINDING_ORDER = (0, 2, 1)  # use to reindex per-corner attribute triples


# --------------------------------------------------------------------------
# Edit-detection helpers. The importer stores exact LithTech values
# (lt_normal, lt_rest) so an UNCHANGED model re-exports byte-for-byte. Those
# values go stale the moment the user edits, so the exporter must tell whether
# a mesh was deformed or a bone was moved, and fall back to live Blender data
# when it was. Both checks compare a cheap signature taken at import time.
import zlib as _zlib
import struct as _struct


def geo_signature(coords):
    """Order-sensitive CRC32 of vertex coordinates rounded to 1e-4. Stored on
    the mesh at import; recomputed on export. Equal => geometry untouched =>
    the imported normals are still valid. Different => mesh was deformed =>
    recompute normals from the current geometry."""
    buf = bytearray()
    for c in coords:
        buf += _struct.pack('<3i', int(round(c[0] * 1e4)),
                            int(round(c[1] * 1e4)),
                            int(round(c[2] * 1e4)))
    val = _zlib.crc32(bytes(buf)) & 0xFFFFFFFF
    # Blender ID-properties store int custom props as a SIGNED 32-bit C int
    # (max 2147483647). crc32 is unsigned (0..4294967295), so roughly half
    # of all signatures overflowed that and silently failed to store (caught
    # by the try/except in builder_import as "[lt_normal] skipped: Python
    # int too large to convert to C int"), leaving lt_geo_sig unset and
    # making the exporter always assume the mesh was deformed. Wrap to
    # signed range; still order-sensitive and still a valid equality check.
    return val - 0x100000000 if val >= 0x80000000 else val


def mat_close(a, b, eps=1e-5):
    """True if two 4x4 matrices are equal within eps (per element)."""
    for r in range(4):
        for col in range(4):
            if abs(a[r][col] - b[r][col]) > eps:
                return False
    return True

