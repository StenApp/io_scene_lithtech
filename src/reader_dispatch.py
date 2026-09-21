# -*- coding: utf-8 -*-
"""
reader_dispatch.py  --  ONE job: look at a file, pick the right reader, return
the unified abc.py:Model (raw LithTech space). Nothing here knows about Blender
or coordinates; conversion happens later in builder_import.

Detection (deterministic, by header -- not the old try/except cascade):
    ABC       : starts with length-prefixed string "Header"    -> ABCModelReader
    ABC v6    : "Header" section's first field is the literal   -> NOT SUPPORTED, see below
                string "MonolithExport Model File v6" instead
                of a uint32 Version (9/10/11/12/13/108)
    LTB PC    : file_type(uint16)=1                            -> PCLTBModelReader
    LTB PS2   : file_type(uint16)=2                            -> PS2LTBModelReader
    LTB DHNP  : starts with length-prefixed string "LTBHeader" -> DHNPABCModelReader /
                (Die Hard: Nakatomi Plaza's own LTB wrapper)      DHNPD3DModelReader,
                depending on the wrapper's FileType (4=ABC body, 1=D3D-hybrid body)

ABC v6 (2026-09): this addon's own reader_abc_v6.py was removed. Reasons,
in short (see git history / project notes for the full investigation):
this addon has no writer that could preserve v6's per-vertex "mesh
deformation" (cloth) animation on export (neither exporter_lta.py nor the
LTA format as implemented here has any vertex-animation/shape-key concept),
so a from-scratch v6 reader here could only ever produce a static rest
pose, not working animations. Five-Damned-Dollarz/io_scene_lithtech
(github.com/Five-Damned-Dollarz/io_scene_lithtech) has a more complete v6
import/export pipeline including animated vertex deformation via Blender
shape keys -- use that addon for v6 (Blood 2 etc.) work instead.
detect_format() still recognises v6 files by header (so they fail loudly
and specifically here, instead of being silently misread as a different
ABC version); read_model() raises NotImplementedError for them.

Works both as an addon package member and standalone (path) via the import shim.
"""

import os
import struct

# The literal version string v6 stores where every other ABC version has a
# uint32 Version field -- kept here (not in a v6 reader module, which no
# longer exists) purely so detect_format() can still recognise and reject
# v6 files with a clear, specific error instead of misreading them as a
# different ABC version. See the module docstring above.
V6_VERSION_STRING = b'MonolithExport Model File v6'

# package member (addon) first, standalone (sys.path) as fallback
try:
    from .reader_abc_pc import ABCModelReader
    from .reader_ltb_pc import PCLTBModelReader
    from .reader_ltb_ps2 import PS2LTBModelReader
    from .reader_lta import LTAModelReader
    from .reader_ltb_dhnp import detect_dhnp, DHNPABCModelReader, DHNPD3DModelReader
except ImportError:
    from reader_abc_pc import ABCModelReader
    from reader_ltb_pc import PCLTBModelReader
    from reader_ltb_ps2 import PS2LTBModelReader
    from reader_lta import LTAModelReader
    from reader_ltb_dhnp import detect_dhnp, DHNPABCModelReader, DHNPD3DModelReader


def detect_format(path):
    """Return one of 'lta', 'abc', 'abc_v6', 'ltb_pc', 'ltb_ps2',
    'ltb_dhnp_abc', 'ltb_dhnp_d3d', or 'unknown'."""
    with open(path, 'rb') as f:
        head = f.read(64)
    # LTA is text starting with '(' (after optional BOM/whitespace)
    stripped = head.lstrip(b'\xef\xbb\xbf \t\r\n')
    if stripped[:1] == b'(' or os.path.splitext(path)[1].lower() == '.lta':
        return 'lta'
    if len(head) >= 8:
        n = struct.unpack_from('<H', head, 0)[0]
        if n == 6 and head[2:8] == b'Header':
            # Both normal ABC (v9-13/108) and v6 start with the "Header"
            # section name + a 4-byte next_section_offset. What comes
            # right after differs: normal ABC has a uint32 Version there;
            # v6 has a length-prefixed string literal instead. Peeking at
            # offset 12 as a uint16 "would-be string length" and checking
            # for the exact v6 marker string distinguishes them safely --
            # no valid Version value (9-13/108) collides with 28 (the
            # marker string's length).
            if len(head) >= 14:
                slen = struct.unpack_from('<H', head, 12)[0]
                if slen == len(V6_VERSION_STRING) and head[14:14 + slen] == V6_VERSION_STRING:
                    return 'abc_v6'
            return 'abc'
        if n == 9 and head[2:11] == b'LTBHeader':
            dhnp = detect_dhnp(path)
            if dhnp is not None:
                return dhnp
    if len(head) >= 2:
        ftype = struct.unpack_from('<H', head, 0)[0]
        if ftype == 1:
            return 'ltb_pc'
        if ftype == 2:
            return 'ltb_ps2'
    return {'.abc': 'abc'}.get(os.path.splitext(path)[1].lower(), 'unknown')


def read_model(path, **lta_options):
    """Detect format and return a populated abc.py:Model.

    lta_options (parse_lod_groups, parse_lod_recipe) only apply to .lta --
    the binary readers take no options and are called without them.
    """
    fmt = detect_format(path)
    if fmt == 'lta':
        return LTAModelReader().from_file(path, **lta_options)
    if fmt == 'abc':
        return ABCModelReader().from_file(path)
    if fmt == 'abc_v6':
        raise NotImplementedError(
            "%s is an ABC v6 file ('MonolithExport Model File v6', e.g. "
            "Blood 2 models). This addon's v6 support was removed 2026-09 "
            "-- it could only ever produce a static rest pose here (no "
            "writer exists that could round-trip v6's vertex-deformation "
            "cloth animation). Use the Five-Damned-Dollarz/io_scene_lithtech "
            "addon instead, which has working v6 import/export including "
            "animated vertex deformation." % os.path.basename(path))
    if fmt == 'ltb_pc':
        return PCLTBModelReader().from_file(path)
    if fmt == 'ltb_ps2':
        return PS2LTBModelReader().from_file(path)
    if fmt == 'ltb_dhnp_abc':
        return DHNPABCModelReader().from_file(path)
    if fmt == 'ltb_dhnp_d3d':
        return DHNPD3DModelReader().from_file(path)
    raise ValueError("Unrecognised LithTech model header: %s" % os.path.basename(path))
