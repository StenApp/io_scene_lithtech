# -*- coding: utf-8 -*-
"""
reader_dispatch.py  --  ONE job: look at a file, pick the right reader, return
the unified abc.py:Model (raw LithTech space). Nothing here knows about Blender
or coordinates; conversion happens later in builder_import.

Detection (deterministic, by header -- not the old try/except cascade):
    ABC      : starts with length-prefixed string "Header"   -> ABCModelReader
    LTB PC   : file_type(uint16)=1                            -> PCLTBModelReader
    LTB PS2  : file_type(uint16)=2                            -> PS2LTBModelReader

Works both as an addon package member and standalone (path) via the import shim.
"""

import os
import struct

# package member (addon) first, standalone (sys.path) as fallback
try:
    from .reader_abc_pc import ABCModelReader
    from .reader_ltb_pc import PCLTBModelReader
    from .reader_ltb_ps2 import PS2LTBModelReader
    from .reader_lta import LTAModelReader
except ImportError:
    from reader_abc_pc import ABCModelReader
    from reader_ltb_pc import PCLTBModelReader
    from reader_ltb_ps2 import PS2LTBModelReader
    from reader_lta import LTAModelReader


def detect_format(path):
    """Return one of 'lta', 'abc', 'ltb_pc', 'ltb_ps2', or 'unknown'."""
    with open(path, 'rb') as f:
        head = f.read(16)
    # LTA is text starting with '(' (after optional BOM/whitespace)
    stripped = head.lstrip(b'\xef\xbb\xbf \t\r\n')
    if stripped[:1] == b'(' or os.path.splitext(path)[1].lower() == '.lta':
        return 'lta'
    if len(head) >= 8:
        n = struct.unpack_from('<H', head, 0)[0]
        if n == 6 and head[2:8] == b'Header':
            return 'abc'
    if len(head) >= 2:
        ftype = struct.unpack_from('<H', head, 0)[0]
        if ftype == 1:
            return 'ltb_pc'
        if ftype == 2:
            return 'ltb_ps2'
    return {'.abc': 'abc'}.get(os.path.splitext(path)[1].lower(), 'unknown')


def read_model(path):
    """Detect format and return a populated abc.py:Model."""
    fmt = detect_format(path)
    if fmt == 'lta':
        return LTAModelReader().from_file(path)
    if fmt == 'abc':
        return ABCModelReader().from_file(path)
    if fmt == 'ltb_pc':
        return PCLTBModelReader().from_file(path)
    if fmt == 'ltb_ps2':
        return PS2LTBModelReader().from_file(path)
    raise ValueError("Unrecognised LithTech model header: %s" % os.path.basename(path))
