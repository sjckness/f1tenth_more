"""
The SLAM map (/slam/map, nav_msgs/OccupancyGrid) as a small PNG for the phone.

numpy for the pixels, zlib + struct for the PNG (no Pillow on the car).
Gray + alpha: occupied cells light, free cells dark, unknown transparent.
Row 0 of the grid is the map's lowest y, so rows are flipped to put +y up.
Grids wider than max_px are reduced by an integer factor; a reduced cell is
occupied if any cell it covers is, free if any is known.
"""

import struct
from typing import Dict, Sequence
import zlib

import numpy as np


def _chunk(kind: bytes, data: bytes) -> bytes:
    return (struct.pack('>I', len(data)) + kind + data
            + struct.pack('>I', zlib.crc32(kind + data) & 0xFFFFFFFF))


def grid_to_png(width: int, height: int, data: Sequence[int], max_px: int = 800,
                occupied_from: int = 50) -> Dict:
    """Return {'png': bytes, 'factor': k, 'width': w, 'height': h} for the reduced grid."""
    if width <= 0 or height <= 0 or len(data) < width * height:
        raise ValueError('empty or short grid')
    grid = np.asarray(data[:width * height], dtype=np.int16).reshape(height, width)
    k = max(1, -(-max(width, height) // max_px))
    h, w = -(-height // k), -(-width // k)
    padded = np.full((h * k, w * k), -1, dtype=np.int16)
    padded[:height, :width] = grid
    blocks = padded.reshape(h, k, w, k)
    occupied = (blocks >= occupied_from).any(axis=(1, 3))
    known = (blocks >= 0).any(axis=(1, 3))
    gray = np.where(occupied, 0xD8, np.where(known, 0x20, 0)).astype(np.uint8)
    alpha = np.where(known, 0xFF, 0).astype(np.uint8)
    img = np.flipud(np.stack([gray, alpha], axis=-1))          # +y up
    raw = b''.join(b'\x00' + img[r].tobytes() for r in range(h))
    png = (b'\x89PNG\r\n\x1a\n'
           + _chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 4, 0, 0, 0))
           + _chunk(b'IDAT', zlib.compress(raw, 6)) + _chunk(b'IEND', b''))
    return {'png': png, 'factor': k, 'width': w, 'height': h}
