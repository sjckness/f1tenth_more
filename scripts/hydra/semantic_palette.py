"""Colours for the Isaac GT semantic mask, shared by the recorder and the video renderer.

The ids are the scene's own: `semantics:` in sim/isaac/config/scenes/<scene>.yaml. The office
world uses {unlabeled 0, floor 1, wall 2, person 3, car 4, glass 7, door 8}.

What the colours are FOR is reading a mask by eye and understanding what Hydra will do with it,
so the comments below say where each id ends up in Hydra's label space
(src/f1tenth_hydra/config/labels/isaac_to_ade20k_mp3d.yaml). numpy only: no ROS, no OpenCV.
"""
from __future__ import annotations

import numpy as np

# id -> (R, G, B), and what Hydra makes of it
PALETTE: dict[int, tuple[int, int, int]] = {
    0: (25, 25, 30),       # unlabeled -> misc (38): integrated, NEVER an object. In the office
                           #   world this is every desk, chair, monitor and plant.
    1: (70, 100, 150),     # floor -> 2
    2: (220, 140, 60),     # wall -> 1, and a `surface_places_label`, so it becomes mesh places
    3: (220, 60, 60),      # person -> 0 (void): masked out of integration, dynamic
    4: (200, 60, 200),     # car -> 0 (void): our own body, if it ever enters frame
    7: (90, 210, 220),     # glass -> 9 (window)
    8: (90, 200, 110),     # door -> 4
}
UNKNOWN = (255, 0, 255)    # magenta: an id the scene did not declare

NAMES: dict[int, str] = {0: "unlabeled", 1: "floor", 2: "wall", 3: "person", 4: "car",
                         7: "glass", 8: "door"}


def colourise(ids: np.ndarray) -> tuple[np.ndarray, list[int]]:
    """mono8 class ids -> (RGB uint8 array, the ids actually present)."""
    rgb = np.zeros(ids.shape + (3,), dtype=np.uint8)
    seen = [int(i) for i in np.unique(ids)]
    for i in seen:
        rgb[ids == i] = PALETTE.get(i, UNKNOWN)
    return rgb, seen
