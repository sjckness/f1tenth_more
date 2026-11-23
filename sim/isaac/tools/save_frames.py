"""Save left/right/depth/semantic PNGs from <run>/frames.npz (written by ros_checks.py geometry).

    /isaac-sim/python.sh tools/save_frames.py <run_dir> <output_dir>
"""
from __future__ import annotations

import logging
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

log = logging.getLogger("save_frames")


def to_rgb(img: np.ndarray) -> np.ndarray:
    """bgra8/bgr8 -> rgb."""
    if img.ndim == 3 and img.shape[2] >= 3:
        return img[:, :, [2, 1, 0]]
    return img


def main(run: str, out: str) -> int:
    logging.basicConfig(level=logging.INFO)
    d = np.load(f"{run}/frames.npz")
    for name in ("left", "right"):
        if d[name].ndim == 3:
            plt.imsave(f"{out}/{name}.png", to_rgb(d[name]))
            log.info("wrote %s/%s.png", out, name)
    dep = d["depth"].astype(float)
    fin = np.isfinite(dep)
    fig, ax = plt.subplots(figsize=(8, 5))
    im = ax.imshow(np.where(fin, dep, np.nan), cmap="turbo", vmin=np.nanmin(dep[fin]) if fin.any() else 0,
                   vmax=np.nanmax(dep[fin]) if fin.any() else 1)
    fig.colorbar(im, ax=ax, label="depth (Z, m) - 32FC1; white = inf/NaN")
    ax.set_title("/zed2/zed_node/depth/depth_registered")
    fig.savefig(f"{out}/depth.png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    if d["semantic"].ndim == 2:
        fig, ax = plt.subplots(figsize=(8, 5))
        im = ax.imshow(d["semantic"], cmap="tab10", vmin=0, vmax=9, interpolation="nearest")
        fig.colorbar(im, ax=ax, label="class id (0 unlabeled,1 floor,2 wall,3 person,4 car)")
        ax.set_title("/sim/ground_truth/left/semantic")
        fig.savefig(f"{out}/ground_truth_semantic.png", dpi=110, bbox_inches="tight")
        plt.close(fig)
    log.info("frames saved to %s", out)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
