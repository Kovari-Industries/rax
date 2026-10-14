"""Calibration corrections specific to RoboDojo's Isaac pinhole renderer."""

import numpy as np


def rendered_intrinsics(matrix) -> np.ndarray:
    """Match Isaac's square-pixel rendering, preserving horizontal FOV.

    RoboDojo sets both apertures with maintain_square_pixels=False. The high
    camera's vertical aperture disagrees with its image aspect ratio; Isaac's
    intrinsics getter reads that stale value even though rendering uses square
    pixels. For these unresized pinhole images, the effective fy is fx. Apply
    this at native observation/preview publication, not to arbitrary user K
    matrices in the general pixel-geometry utilities.
    """
    intrinsic = np.array(matrix, dtype=np.float64, copy=True)
    if (intrinsic.shape != (3, 3) or not np.isfinite(intrinsic).all()
            or intrinsic[0, 0] <= 0):
        raise ValueError("Invalid Isaac pinhole intrinsic matrix")
    intrinsic[1, 1] = intrinsic[0, 0]
    return intrinsic
