"""Isaac square-pixel calibration, independent of simulator availability."""
import numpy as np
import pytest

from services.robodojo.camera_calibration import rendered_intrinsics
from services.robodojo.pixel_geometry import pixel_ray_world


@pytest.mark.parametrize("fx,fy", [(288.13254, 336.46432), (397.04126, 397.04126)])
def test_rendered_calibration_preserves_horizontal_fov_and_input(fx, fy):
    original = np.array([[fx, 0, 320], [0, fy, 240], [0, 0, 1.]])
    before = original.copy()
    result = rendered_intrinsics(original)
    expected = original.copy()
    expected[1, 1] = fx
    np.testing.assert_array_equal(result, expected)
    np.testing.assert_array_equal(original, before)
    assert not np.shares_memory(result, original)


@pytest.mark.parametrize("matrix", [np.eye(2), np.full((3, 3), np.nan), np.zeros((3, 3))])
def test_invalid_calibration_is_rejected(matrix):
    with pytest.raises(ValueError, match="intrinsic"):
        rendered_intrinsics(matrix)


def test_general_pixel_geometry_still_accepts_non_square_calibration():
    # The correction is Isaac-specific, not a global fy=fx assumption.
    observation = {"camera_parameters": {"cam_high": {
        "intrinsic_matrix": [[100, 0, 320], [0, 200, 240], [0, 0, 1]],
        "camera_to_world_usd": np.eye(4), "resolution": [640, 480],
    }}}
    ray = pixel_ray_world(observation, {"camera": "cam_high", "pixel": [420, 440]},
                          coordinate_space="image_pixels")
    np.testing.assert_allclose(ray["ray_cv"], [1, 1, 1])
