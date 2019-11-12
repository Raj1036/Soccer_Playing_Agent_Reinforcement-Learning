from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from absl.testing import absltest
from dm_control import mjcf
from dm_control.locomotion.arenas import floors
import numpy as np


class FloorsTest(absltest.TestCase):

  def test_can_compile_mjcf(self):
    arena = floors.Floor()
    mjcf.Physics.from_mjcf_model(arena.mjcf_model)

  def test_size(self):
    floor_size = (12.9, 27.1)
    arena = floors.Floor(size=floor_size)
    self.assertEqual(tuple(arena.ground_geoms[0].size[:2]), floor_size)

  def test_top_camera(self):
    floor_width, floor_height = 12.9, 27.1
    arena = floors.Floor(size=[floor_width, floor_height])

    self.assertGreater(floors._TOP_CAMERA_Y_PADDING_FACTOR, 1)
    np.testing.assert_array_equal(arena._top_camera.zaxis, (0, 0, 1))

    expected_camera_y = floor_height * floors._TOP_CAMERA_Y_PADDING_FACTOR
    np.testing.assert_allclose(
        np.tan(np.deg2rad(arena._top_camera.fovy / 2)),
        expected_camera_y / arena._top_camera.pos[2])


if __name__ == '__main__':
  absltest.main()
