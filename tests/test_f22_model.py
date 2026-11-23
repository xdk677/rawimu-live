import math
import unittest

from test_rawimu_live import live


class F22ModelTests(unittest.TestCase):
    def test_twin_outward_canted_fins_replace_single_center_fin(self):
        vertices = [point for points, _, _ in live.aircraft_parts() for point in points]
        tips = [point for point in vertices if point[0] < -.7 and point[2] < -.7]
        self.assertTrue(any(point[1] > .5 for point in tips))
        self.assertTrue(any(point[1] < -.5 for point in tips))
        self.assertFalse(any(abs(point[1]) < .1 for point in tips))

    def test_faces_have_finite_vertices_valid_indices_and_nonzero_normals(self):
        for vertices, faces, color in live.aircraft_parts():
            self.assertTrue(all(math.isfinite(value) for point in vertices for value in point))
            self.assertTrue(all(0 <= channel <= 255 for channel in color))
            for face in faces:
                self.assertGreaterEqual(len(face), 3)
                self.assertTrue(all(0 <= index < len(vertices) for index in face))
                a, b, c = (vertices[index] for index in face[:3])
                self.assertGreater(live.norm(live.cross(live.sub(b, a), live.sub(c, a))), 1e-9)

