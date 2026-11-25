import sys
from pathlib import Path
import unittest

from test_rawimu_live import ROOT

sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / '.vendor'))

FIXTURE = b'''AC3Db
MATERIAL "gray" rgb 0.5 0.6 0.7 amb 0.2 0.2 0.2 emis 0 0 0 spec 0 0 0 shi 8 trans 0
OBJECT world
kids 1
OBJECT group
loc 1 2 3
kids 1
OBJECT poly
name "triangle"
loc 2 0 0
numvert 3
0 0 0
1 0 0
0 1 0
numsurf 1
SURF 0x10
mat 0
refs 3
0 0 0
1 1 0
2 0 1
kids 0
'''


class DetailedMeshTests(unittest.TestCase):
    def test_parser_preserves_parent_translation_uv_and_material(self):
        from aircraft_mesh import parse_ac3d
        parsed = parse_ac3d(FIXTURE)
        part = parsed['objects'][0]
        self.assertEqual(part['vertices'], [(3., 2., 3.), (4., 2., 3.), (3., 3., 3.)])
        self.assertEqual(part['faces'][0]['uv'], [(0., 0.), (1., 0.), (0., 1.)])
        self.assertEqual(parsed['materials'][0]['rgb'], (.5, .6, .7))

    def test_parser_rejects_out_of_range_indices(self):
        from aircraft_mesh import parse_ac3d
        with self.assertRaises(ValueError):
            parse_ac3d(FIXTURE.replace(b'2 0 1\n', b'99 0 1\n'))

    def test_object_data_supports_windows_line_endings(self):
        from aircraft_mesh import parse_ac3d
        fixture = FIXTURE.replace(b'name "triangle"\n', b'name "triangle"\ndata 8\ntriangle\n')
        self.assertEqual(parse_ac3d(fixture.replace(b'\n', b'\r\n'))['objects'][0]['name'], 'triangle')

    def test_real_asset_has_detailed_finite_geometry_and_valid_textures(self):
        import numpy as np
        from aircraft_mesh import load_f22
        model = load_f22()
        self.assertGreater(model['triangles'], 10000)
        self.assertLess(model['triangles'], 150000)
        for batch in model['batches']:
            array = batch['array']
            self.assertEqual(array.shape[1], 8)
            self.assertTrue(np.isfinite(array).all())
            self.assertTrue(np.allclose(np.linalg.norm(array[:, 3:6], axis=1), 1, atol=1e-4))
            if batch['texture']:
                self.assertTrue(Path(batch['texture']).is_file())
        left = next(part for part in model['parts'] if part['name'] == 'Left_Wing_0')
        right = next(part for part in model['parts'] if part['name'] == 'Right_Wing_0')
        self.assertLess(left['center'][1], 0)
        self.assertGreater(right['center'][1], 0)
        self.assertLess(model['bounds'][0][0], -1.8)
        self.assertGreater(model['bounds'][1][0], 1.8)


class OpenGLF22Tests(unittest.TestCase):
    def test_app_snapshot_resize_and_render_failure_fallback(self):
        import tempfile
        import tkinter as tk
        import numpy as np
        from PIL import Image
        from test_rawimu_live import live
        root = tk.Tk()
        failures = []
        root.report_callback_exception = lambda *args: failures.append(args)
        app = live.App(root, dict(live.DEFAULT_CFG, renderer='auto'), autostart=False)
        try:
            root.update()
            view = app.scene.gl_view
            self.assertIsNotNone(view)
            self.assertFalse(view.failed, app.scene.gpu_error)
            for size in ('1100x720', '1600x960'):
                root.geometry(size)
                root.update()
                captured = np.asarray(view.capture())
                self.assertEqual(captured.shape[:2], (view.winfo_height(), view.winfo_width()))
            with tempfile.TemporaryDirectory() as folder:
                path = str(Path(folder) / 'snapshot.png')
                self.assertTrue(app.save_snapshot(path))
                with Image.open(path) as image:
                    self.assertGreater(image.width, 1500)
            app.scene.update(live.quat_to_dcm(live.euler_to_quat(.2, .3, .4)), .2, .3, .4, 'F-22')
            view._fail(RuntimeError('simulated GPU failure'))
            root.update()
            self.assertFalse(view.winfo_ismapped())
            self.assertEqual(app.scene.gpu_error, 'simulated GPU failure')
            self.assertTrue(any(app.cv3d.coords(item) for item in app.scene.faces))
        finally:
            app.on_close()
        self.assertEqual(failures, [])

    def test_context_released_even_when_buffer_deletion_fails(self):
        import tkinter as tk
        from unittest.mock import patch
        from aircraft_gl import DetailedF22View
        from OpenGL.WGL import wglDeleteContext
        root = tk.Tk()
        failures = []
        root.report_callback_exception = lambda *args: failures.append(args)
        view = DetailedF22View(root, {'view_az': -125., 'view_el': -26.})
        view.pack(fill='both', expand=True)
        root.update()
        with patch('aircraft_gl.GL.glDeleteBuffers', side_effect=RuntimeError('driver cleanup error')), \
                patch('OpenGL.WGL.wglDeleteContext', wraps=wglDeleteContext) as delete_context:
            root.destroy()
        delete_context.assert_called_once()
        self.assertFalse(view.context_created)
        self.assertIsNone(view._OpenGLFrame__context)
        self.assertEqual(failures, [])

    def test_gpu_render_and_attitude_update(self):
        import tkinter as tk
        import numpy as np
        from aircraft_gl import DetailedF22View
        root = tk.Tk()
        root.geometry('480x360')
        failures = []
        view = DetailedF22View(root, {'view_az': -125., 'view_el': -26.}, on_error=failures.append)
        view.pack(fill='both', expand=True)
        try:
            root.update()
            self.assertEqual(failures, [])
            self.assertTrue(view.context_created)
            first = np.asarray(view.capture())
            self.assertGreater(len(np.unique(first.reshape(-1, 3), axis=0)), 100)
            view.set_attitude(((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)))
            second = np.asarray(view.capture())
            self.assertGreater(int(np.count_nonzero(np.any(first != second, axis=2))), 1000)
            self.assertTrue(view.renderer)
        finally:
            root.destroy()
