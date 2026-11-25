import sys
import tkinter as tk
import unittest

from test_rawimu_live import ROOT, live

sys.path.insert(0, str(ROOT / 'src'))


@unittest.skipUnless(sys.platform.startswith('darwin'), 'macOS CGL only')
class MacF22ViewTests(unittest.TestCase):
    def setUp(self):
        from mac_aircraft_gl import MacF22View
        self.root = tk.Tk()
        self.root.geometry('640x480')
        self.view = MacF22View(
            self.root, {'view_az': -125., 'view_el': -26.})
        self.view.pack(fill='both', expand=True)
        self.root.update()

    def tearDown(self):
        self.view._cleanup()
        self.root.update()
        self.root.destroy()

    def test_native_cgl_frame_contains_textured_model(self):
        self.assertFalse(self.view.failed)
        self.assertTrue(self.view.context_created)
        self.assertTrue(self.view.renderer)
        image = self.view.capture()
        self.assertEqual(image.size, (
            self.view.winfo_width(), self.view.winfo_height()))
        self.assertGreater(len(image.resize((160, 120)).getcolors(160 * 120)), 200)

    def test_attitude_changes_rendered_pixels(self):
        import numpy as np
        first = np.asarray(self.view.capture())
        self.view.set_attitude(np.array((
            (-0.86, 0.17, 0.48),
            (0.17, 0.97, -0.04),
            (-0.48, 0.05, -0.87),
        ), dtype=np.float32))
        self.root.update()
        second = np.asarray(self.view.capture())
        self.assertGreater(int(np.count_nonzero(np.any(first != second, axis=2))), 1000)

    def test_unchanged_attitude_and_photo_are_reused(self):
        import numpy as np
        from unittest.mock import patch
        photo = self.view._photo
        self.view.set_attitude(np.eye(3, dtype=np.float32))
        with patch.object(
                self.view, 'redraw', wraps=self.view.redraw) as redraw:
            self.view.set_attitude(self.view.rotation.copy())
            self.assertEqual(redraw.call_count, 0)
            rotated = np.array((
                (0., -1., 0.),
                (1., 0., 0.),
                (0., 0., 1.),
            ), dtype=np.float32)
            self.view.set_attitude(rotated)
            self.assertEqual(redraw.call_count, 1)
        self.assertIs(self.view._photo, photo)

    def test_resize_rebuilds_framebuffer(self):
        for width, height in ((480, 320), (720, 540)):
            self.root.geometry('%dx%d' % (width, height))
            self.root.update()
            image = self.view.capture()
            self.assertEqual(image.size, (
                self.view.winfo_width(), self.view.winfo_height()))
        self.assertFalse(self.view.failed)

    def test_main_scene_uses_mac_renderer(self):
        from mac_aircraft_gl import MacF22View
        root = tk.Tk()
        root.withdraw()
        app = live.App(
            root, dict(live.DEFAULT_CFG, renderer='auto'), autostart=False)
        try:
            root.update()
            self.assertIsInstance(app.scene.gl_view, MacF22View)
            self.assertFalse(app.scene.gl_view.failed)
            self.assertEqual(app.scene.gpu_error, '')
        finally:
            app.on_close()
            for _ in range(50):
                try:
                    root.update()
                    if not root.winfo_exists():
                        break
                except tk.TclError:
                    break


if __name__ == '__main__':
    unittest.main()
