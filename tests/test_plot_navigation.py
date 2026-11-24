import tkinter as tk
import unittest

from test_rawimu_live import live


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=600, height=350)
        self.canvas.pack()
        self.root.update()
        self.panel = live.PlotPanel(self.canvas, 'test', 'unit', ['X'])
        self.panel.resize(600, 350)
        self.owner = type('Owner', (), {'frozen': False})()
        self.panel.bind_navigation(self.owner)
        self.panel.update([0., 5., 10.], [[-10., 10., -10.]])

    def tearDown(self):
        self.root.destroy()

    def wheel(self, state=0):
        self.canvas.event_generate('<MouseWheel>', x=300, y=160, delta=120, state=state)
        self.root.update()

    def test_only_frozen_zoom_and_reset_on_resume(self):
        base = self.panel._bounds
        self.wheel()
        self.assertEqual(self.panel._bounds, base)
        self.owner.frozen = True
        self.wheel()
        self.assertLess(self.panel._bounds[5] - self.panel._bounds[4], base[5] - base[4])
        self.owner.frozen = False
        self.panel.update(self.panel._last_t, self.panel._last_ys)
        self.assertEqual(self.panel._bounds, base)

    def test_y_zoom_clips_lines_and_restores(self):
        self.owner.frozen = True
        base = self.panel._bounds
        self.wheel(state=4)  # Ctrl: vertical only.
        self.assertEqual(self.panel._bounds, base)
        self.assertIsNotNone(self.panel.navigation.view)
        left, top, width, height = base[:4]
        for item in self.panel.pool.values():
            if self.canvas.itemcget(item, 'state') != 'hidden':
                coords = self.canvas.coords(item)
                self.assertTrue(all(top - 1e-6 <= y <= top + height + 1e-6 for y in coords[1::2]))
        self.panel.navigation.reset()
        self.assertEqual(self.panel._bounds, base)

    def test_pan_cursor_and_changed_source(self):
        self.owner.frozen = True
        self.wheel(state=1)  # Shift: horizontal only.
        before = self.panel._bounds
        self.canvas.event_generate('<ButtonPress-1>', x=300, y=160)
        self.canvas.event_generate('<B1-Motion>', x=340, y=160)
        self.canvas.event_generate('<ButtonRelease-1>', x=340, y=160)
        self.assertLess(self.panel._bounds[4], before[4])
        inspected = []
        self.panel.bind_cursor(inspected.append)
        self.canvas.event_generate('<Motion>', x=300, y=160)
        bounds = self.panel._bounds
        self.assertAlmostEqual(inspected[-1], bounds[4] + (300-bounds[0])/bounds[2]*(bounds[5]-bounds[4]))
        self.panel.time_range = (20., 30.)
        self.panel.update([20., 30.], [[0., 1.]])
        self.assertEqual(self.panel._bounds[4:], (20., 30.))

    def test_double_click_reset_and_zoom_limits(self):
        self.owner.frozen = True
        for _ in range(70):
            self.wheel()
        view = self.panel.navigation.view
        self.assertGreater(view[1]-view[0], 0.)
        self.panel.navigation.double_click(type('Event', (), {'x': 300, 'y': 160})())
        self.assertIsNone(self.panel.navigation.view)
        for _ in range(5):
            self.canvas.event_generate('<MouseWheel>', x=300, y=160, delta=-120)
        self.assertEqual(self.panel.navigation.view, self.panel.navigation.base)

    def test_fault_lanes_keep_y_and_integrity_children_are_independent(self):
        self.canvas.destroy()
        canvas = tk.Canvas(self.root, width=600, height=400)
        canvas.pack()
        self.root.update()
        panel = live.StatusPanel(canvas)
        panel.bind_navigation(self.owner)
        panel.resize(600, 400)
        panel.update([0., 5., 10.], [[0, 256, 0]])
        self.owner.frozen = True
        canvas.event_generate('<MouseWheel>', x=300, y=160, delta=120)
        self.assertEqual(panel.navigation.view[2:], (0., 1.))
        canvas.destroy()
        canvas = tk.Canvas(self.root, width=600, height=400)
        canvas.pack()
        integrity = live.IntegrityPanel(canvas)
        integrity.bind_navigation(self.owner)
        integrity.resize(600, 400)
        self.root.update()
        for child in (integrity.time_plot, integrity.counter_plot):
            child.update([0., 5., 10.], [[10., 11., 10.]])
        integrity.time_canvas.event_generate('<MouseWheel>', x=300, y=80, delta=120)
        self.assertIsNotNone(integrity.time_plot.navigation.view)
        self.assertIsNone(integrity.counter_plot.navigation.view)

    def test_allan_logarithmic_chart_zoom_and_pan(self):
        import numpy as np
        from performance_window import LogAllanPlot
        self.canvas.destroy()
        canvas = tk.Canvas(self.root, width=600, height=350)
        canvas.pack()
        self.root.update()
        plot = LogAllanPlot(canvas, 'Allan')
        tau = np.logspace(-2, 2, 100)
        values = np.column_stack([tau**-.5, tau**-.4, tau**-.3])
        plot.set_data(tau, values, 'deg/s')
        canvas.event_generate('<MouseWheel>', x=300, y=160, delta=120)
        self.assertIsNotNone(plot.navigation.view)
        self.assertLess(plot.navigation.view[1]-plot.navigation.view[0], 4.)
        # Resize/map redraw must preserve the chosen logarithmic viewport.
        chosen = plot.navigation.view
        plot.redraw()
        self.assertEqual(plot.navigation.view, chosen)
        plot.set_data(tau, values**2, '(deg/s)^2')
        self.assertIsNone(plot.navigation.view)

    def test_application_all_views_freeze_and_resume_gate(self):
        self.canvas.destroy()
        app = live.App(self.root, dict(live.DEFAULT_CFG, renderer='canvas'), autostart=False)
        try:
            app.open_axes_window()
            self.root.update()
            all_panels = list(app._plot_panels()) + list(app.gpimu_view.panels)
            self.assertTrue(all(not p.navigation.allow() for _, p in all_panels
                                if not isinstance(p, live.IntegrityPanel)))
            app.toggle_freeze()
            self.assertTrue(all(p.navigation.allow() for _, p in all_panels
                               if not isinstance(p, live.IntegrityPanel)))
            gp = app.gpimu_view.monitor_panels[3][1]
            gp.resize(600, 350)
            gp.render_enabled = True
            gp.update([0., 5., 10.], [[0., 1., 0.]])
            original = gp._bounds
            gp.navigation.wheel(type('Event', (), {'x': 300, 'y': 160, 'state': 0, 'delta': 120})())
            self.assertNotEqual(gp._bounds, original)
            app.toggle_freeze()
            self.assertIsNone(gp.navigation.view)
            self.assertEqual(gp._bounds, original)
            app.gpimu_view.file_data = {}
            self.assertTrue(gp.navigation.allow())
            app.gpimu_view.file_data = None
        finally:
            app.on_close()
            # App closes the root itself.
            self.root = tk.Tk()
            self.root.withdraw()

