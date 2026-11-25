"""Render static-performance tabs with clearly marked synthetic stationary samples."""
import argparse
import importlib.util
from pathlib import Path
import sys
import time
import tkinter as tk

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from performance_data import AnalysisData


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compact', action='store_true')
    parser.add_argument('--output-dir', default=str(ROOT / 'docs/images'))
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('viewer', str(ROOT / 'src/rawimu_live.py'))
    live = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(live)
    root = tk.Tk()
    root.withdraw()
    app = live.App(root, dict(live.DEFAULT_CFG, renderer='canvas'), autostart=False)
    app.open_performance_window()
    view = app.performance_window
    view.window.title('Static IMU performance - SYNTHETIC STATIONARY DEMO')
    if args.compact:
        view.window.geometry('1100x760')
    count = 12000
    rows = np.zeros((count, 13))
    rows[:, 0] = np.arange(count) * .01
    rows[:, 1] = 1400000000. + rows[:, 0]
    rng = np.random.RandomState(52)
    rows[:, 2:8] = rng.normal(size=(count, 6)) * [.001, .0015, .0008, .003, .004, .003]
    rows[:, 2:8] += [.004, -.002, .001, .015, -.008, 9.80665]
    rows[:, 8] = 25.
    rows[:, 9] = .01
    rows[0, 9] = np.nan
    rows[:, 10] = 100
    rows[:, 11] = np.arange(count) % 256
    view.source_data = AnalysisData(rows, 'SYNTHETIC STATIONARY DEMO')
    view.refresh_source()
    view.confirmed_var.set(True)
    view.start_analysis()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + 20.

    def save_tab(index):
        if view.busy:
            if time.monotonic() > deadline:
                raise RuntimeError('Analysis timeout')
            root.after(100, lambda: save_tab(index))
            return
        if view.result is None:
            raise RuntimeError(view.status_var.get())
        view.notebook.select(index)
        root.after(350, lambda: capture(index))

    def capture(index):
        suffix = ('stats', 'smoothing', 'allan')[index]
        path = output / ('ui_python_performance_' + suffix + ('_compact' if args.compact else '') + '.png')
        root.update_idletasks()
        if not live.grab_window(view.window.title(), str(path), int(view.window.wm_frame(), 16)):
            raise RuntimeError('Snapshot failed')
        print(path)
        if index < 2:
            save_tab(index + 1)
        else:
            app.on_close()
    root.after(250, lambda: save_tab(0))
    root.mainloop()


if __name__ == '__main__':
    main()
