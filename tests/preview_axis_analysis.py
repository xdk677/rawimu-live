"""Render the independent analysis window with bundled replay data, without serial hardware."""
import argparse
import importlib.util
from pathlib import Path
import tkinter as tk

ROOT = Path(__file__).resolve().parents[1]


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
    app = live.App(root, dict(live.DEFAULT_CFG, renderer='canvas', auto_bias=False,
                              auto_accel_scale=False), autostart=False)
    app.sess.feed((ROOT / 'examples/rawimu_test.dat').read_bytes())
    app.sess.state = 'finished'
    app.open_axes_window()
    view = app.axes_window
    view.window.title('IMU independent axis analysis - BUNDLED REPLAY DEMO')
    if args.compact:
        view.window.geometry('1000x720')

    def save():
        app._dirty = True
        app.refresh()
        app.inspect_time(12.5)
        root.update_idletasks()
        folder = Path(args.output_dir)
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / ('ui_python_axis_analysis' + ('_compact' if args.compact else '') + '.png')
        if not live.grab_window(view.window.title(), str(path), int(view.window.wm_frame(), 16)):
            raise RuntimeError('Snapshot failed')
        print(path)
        app.on_close()

    root.after(700, save)
    root.mainloop()


if __name__ == '__main__':
    main()
