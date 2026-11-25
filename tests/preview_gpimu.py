"""Render GPIMU tabs using synthetic GPIMU derived from the bundled RAWIMU fixture."""
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
    app = live.App(root, dict(live.DEFAULT_CFG, auto_bias=False, auto_accel_scale=False), autostart=False)
    root.title('RAWIMU Workbench - SYNTHETIC GPIMU / BUNDLED RAWIMU DEMO')
    if args.compact:
        root.geometry('1100x720')
    app.sess.feed((ROOT / 'examples/gpimu_test.dat').read_bytes())
    app.sess.state = 'finished'
    app.sess.status = 'SYNTHETIC GPIMU derived from bundled RAWIMU - no hardware'
    app.notebook.select(app.gpimu_page)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)

    def capture(index):
        if not app.gpimu_view.monitor_panels[0][0].winfo_ismapped() and index == 0:
            root.after(100, lambda: capture(index))
            return
        app.gpimu_view._mark_dirty()
        app._dirty = True
        app.refresh()
        app.gpimu_view.inspect_time(8.)
        root.update_idletasks()
        name = ('monitor', 'comparison', 'fields')[index]
        path = output / ('ui_python_gpimu_' + name + ('_compact' if args.compact else '') + '.png')
        if not app.save_snapshot(str(path)):
            raise RuntimeError('Snapshot failed')
        print(path)
        if index < 2:
            app.gpimu_view.notebook.select(index+1)
            root.after(350, lambda: capture(index+1))
        else:
            app.on_close()
    root.after(700, lambda: capture(0))
    root.mainloop()


if __name__ == '__main__':
    main()
