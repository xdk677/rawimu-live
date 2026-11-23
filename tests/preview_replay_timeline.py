"""Capture the actual paused replay timeline using bundled data, without serial hardware."""
import argparse
import importlib.util
from pathlib import Path
import time
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
    root.title('RAWIMU Workbench - BUNDLED REPLAY TIMELINE DEMO')
    if args.compact:
        root.geometry('1100x720')
    app.start_replay(str(ROOT / 'examples/rawimu_test.dat'), speed=1., start_position=7.2, paused=True)
    deadline = time.monotonic() + 12
    def save():
        info = app.worker.timeline()
        if not info['ready'] or info['seeking']:
            if time.monotonic() > deadline:
                raise RuntimeError('Replay preparation timed out: ' + app.sess.error)
            root.after(100, save)
            return
        app._dirty = True
        app.refresh()
        root.update_idletasks()
        output = Path(args.output_dir)
        output.mkdir(parents=True, exist_ok=True)
        path = output / ('ui_python_replay_timeline' + ('_compact' if args.compact else '') + '.png')
        if not app.save_snapshot(str(path)):
            raise RuntimeError('Snapshot failed')
        print(path)
        app.on_close()
    root.after(700, save)
    root.mainloop()


if __name__ == '__main__':
    main()
