"""Render clearly marked synthetic IMU status changes for UI inspection (no serial access)."""
import argparse
import importlib.util
import math
from pathlib import Path
import struct
import tkinter as tk

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compact', action='store_true', help='inspect the 1100x720 minimum layout')
    parser.add_argument('--output-dir', default=str(ROOT / 'docs/images'))
    args = parser.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    spec = importlib.util.spec_from_file_location('viewer', str(ROOT / 'src/rawimu_live.py'))
    live = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(live)
    root = tk.Tk()
    app = live.App(root, dict(live.DEFAULT_CFG, auto_bias=False, auto_accel_scale=False), autostart=False)
    root.title('RAWIMU Workbench - SYNTHETIC STATUS DEMO')
    if args.compact:
        root.geometry('1100x720')
    data = bytearray((ROOT / 'examples/rawimu_test.dat').read_bytes())
    for index in range(2000):
        start = index * 72
        flags = ((1 << ((index // 180) % 8)) if (index // 90) % 2 else 0) | (128 if index > 1200 else 0)
        word = ((2500 + int(50 * math.sin(index / 200.))) << 16) | (flags << 8) | (index & 255)
        struct.pack_into('<I', data, start + 40, word)
        struct.pack_into('<I', data, start + 68, live.crc32_raw(data[start:start + 68]))
    app.sess.parser.feed(data)
    app.sess.state = 'finished'
    app.sess.status = 'SYNTHETIC STATUS DEMO - for display verification only'

    def save(name):
        app._dirty = True
        app.refresh()
        app.inspect_time(12.5)
        root.update_idletasks()
        path = output / ('ui_python_status_' + name + ('_compact' if args.compact else '') + '.png')
        if not app.save_snapshot(str(path)):
            raise RuntimeError('Snapshot failed: ' + str(path))
        print(path)

    def overview():
        save('overview')
        app.notebook.select(1)
        root.after(350, analysis)

    def analysis():
        save('analysis')
        app.on_close()

    root.after(700, overview)
    root.mainloop()


if __name__ == '__main__':
    main()
