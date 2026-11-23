"""Reproduce timing diagnostics and static-Allan results for a supplied recording."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import sys
import time
import tkinter as tk

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from performance_data import AnalysisData, load_recording
from imu_performance import AXES, UNITS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording')
    parser.add_argument('--cached-records', help='Optional previously decoded .npy rows, for this exact recording')
    parser.add_argument('--output-dir', default=str(ROOT/'docs/diagnostics'))
    parser.add_argument('--compact', action='store_true')
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('viewer', str(ROOT/'src/rawimu_live.py'))
    live = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(live)
    cfg = dict(live.DEFAULT_CFG, renderer='canvas', auto_bias=False, auto_accel_scale=False)
    if args.cached_records:
        data = AnalysisData(np.load(args.cached_records, mmap_mode='r'), args.recording,
                            {'model': 'M1633', 'crc_errors': 0, 'static_confirmation': 'user supplied',
                             'cache_note': 'decoded from the exact source recording before this verification'})
    else:
        data = load_recording(args.recording, cfg)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    root = tk.Tk()
    root.withdraw()
    app = live.App(root, cfg, autostart=False)
    app.open_performance_window()
    view = app.performance_window
    view.window.title('Static IMU - ACTUAL RECORDING - bounded timestamp jitter')
    if args.compact:
        view.window.geometry('1100x760')
    view.source_data = data
    view.refresh_source()
    view.confirmed_var.set(True)  # The user explicitly stated this recording is stationary.
    view.start_analysis()
    deadline = time.monotonic() + 120.

    def complete():
        if view.busy:
            if time.monotonic() > deadline:
                raise RuntimeError('Analysis timeout')
            root.after(100, complete)
            return
        if view.result is None or view.result['allan'] is None:
            raise RuntimeError(view.status_var.get()+'; '+view.quality_var.get())
        result = view.result
        report = {'source': args.recording, 'quality': result['quality'],
                  'static_confirmation': 'user supplied',
                  'stats': {key: value.tolist() for key, value in result['stats'].items()},
                  'motion_notes': result['motion_notes']}
        (output/'allan_actual_timing_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        with (output/'allan_actual_summary.csv').open('w', newline='', encoding='utf-8-sig') as handle:
            writer = csv.writer(handle)
            keys = tuple(result['stats'])
            writer.writerow(['axis', 'unit']+list(keys))
            for k, (axis, unit) in enumerate(zip(AXES, UNITS)):
                writer.writerow([axis, unit]+[result['stats'][key][k] for key in keys])
        allan = result['allan']
        with (output/'allan_actual_curve.csv').open('w', newline='', encoding='utf-8-sig') as handle:
            writer = csv.writer(handle)
            writer.writerow(['tau_s', 'overlapping_pairs']+
                            [axis+'_deviation_'+unit for axis, unit in zip(AXES, UNITS)]+
                            [axis+'_variance_('+unit+')^2' for axis, unit in zip(AXES, UNITS)])
            for k, tau in enumerate(allan['tau']):
                writer.writerow([tau, allan['pairs'][k]]+allan['deviation'][k].tolist()+allan['variance'][k].tolist())
        print(json.dumps({'quality': result['quality'], 'allan_min': result['stats']['allan_min'].tolist(),
                          'tau_at_min': result['stats']['allan_min_tau'].tolist()}, ensure_ascii=True, indent=2))
        root.after(350, lambda: snapshot(0))

    def snapshot(index):
        root.update_idletasks()
        name = 'stats' if index == 0 else 'allan'
        path = output/('actual_timestamp_jitter_'+name+('_compact' if args.compact else '')+'.png')
        if not live.grab_window(view.window.title(), str(path), int(view.window.wm_frame(), 16)):
            raise RuntimeError('Snapshot failed')
        print(path)
        if index == 0:
            view.notebook.select(2)
            root.after(350, lambda: snapshot(1))
        else:
            app.on_close()

    root.after(200, complete)
    root.mainloop()


if __name__ == '__main__':
    main()
