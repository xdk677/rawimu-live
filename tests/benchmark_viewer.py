"""Optional local benchmark. Run from any directory; --baseline accepts a git ref."""
import argparse
import importlib.util
import math
from pathlib import Path
import statistics
import subprocess
import time
import timeit
import types

ROOT = Path(__file__).resolve().parents[1]


def load_current():
    spec = importlib.util.spec_from_file_location('current', str(ROOT / 'src/rawimu_live.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def benchmark_backend(module):
    frame = (ROOT / 'examples/rawimu_test.dat').read_bytes()[:68]
    crc_time = timeit.timeit(lambda: module.crc32_raw(frame), number=20000)
    print('%s CRC 20000 calls: %.2f ms' % (module.__name__, crc_time * 1000))
    for name in ('rawimu_test.dat', 'rawimua_ascii.dat'):
        data = (ROOT / 'examples' / name).read_bytes()
        trials = []
        for _ in range(7):
            session = module.Session(dict(module.DEFAULT_CFG, auto_bias=False, auto_accel_scale=False))
            start = time.perf_counter()
            for pos in range(0, len(data), 8192):
                session.parser.feed(data[pos:pos + 8192])
            trials.append(time.perf_counter() - start)
        elapsed = statistics.median(trials)
        print('  %s: %.2f ms, %.2f MB/s, %d IMU frames' % (
            name, elapsed * 1000, len(data) / 1048576 / elapsed, session.n_imu))


def benchmark_plots(modules):
    import tkinter as tk
    root = tk.Tk()
    root.title('RAWIMU plot benchmark')
    root.geometry('800x260')
    try:
        for samples in (6000, 60000):
            for module in modules:
                canvas = tk.Canvas(root)
                canvas.pack(fill='both', expand=True)
                root.update()
                panel = module.PlotPanel(canvas, 'Plot', 'unit', ['X', 'Y', 'Z'], min_span=.1)
                panel.resize(800, 260)
                trials = []
                for frame in range(18):
                    times = [k * .001 + frame * .05 for k in range(samples)]
                    values = [[math.sin(t * k) * 10 for t in times] for k in (1, 2, 3)]
                    start = time.perf_counter()
                    panel.update(times, values)
                    root.update_idletasks()
                    trials.append(time.perf_counter() - start)
                print('%s plot %d samples (3 channels + paint): %.2f ms' % (
                    module.__name__, samples, statistics.median(trials[3:]) * 1000))
                canvas.destroy()
    finally:
        root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', help='git ref containing the original src/rawimu_live.py')
    parser.add_argument('--plots', action='store_true', help='also benchmark actual Tk drawing (opens a window)')
    args = parser.parse_args()
    modules = []
    if args.baseline:
        source = subprocess.check_output(['git', 'show', args.baseline + ':src/rawimu_live.py'], cwd=str(ROOT))
        baseline = types.ModuleType('baseline')
        exec(compile(source.decode('utf-8'), 'baseline', 'exec'), baseline.__dict__)
        modules.append(baseline)
    modules.append(load_current())
    for module in modules:
        benchmark_backend(module)
    if args.plots:
        benchmark_plots(modules)


if __name__ == '__main__':
    main()
