"""Render a loopback UDP capture with SOW labels; never contacts IMU hardware."""
import socket
import sys
import tkinter as tk
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
import rawimu_live as live


def main():
    root = tk.Tk()
    app = live.App(root, dict(live.DEFAULT_CFG, renderer='canvas', auto_bias=False,
                              auto_accel_scale=False), autostart=False)
    root.geometry('1560x880+16+16')
    with socket.socket(type=socket.SOCK_DGRAM) as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    app.source_var.set('UDP 接收')
    app.on_source_changed()
    app.host_var.set('127.0.0.1')
    app.network_port_var.set(str(port))
    app.toggle_port()
    app.worker.ready.wait(2)
    data = (ROOT/'examples/rawimu_test.dat').read_bytes()
    with socket.socket(type=socket.SOCK_DGRAM) as sender:
        for first in range(0, len(data), 18000):
            sender.sendto(data[first:first+18000], ('127.0.0.1', port))
    app.time_axis_var.set('GPS 周内秒 (s)')
    app.on_axis_changed()
    def capture():
        app.toggle_freeze()
        app.inspect_time(app._status_view['time'][-1])
        root.update_idletasks()
        path = ROOT/'docs/images/ui_python_network_sow.png'
        print('UDP received:', app.sess.n_imu)
        print('SOW:', app.chart_time_axis.seconds_at(app._status_view['time'][-1]))
        print('Screenshot:', app.save_snapshot(str(path)))
        worker = app.worker
        worker.stop()
        worker.join(2)
        app.on_close()
    root.after(1200, capture)
    root.mainloop()


if __name__ == '__main__':
    main()
