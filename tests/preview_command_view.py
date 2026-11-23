"""Render manual UDP commands against a local echo peer; no hardware access."""
import socket
import sys
import threading
import tkinter as tk
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
import rawimu_live as live


def main():
    peer = socket.socket(type=socket.SOCK_DGRAM)
    peer.bind(('127.0.0.1', 0))
    peer.settimeout(3)
    root = tk.Tk()
    app = live.App(root, dict(live.DEFAULT_CFG, renderer='canvas'), autostart=False)
    root.geometry('1560x900+16+8')
    with socket.socket(type=socket.SOCK_DGRAM) as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    app.source_var.set('UDP 接收')
    app.on_source_changed()
    app.host_var.set('127.0.0.1')
    app.network_port_var.set(str(port))
    app.toggle_port()
    app.worker.ready.wait(2)
    app.notebook.select(app.command_page)
    view = app.command_view
    view.target_var.set('custom')
    view.target_host_var.set('127.0.0.1')
    view.target_port_var.set(str(peer.getsockname()[1]))
    view.input.insert('1.0', 'PING')
    def echo():
        data, source = peer.recvfrom(1024)
        print('Loopback command:', repr(data))
        peer.sendto(b'<OK: loopback command received\r\n', source)
    echo_thread = threading.Thread(target=echo, daemon=True)
    echo_thread.start()
    root.after(150, view.send)
    def capture():
        app.refresh()
        root.update_idletasks()
        print('Screenshot:', app.save_snapshot(str(ROOT/'docs/images/ui_python_command_view.png')))
        worker = app.worker
        worker.stop()
        worker.join(2)
        echo_thread.join(1)
        peer.close()
        app.on_close()
    root.after(1300, capture)
    root.mainloop()


if __name__ == '__main__':
    main()
