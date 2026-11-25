import unittest
import tkinter as tk
from unittest.mock import patch

from test_rawimu_live import live
import serial_ports


class SerialEnumerationTests(unittest.TestCase):
    def test_refresh_on_open_preserves_manual_selection(self):
        root = tk.Tk()
        root.withdraw()
        with patch.object(serial_ports, 'serial_port_names', side_effect=[['COM2'], ['COM2', 'COM10']]):
            app = live.App(root, dict(live.DEFAULT_CFG, port='COM90', renderer='canvas'))
            try:
                self.assertEqual(app.port_box['values'], ('COM2',))
                root.tk.call(app.port_box['postcommand'])
                self.assertEqual(app.port_box['values'], ('COM2', 'COM10'))
                self.assertEqual(app.port_var.get(), 'COM90')
            finally:
                app.on_close()

    def test_windows_merges_virtual_ports_and_updates_after_unplug(self):
        with patch.object(serial_ports.os, 'name', 'nt'), \
                patch.object(serial_ports, '_pyserial_names', side_effect=OSError()), \
                patch.object(serial_ports, '_registry_names', side_effect=[['com10'], []]), \
                patch.object(serial_ports, '_dos_names', return_value=['COM2', 'COM10x', 'COM2', 'C:']):
            self.assertEqual(serial_ports.serial_port_names(), ['COM2', 'COM10'])
            self.assertEqual(serial_ports.serial_port_names(), ['COM2'])
