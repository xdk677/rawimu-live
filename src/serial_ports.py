"""Enumerate serial endpoints without opening devices."""
import os
import re


def _pyserial_names():
    from serial.tools import list_ports
    return [port.device for port in list_ports.comports()]


def _registry_names():
    import winreg
    with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r'HARDWARE\DEVICEMAP\SERIALCOMM') as key:
        return [winreg.EnumValue(key, i)[1] for i in range(winreg.QueryInfoKey(key)[1])]


def _dos_names():
    import ctypes
    from ctypes import wintypes
    query = ctypes.WinDLL('kernel32', use_last_error=True).QueryDosDeviceW
    query.argtypes = (wintypes.LPCWSTR, wintypes.LPWSTR, wintypes.DWORD)
    query.restype = wintypes.DWORD
    capacity = 4096
    while True:
        buffer = ctypes.create_unicode_buffer(capacity)
        length = query(None, buffer, capacity)
        if length:
            return buffer[:length].split('\0')
        error = ctypes.get_last_error()
        if error != 122:  # ERROR_INSUFFICIENT_BUFFER
            raise ctypes.WinError(error)
        capacity *= 2


def serial_port_names():
    windows = os.name == 'nt'
    sources = (_pyserial_names, _registry_names, _dos_names) if windows else (_pyserial_names,)
    names = set()
    for source in sources:
        try:
            values = source()
        except Exception:
            # Each enumeration API can fail independently, including in a frozen app.
            continue
        for value in values:
            if windows:
                if not isinstance(value, str):
                    continue
                value = value.strip().upper()
                if not re.fullmatch(r'COM[0-9]+', value):
                    continue
            names.add(value)
    return sorted(names, key=lambda name: (int(name[3:]), name)) if windows else sorted(names)
