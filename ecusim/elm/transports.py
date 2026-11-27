# SPDX-License-Identifier: GPL-3.0-or-later
"""Ways for a diagnostic app to reach the emulated ELM327 (DESIGN §7.7.1).

Each transport runs on its own thread, owns one ``Elm327`` (its own AT settings, like a separate real
adapter) and serves one app at a time:

- ``TcpTransport``: "Wi-Fi ELM327" (port 35000).
- ``PtyTransport``: a virtual serial port (Linux, macOS) reachable through a fixed symlink.
- ``SerialTransport``: an existing serial port (com0com pair on Windows, null-modem cable, ...).
- ``BluetoothTransport``: Serial Port Profile through BlueZ (Linux).
"""

from __future__ import annotations

import os
import select
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Callable

from .emulator import Elm327

SPP_UUID = "00001101-0000-1000-8000-00805f9b34fb"


# ====================================================================== byte channels
class Channel:
    """A connected app. ``read`` returns b"" on timeout and None when the app has gone."""

    def read(self, timeout: float) -> bytes | None:
        raise NotImplementedError

    def write(self, data: bytes) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


class SocketChannel(Channel):
    def __init__(self, sock: socket.socket):
        self.sock = sock

    def read(self, timeout: float) -> bytes | None:
        try:
            r, _, _ = select.select([self.sock], [], [], max(timeout, 0.0))
            if not r:
                return b""
            data = self.sock.recv(512)
        except OSError:
            return None
        return data or None

    def write(self, data: bytes) -> None:
        self.sock.sendall(data)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class FdChannel(Channel):
    """Master side of a pseudo-terminal."""

    def __init__(self, fd: int):
        self.fd = fd

    def read(self, timeout: float) -> bytes | None:
        try:
            r, _, _ = select.select([self.fd], [], [], max(timeout, 0.0))
            if not r:
                return b""
            return os.read(self.fd, 512)
        except OSError:  # EIO: nobody has the port open; keep waiting
            time.sleep(min(timeout, 0.1))
            return b""

    def write(self, data: bytes) -> None:
        while data:
            n = os.write(self.fd, data)
            data = data[n:]


class SerialChannel(Channel):
    def __init__(self, port):
        self.port = port

    def read(self, timeout: float) -> bytes | None:
        try:
            self.port.timeout = max(timeout, 0.0)
            first = self.port.read(1)
            if not first:
                return b""
            return first + self.port.read(self.port.in_waiting or 0)
        except Exception:  # pyserial raises SerialException / OSError when the device goes away
            return None

    def write(self, data: bytes) -> None:
        self.port.write(data)


# ====================================================================== the adapter loop
def serve(channel: Channel, emu: Elm327, stop: threading.Event, baud: int | None = None,
          idle: Callable[[], None] | None = None, log: Callable[[str, str], None] | None = None) -> None:
    """Talk ELM327 on ``channel`` until the app goes away or ``stop`` is set."""
    log = log or (lambda *_: None)

    def send(text: str) -> None:
        if not text:
            return
        data = text.encode("ascii", "replace")
        channel.write(data)
        if baud:  # pace the output like a real UART: 10 bits per byte
            time.sleep(len(data) * 10.0 / baud)

    while not stop.is_set():
        data = channel.read(0.1)
        if data is None:
            return
        if idle is not None:
            idle()
        if not data:
            continue
        for r in emu.feed(data):
            if r.command:
                log("app", r.command)
            send(r.echo)
            if r.delay > 0:
                more = channel.read(r.delay)
                if more is None:
                    return
                if more:  # the app interrupted the request
                    send(r.stopped)
                    log("elm", "STOPPED")
                    break
            send(r.body)
            log("elm", " | ".join(x for x in r.body.replace("\n", "").split("\r") if x and x != ">"))


# ====================================================================== transports
class Transport:
    kind = ""
    title = ""

    def __init__(self, emu: Elm327, log: Callable[[str, str, str], None] | None = None):
        self.emu = emu
        self._log = log or (lambda *_: None)
        self.error = ""
        self.client = ""  # who is connected, "" when nobody
        self.note = ""  # extra status text (refused clients, addresses, ...)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def log(self, who: str, text: str) -> None:
        self._log(self.kind, who, text)

    def start(self) -> None:
        self.error, self.client, self.note = "", "", ""
        self._stop.clear()
        try:
            self._open()
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
            self._close()
            raise
        self._thread = threading.Thread(target=self._guarded, name=f"elm-{self.kind}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._thread = None
        self._close()
        self.client = ""

    def _guarded(self) -> None:
        try:
            self._run()
        except Exception as e:  # report instead of dying silently
            self.error = f"{type(e).__name__}: {e}"

    def status(self) -> str:
        if self.error:
            return self.error
        if not self.running:
            return "off"
        return f"connected: {self.client}" if self.client else "waiting for app"

    # subclasses
    def _open(self) -> None: ...

    def _run(self) -> None: ...

    def _close(self) -> None: ...


class TcpTransport(Transport):
    kind, title = "tcp", "Wi-Fi / TCP"

    def __init__(self, emu: Elm327, host: str = "127.0.0.1", port: int = 35000, **kw):
        super().__init__(emu, **kw)
        self.host, self.port = host, port
        self._srv: socket.socket | None = None

    def _open(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if sys.platform != "win32":  # on Windows SO_REUSEADDR would let two programs share the port
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv = s
        s.bind((self.host, self.port))
        s.listen(4)

    def _refuse_extra(self) -> None:
        r, _, _ = select.select([self._srv], [], [], 0)
        if r:
            conn, addr = self._srv.accept()
            self.note = f"refused {addr[0]}: the adapter is already in use"
            conn.close()

    def _run(self) -> None:
        while not self._stop.is_set():
            r, _, _ = select.select([self._srv], [], [], 0.2)
            if not r:
                continue
            conn, addr = self._srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.client, self.note = addr[0], ""
            self.log("info", f"app connected from {addr[0]}")
            ch = SocketChannel(conn)
            try:
                serve(ch, self.emu, self._stop, idle=self._refuse_extra, log=self.log)
            finally:
                ch.close()
                self.log("info", "app disconnected")
                self.client = ""

    def _close(self) -> None:
        if self._srv is not None:
            self._srv.close()
            self._srv = None


def local_addresses() -> list[str]:
    """This computer's LAN addresses (for typing into a phone app). Sends nothing."""
    found = []
    for probe in ("10.255.255.255", "192.168.255.255"):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect((probe, 1))
                ip = s.getsockname()[0]
                if ip not in found and not ip.startswith("127."):
                    found.append(ip)
        except OSError:
            pass
    return found


class PtyTransport(Transport):
    kind, title = "pty", "Virtual serial port"

    def __init__(self, emu: Elm327, link: str | Path = "", **kw):
        super().__init__(emu, **kw)
        self.link = Path(link) if link else default_pty_link()
        self.device = ""
        self._master = self._slave = -1
        self._last_rx = 0.0

    def _open(self) -> None:
        import tty

        self._master, self._slave = os.openpty()
        tty.setraw(self._slave)  # no echo, no CR/LF translation: the app sees bytes as sent
        self.device = os.ttyname(self._slave)
        self.link.parent.mkdir(parents=True, exist_ok=True)
        if self.link.is_symlink():
            self.link.unlink()  # stale link from an earlier run
        elif self.link.exists():
            raise FileExistsError(f"{self.link} exists and is not a link")
        os.symlink(self.device, self.link)

    def _run(self) -> None:
        ch = FdChannel(self._master)

        def idle() -> None:
            self.client = "app" if time.monotonic() - self._last_rx < 10.0 else ""

        orig_read = ch.read

        def read(timeout: float) -> bytes | None:
            data = orig_read(timeout)
            if data:
                self._last_rx = time.monotonic()
            return data

        ch.read = read  # type: ignore[method-assign]
        serve(ch, self.emu, self._stop, idle=idle, log=self.log)

    def _close(self) -> None:
        try:
            if self.link.is_symlink() and os.readlink(self.link) == self.device:
                self.link.unlink()
        except OSError:
            pass
        for fd in (self._master, self._slave):
            if fd >= 0:
                try:
                    os.close(fd)
                except OSError:
                    pass
        self._master = self._slave = -1

    def status(self) -> str:
        s = super().status()
        return s if self.error or not self.running else f"{s} — {self.link} → {self.device}"


def default_pty_link() -> Path:
    if os.access("/dev", os.W_OK):
        return Path("/dev/ttyELM327")
    from ..persistence import default_state_dir

    return default_state_dir() / "ttyELM327"


class SerialTransport(Transport):
    kind, title = "serial", "Existing serial port"

    def __init__(self, emu: Elm327, port: str = "", baud: int = 38400, **kw):
        super().__init__(emu, **kw)
        self.port_name, self.baud = port, baud
        self._port = None

    def _open(self) -> None:
        import serial

        if not self.port_name:
            raise ValueError("no serial port chosen")
        self._port = serial.serial_for_url(self.port_name, baudrate=self.baud, timeout=0)

    def _run(self) -> None:
        ch = SerialChannel(self._port)
        self.client = self.port_name
        serve(ch, self.emu, self._stop, baud=self.baud, log=self.log)
        if not self._stop.is_set():
            self.error = f"{self.port_name} was closed or unplugged"

    def _close(self) -> None:
        if self._port is not None:
            try:
                self._port.close()
            except Exception:
                pass
            self._port = None

    def status(self) -> str:
        if self.error or not self.running:
            return super().status()
        return f"open: {self.port_name} at {self.baud} baud"


def bluetooth_unavailable() -> str:
    """Why the Bluetooth transport can't be used here, or "" if it can."""
    if not sys.platform.startswith("linux"):
        if sys.platform == "win32":
            return ("Windows: add an incoming COM port in Bluetooth settings (More Bluetooth settings → "
                    "COM Ports), then use it as an existing serial port")
        return "not supported on this system"
    try:
        import jeepney  # noqa: F401
    except ImportError:
        return "needs the Python package 'jeepney' (Arch: python-jeepney)"
    if not hasattr(socket, "AF_BLUETOOTH"):
        return "this Python was built without Bluetooth sockets"
    return ""


class BluetoothTransport(Transport):
    """Serial Port Profile server through BlueZ's ProfileManager1 (D-Bus). BlueZ accepts the RFCOMM
    connection and hands us its socket; pairing is done in the system's Bluetooth settings."""

    kind, title = "bt", "Bluetooth"
    PATH = "/org/ecusim/elm327"

    def __init__(self, emu: Elm327, name: str = "OBDII", rename: bool = False, **kw):
        super().__init__(emu, **kw)
        self.name, self.rename = name, rename
        self._conn = None
        self._adapter = ""
        self._old_alias: str | None = None
        self._sock: socket.socket | None = None
        self._serving: threading.Thread | None = None

    def _call(self, path: str, iface: str, method: str, sig: str = "", body: tuple = ()):
        from jeepney import DBusAddress, new_method_call
        from jeepney.wrappers import unwrap_msg

        addr = DBusAddress(path, bus_name="org.bluez", interface=iface)
        return unwrap_msg(self._conn.send_and_get_reply(new_method_call(addr, method, sig, body), timeout=5.0))

    def _open(self) -> None:
        reason = bluetooth_unavailable()
        if reason:
            raise RuntimeError(reason)
        from jeepney.io.blocking import open_dbus_connection

        self._conn = open_dbus_connection(bus="SYSTEM", enable_fds=True)
        objs = self._call("/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects")[0]
        adapters = sorted(p for p, ifs in objs.items() if "org.bluez.Adapter1" in ifs)
        if not adapters:
            raise RuntimeError("no Bluetooth adapter found")
        self._adapter = adapters[0]
        props = objs[self._adapter]["org.bluez.Adapter1"]
        if not props.get("Powered", ("b", False))[1]:
            raise RuntimeError("Bluetooth is switched off")
        opts = {
            "Name": ("s", self.name or "OBDII"),
            "Role": ("s", "server"),
            "RequireAuthentication": ("b", False),
            "RequireAuthorization": ("b", False),
            "AutoConnect": ("b", True),
        }
        self._call("/org/bluez", "org.bluez.ProfileManager1", "RegisterProfile", "osa{sv}",
                   (self.PATH, SPP_UUID, opts))
        if self.rename and self.name:
            self._old_alias = props.get("Alias", ("s", ""))[1]
            try:
                self._set_alias(self.name)
            except Exception as e:
                self._old_alias = None
                self.note = f"could not rename the Bluetooth adapter: {e}"

    def _set_alias(self, alias: str) -> None:
        self._call(self._adapter, "org.freedesktop.DBus.Properties", "Set", "ssv",
                   ("org.bluez.Adapter1", "Alias", ("s", alias)))

    def _run(self) -> None:
        from jeepney import MessageType, new_error, new_method_return
        from jeepney.low_level import HeaderFields

        conn = self._conn
        while not self._stop.is_set():
            try:
                msg = conn.receive(timeout=0.2)
            except TimeoutError:
                continue
            h = msg.header
            if h.message_type != MessageType.method_call or h.fields.get(HeaderFields.path) != self.PATH:
                continue
            member = h.fields.get(HeaderFields.member)
            if member == "NewConnection":
                device, fd = msg.body[0], msg.body[1]
                sock = fd.to_socket()
                if self._serving is not None and self._serving.is_alive():
                    conn.send(new_error(msg, "org.bluez.Error.Rejected", "s", ("adapter already in use",)))
                    sock.close()
                    self.note = f"refused {device.rsplit('/', 1)[-1][4:].replace('_', ':')}: already in use"
                    continue
                conn.send(new_method_return(msg))
                self._start_serving(sock, device)
            elif member == "RequestDisconnection":
                if self._sock is not None:
                    try:
                        self._sock.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass
                conn.send(new_method_return(msg))
            else:  # Release
                conn.send(new_method_return(msg))

    def _start_serving(self, sock: socket.socket, device: str) -> None:
        mac = device.rsplit("/", 1)[-1][4:].replace("_", ":")
        self._sock = sock
        self.client, self.note = mac, ""
        self.log("info", f"app connected from {mac}")

        def run() -> None:
            ch = SocketChannel(sock)
            try:
                serve(ch, self.emu, self._stop, log=self.log)
            finally:
                ch.close()
                self.log("info", "app disconnected")
                self.client = ""
                self._sock = None

        self._serving = threading.Thread(target=run, name="elm-bt-client", daemon=True)
        self._serving.start()

    def _close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self._serving is not None:
            self._serving.join(timeout=2.0)
            self._serving = None
        if self._conn is not None:
            try:
                self._call("/org/bluez", "org.bluez.ProfileManager1", "UnregisterProfile", "o", (self.PATH,))
            except Exception:
                pass
            if self._old_alias is not None:
                try:
                    self._set_alias(self._old_alias)
                except Exception:
                    pass
                self._old_alias = None
            self._conn.close()
            self._conn = None

    def status(self) -> str:
        s = super().status()
        if self.running and not self.client and not self.error:
            s = "waiting for app (pair this computer from the phone first)"
        return s
