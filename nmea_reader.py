#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Monitor and byte-for-byte logger for a GNSS receiver's serial output.

GUI: python nmea_reader.py
CLI: python nmea_reader.py --console --port COM3 --baud 9600
The GUI can configure Air530Z using the same serial connection.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

# The release ZIP includes an unmodified copy of pySerial and its license.
VENDOR_DIR = Path(__file__).resolve().parent / "vendor"
if VENDOR_DIR.is_dir():
    sys.path.insert(0, str(VENDOR_DIR))

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit(
        "Brak biblioteki pySerial. Rozpakuj caly pakiet razem z folderem vendor\n"
        "lub zainstaluj biblioteke: python -m pip install pyserial==3.5"
    )


def calculate_checksum(body: str) -> str:
    """XOR of the ASCII body, without the $ and * delimiters."""
    checksum = 0
    for byte in body.encode("ascii"):
        checksum ^= byte
    return f"{checksum:02X}"


class Air530ZConfigurator:
    """Use from the serial owner's worker only; commands have no ACK."""

    def __init__(self, ser: serial.Serial):
        self.ser = ser

    def _send(self, body: str) -> str:
        if not self.ser.is_open:
            raise serial.SerialException("Port nie jest otwarty.")
        cmd = f"${body}*{calculate_checksum(body)}\r\n"
        written = self.ser.write(cmd.encode('ascii'))
        if written != len(cmd):
            raise serial.SerialException("Niepełny zapis komendy.")
        self.ser.flush()
        return cmd.rstrip("\r\n")

    def set_baudrate(self, baud):
        codes = {9600: 0, 115200: 5}
        if baud not in codes:
            raise ValueError("Dozwolone prędkości: 9600, 115200.")
        cmd = self._send(f"PCAS01,{codes[baud]}")
        time.sleep(0.2)  # Runs in Capture's worker, never in Tk's thread.
        self.ser.baudrate = baud
        return cmd

    def set_system_mode(self, mode):
        if mode not in range(1, 8):
            raise ValueError("Tryb systemów musi być z zakresu 1–7.")
        return self._send(f"PCAS04,{mode}")

    def set_update_rate(self, ms):
        if ms not in (1000, 500, 200, 100):
            raise ValueError("Dozwolone interwały: 1000, 500, 200, 100 ms.")
        return self._send(f"PCAS02,{ms}")

    def set_nmea_sentences(self, bitmask):
        # Eight flags in GUI order; reserved five fields are always zero.
        flags = list(bitmask)
        if len(flags) != 8 or any(flag not in (0, 1) for flag in flags):
            raise ValueError("Wymagane 8 wartości binarnych GGA…ANT.")
        return self._send("PCAS03," + ",".join(str(int(f)) for f in flags + [0] * 5))

    def restart(self, mode):
        if mode not in range(4):
            raise ValueError("Tryb restartu musi być z zakresu 0–3.")
        return self._send(f"PCAS10,{mode}")

    def query_firmware(self):
        return self._send("PCAS06,0")

    def set_satellite_mask(self, system, mask_hex):
        if system not in (1, 2, 4, 5):
            raise ValueError("Dozwolone systemy maski: 1, 2, 4, 5.")
        if (len(mask_hex) != 8 or
                any(c not in "0123456789abcdefABCDEF" for c in mask_hex)):
            raise ValueError("Maska musi zawierać dokładnie 8 cyfr hex.")
        return self._send(f"PCAS15,{system},{mask_hex.upper()}")


def default_folder() -> Path:
    return Path.home() / "NMEA_zapisy"


def new_log_path(folder: Path) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return folder / f"nmea_{stamp}.nmea"


class Capture:
    """Serial I/O and log writes run off the GUI thread; events are FIFO."""

    def __init__(self, port: str, baud: int, log_path: Path):
        self.port = port
        self.baud = baud
        self.log_path = Path(log_path)
        self.events: queue.Queue = queue.Queue()
        self.stop_event = threading.Event()
        self.commands: queue.Queue = queue.Queue()
        self.firmware_deadline = None
        self.firmware_buffer = b""
        self.firmware_found = False
        self.thread = threading.Thread(target=self._run, daemon=False)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()

    def configure(self, method: str, *args) -> None:
        if method not in ("set_baudrate", "set_system_mode", "set_update_rate",
                          "set_nmea_sentences", "restart", "query_firmware",
                          "set_satellite_mask"):
            raise ValueError("Nieznana operacja konfiguracji.")
        if self.stop_event.is_set():
            raise ValueError("Połączenie jest zamykane.")
        self.commands.put((method, args))

    def _configure_pending(self, configurator):
        try:
            method, args = self.commands.get_nowait()
        except queue.Empty:
            return
        try:
            cmd = getattr(configurator, method)(*args)
            if method == "set_baudrate":
                self.baud = args[0]
            if method == "query_firmware":
                self.firmware_deadline = time.monotonic() + 5
                self.firmware_buffer = b""
                self.firmware_found = False
            self.events.put(("configured", (cmd, self.baud)))
        except (ValueError, OSError, serial.SerialException) as exc:
            self.events.put(("configuration_error", str(exc)))
            # After a failed baud change host/device speeds may disagree.
            if method == "set_baudrate":
                raise

    def _observe_firmware(self, data):
        if self.firmware_deadline is None:
            return
        self.firmware_buffer += data
        while b"\n" in self.firmware_buffer:
            line, self.firmware_buffer = self.firmware_buffer.split(b"\n", 1)
            line = line.rstrip(b"\r")
            # Observe existing input only; never read separately or consume NMEA.
            if line.startswith(b"$"):
                header = line.split(b",", 1)[0]
                if header.endswith(b"TXT") or header in (b"$PCAS06", b"$PCTXT", b"$GPTEXT"):
                    self.events.put(("firmware", line.decode("ascii", errors="replace")))
                    self.firmware_found = True
        self.firmware_buffer = self.firmware_buffer[-4096:]
        if time.monotonic() >= self.firmware_deadline:
            if not self.firmware_found:
                self.events.put(("firmware", "Nie zaobserwowano odpowiedzi FW w ciągu 5 s. Sprawdź też podgląd NMEA."))
            self.firmware_deadline = None
            self.firmware_buffer = b""

    def _run(self) -> None:
        handle = None
        log = None
        total_bytes = total_lines = 0
        failed = False
        created = False
        try:
            handle = serial.Serial(
                port=None, baudrate=self.baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.2, write_timeout=1,
                xonxoff=False, rtscts=False, dsrdtr=False,
            )
            handle.dtr = False
            handle.rts = False
            handle.port = self.port
            handle.open()
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            # Exclusive binary mode prevents overwrite and CR/LF conversion.
            log = self.log_path.open("xb")
            created = True
            self.events.put(("opened", str(self.log_path)))
            configurator = Air530ZConfigurator(handle)
            while not self.stop_event.is_set():
                self._configure_pending(configurator)
                data = handle.read(min(max(handle.in_waiting, 1), 4096))
                self._observe_firmware(data)
                if not data:
                    continue
                # Never filter by sentence type, checksum, fix status or encoding.
                log.write(data)
                log.flush()
                total_bytes += len(data)
                total_lines += data.count(b"\n")
                self.events.put(("data", data))
        except (serial.SerialException, OSError, ValueError) as exc:
            failed = True
            self.events.put(("error", str(exc)))
        finally:
            for resource in (log, handle):
                if resource is not None:
                    try:
                        resource.close()
                    except (OSError, serial.SerialException) as exc:
                        failed = True
                        self.events.put(("error", str(exc)))
            self.events.put(("closed", {
                "bytes": total_bytes, "lines": total_lines,
                "failed": failed, "created": created,
                "path": str(self.log_path),
            }))


def run_console(port: str, baud: int, output: Path) -> int:
    capture = Capture(port, baud, output)
    capture.start()
    print(f"Port: {port}, {baud} bit/s, 8N1. Zatrzymanie: Ctrl+C.", file=sys.stderr)
    received = 0
    opened_at = None
    no_data_reported = False
    while True:
        try:
            try:
                kind, value = capture.events.get(timeout=0.25)
            except queue.Empty:
                if opened_at and not received and not no_data_reported:
                    if time.monotonic() - opened_at > 5:
                        print("Brak danych. Sprawdz port, baud rate oraz TX -> RX.", file=sys.stderr)
                        no_data_reported = True
                continue
            if kind == "opened":
                opened_at = time.monotonic()
                print(f"Zapis: {value}", file=sys.stderr)
            elif kind == "data":
                received += len(value)
                # Binary stdout preserves the stream when redirected to a file.
                if hasattr(sys.stdout, "buffer"):
                    sys.stdout.buffer.write(value)
                    sys.stdout.buffer.flush()
                else:
                    sys.stdout.write(value.decode("ascii", errors="replace"))
                    sys.stdout.flush()
            elif kind == "error":
                print(f"Blad: {value}", file=sys.stderr)
            elif kind == "closed":
                capture.thread.join()
                print(f"Koniec. Zapisano {value['bytes']} bajtow, {value['lines']} znakow LF.", file=sys.stderr)
                return 1 if value["failed"] else 0
        except KeyboardInterrupt:
            capture.stop()
        except (BrokenPipeError, OSError):
            capture.stop()
            capture.thread.join()
            return 1


def run_gui() -> int:
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
    except ImportError:
        print("Brak Tkinter. Zainstaluj Python z obsluga Tcl/Tk lub uzyj --console.", file=sys.stderr)
        return 1

    class Window:
        MAX_VIEW_CHARS = 250_000

        def __init__(self, root):
            self.root = root
            self.capture = None
            self.closing = False
            self.opened_at = None
            self.last_data_at = None
            self.total_bytes = 0
            self.total_lines = 0
            self.view_chars = 0
            self.folder = default_folder()
            self.port = tk.StringVar()
            self.baud = tk.StringVar(value="9600")
            self.status = tk.StringVar(value="Podłącz odbiornik, wybierz port i kliknij Połącz.")
            self.stats = tk.StringVar(value="0 bajtów   |   0 wierszy zakończonych LF")
            self.log_name = tk.StringVar(value=f"Folder zapisu: {self.folder}")
            self.follow = tk.BooleanVar(value=True)
            root.title("NMEA Reader — pełny strumień GNSS")
            root.geometry("1080x680")
            root.minsize(780, 500)
            root.protocol("WM_DELETE_WINDOW", self.close)
            style = ttk.Style(root)
            if "vista" in style.theme_names():
                style.theme_use("vista")
            style.configure("TButton", padding=(10, 6))
            self.config_ready = False
            self.config_busy = False
            self.notebook = ttk.Notebook(root)
            self.notebook.pack(fill="both", expand=True)
            frame = ttk.Frame(self.notebook, padding=16)
            self.notebook.add(frame, text="Odbiór NMEA")
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(4, weight=1)
            ttk.Label(frame, text="NMEA Reader", font=("Segoe UI", 20, "bold")).grid(row=0, column=0, sticky="w")
            ttk.Label(frame, text="Pełny odbiór z portu szeregowego • automatyczny zapis do pliku .nmea").grid(row=1, column=0, sticky="w", pady=(2, 14))

            controls = ttk.Frame(frame)
            controls.grid(row=2, column=0, sticky="ew")
            controls.columnconfigure(1, weight=1)
            ttk.Label(controls, text="Port").grid(row=0, column=0, padx=(0, 8))
            self.ports = ttk.Combobox(controls, textvariable=self.port, width=38)
            self.ports.grid(row=0, column=1, sticky="ew")
            self.refresh_btn = ttk.Button(controls, text="Odśwież", command=self.refresh_ports)
            self.refresh_btn.grid(row=0, column=2, padx=8)
            ttk.Label(controls, text="bit/s").grid(row=0, column=3, padx=(8, 6))
            self.rates = ttk.Combobox(controls, textvariable=self.baud, width=9, state="readonly", values=("4800", "9600", "19200", "38400", "57600", "115200"))
            self.rates.grid(row=0, column=4)
            self.connect_btn = ttk.Button(controls, text="Połącz", command=self.toggle)
            self.connect_btn.grid(row=0, column=5, padx=(12, 0))

            toolbar = ttk.Frame(frame)
            toolbar.grid(row=3, column=0, sticky="ew", pady=(12, 8))
            self.folder_btn = ttk.Button(toolbar, text="Folder zapisu…", command=self.choose_folder)
            self.folder_btn.pack(side="left")
            ttk.Button(toolbar, text="Otwórz folder", command=self.open_folder).pack(side="left", padx=6)
            ttk.Button(toolbar, text="Wyczyść podgląd", command=self.clear).pack(side="left")
            ttk.Button(toolbar, text="Kopiuj podgląd", command=self.copy).pack(side="left", padx=6)
            ttk.Checkbutton(toolbar, text="Przewijaj", variable=self.follow).pack(side="right")

            panel = ttk.Frame(frame)
            panel.grid(row=4, column=0, sticky="nsew")
            panel.columnconfigure(0, weight=1)
            panel.rowconfigure(0, weight=1)
            self.text = tk.Text(panel, wrap="none", font=("Consolas", 10), bg="#101B28", fg="#DAE8F3", selectbackground="#285E85", insertbackground="white", relief="flat", padx=12, pady=10, state="disabled", undo=False)
            self.text.grid(row=0, column=0, sticky="nsew")
            vertical = ttk.Scrollbar(panel, orient="vertical", command=self.text.yview)
            vertical.grid(row=0, column=1, sticky="ns")
            horizontal = ttk.Scrollbar(panel, orient="horizontal", command=self.text.xview)
            horizontal.grid(row=1, column=0, sticky="ew")
            self.text.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
            ttk.Label(frame, textvariable=self.status, wraplength=980).grid(row=5, column=0, sticky="w", pady=(10, 2))
            ttk.Label(frame, textvariable=self.stats).grid(row=6, column=0, sticky="w")
            ttk.Label(frame, textvariable=self.log_name, wraplength=980).grid(row=7, column=0, sticky="w", pady=(4, 0))
            ttk.Label(frame, text="Podgląd: ostatnie 250 tys. znaków. Plik zawiera wszystkie odebrane bajty.").grid(row=8, column=0, sticky="w", pady=(5, 0))
            self.build_configuration()
            self.refresh_ports()
            root.after(80, self.poll)

        def build_configuration(self):
            page = ttk.Frame(self.notebook, padding=12)
            self.notebook.add(page, text="Konfiguracja odbiornika")
            page.columnconfigure(0, weight=1)
            page.columnconfigure(1, weight=1)
            self.config_buttons = []
            self.config_status = tk.StringVar(value="Otwórz port w zakładce Odbiór NMEA.")
            self.firmware = tk.StringVar(value="Wersja FW: —")
            self.config_baud = tk.StringVar(value="9600")
            self.system_mode = tk.StringVar(value="GPS+BDS (3)")
            self.update_rate = tk.StringVar(value="1 Hz (1000 ms)")
            self.restart_mode = tk.StringVar(value="Hot Start (0)")
            self.mask_system = tk.StringVar(value="GPS (1)")
            self.mask_hex = tk.StringVar(value="00000001")
            self.nmea_flags = [tk.BooleanVar(value=True) for _ in range(8)]

            def section(title, row, col):
                box = ttk.LabelFrame(page, text=title, padding=8)
                box.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)
                return box

            def combo(box, variable, values):
                widget = ttk.Combobox(box, textvariable=variable, values=values,
                                      state="readonly", width=25)
                widget.pack(fill="x", pady=3)
                return widget

            def button(box, text, action):
                widget = ttk.Button(box, text=text, command=action, state="disabled")
                widget.pack(fill="x", pady=3)
                self.config_buttons.append(widget)

            box = section("Prędkość transmisji", 0, 0)
            combo(box, self.config_baud, ("9600", "115200"))
            button(box, "Zmień prędkość", lambda: self.submit_configuration(
                "set_baudrate", int(self.config_baud.get())))
            box = section("Systemy satelitarne", 0, 1)
            systems = ("Tylko GPS (1)", "Tylko BDS (2)", "GPS+BDS (3)",
                       "GLONASS (4)", "GPS+GLONASS (5)", "BDS+GLONASS (6)", "Wszystkie (7)")
            combo(box, self.system_mode, systems)
            button(box, "Zastosuj tryb", lambda: self.submit_configuration(
                "set_system_mode", systems.index(self.system_mode.get()) + 1))
            box = section("Częstotliwość aktualizacji", 1, 0)
            rates = ("1 Hz (1000 ms)", "2 Hz (500 ms)", "5 Hz (200 ms)", "10 Hz (100 ms)")
            combo(box, self.update_rate, rates)
            button(box, "Ustaw częstotliwość", lambda: self.submit_configuration(
                "set_update_rate", (1000, 500, 200, 100)[rates.index(self.update_rate.get())]))
            box = section("Zdania NMEA", 1, 1)
            flags = ttk.Frame(box)
            flags.pack(fill="x")
            for i, name in enumerate(("GGA", "RMC", "GSA", "GSV", "GLL", "VTG", "ZDA", "ANT")):
                ttk.Checkbutton(flags, text=name, variable=self.nmea_flags[i]).grid(
                    row=i // 4, column=i % 4, sticky="w")
            button(box, "Zapisz wybór zdań", lambda: self.submit_configuration(
                "set_nmea_sentences", [int(v.get()) for v in self.nmea_flags]))
            box = section("Restart i diagnostyka", 2, 0)
            ttk.Label(box, text="Tryb restartu").pack(anchor="w")
            restarts = ("Hot Start (0)", "Warm Start (1)", "Cold Start (2)", "Factory Start (3)")
            combo(box, self.restart_mode, restarts)
            button(box, "Wykonaj restart", lambda: self.submit_configuration(
                "restart", restarts.index(self.restart_mode.get())))
            button(box, "Odczytaj wersję FW", lambda: self.submit_configuration("query_firmware"))
            box = section("Maska satelitów (zaawansowane)", 2, 1)
            ttk.Label(box, text="System").pack(anchor="w")
            masks = ("GPS (1)", "BDS (2)", "SBAS (4)", "QZSS (5)")
            combo(box, self.mask_system, masks)
            ttk.Label(box, text="Maska (8 hex)").pack(anchor="w")
            ttk.Entry(box, textvariable=self.mask_hex).pack(fill="x", pady=3)
            button(box, "Zastosuj maskę", lambda: self.submit_configuration(
                "set_satellite_mask", (1, 2, 4, 5)[masks.index(self.mask_system.get())],
                self.mask_hex.get()))
            ttk.Label(page, textvariable=self.config_status, wraplength=730).grid(
                row=3, column=0, columnspan=2, sticky="w", pady=8)
            ttk.Label(page, text="Moduł nie wysyła potwierdzeń komend.").grid(
                row=4, column=0, columnspan=2, sticky="w")
            ttk.Label(page, textvariable=self.firmware, wraplength=730).grid(
                row=5, column=0, columnspan=2, sticky="w", pady=6)

        def update_config_buttons(self):
            enabled = (self.config_ready and not self.config_busy and self.capture
                       and not self.capture.stop_event.is_set() and not self.closing)
            for button in self.config_buttons:
                button.configure(state="normal" if enabled else "disabled")

        def submit_configuration(self, method, *args):
            if not self.config_ready or self.config_busy or not self.capture or self.closing:
                return
            try:
                self.capture.configure(method, *args)
            except ValueError as exc:
                self.config_status.set(f"Błąd konfiguracji: {exc}")
                return
            self.config_busy = True
            self.update_config_buttons()
            if method == "query_firmware":
                self.firmware.set("Wersja FW: nasłuchiwanie strumienia przez 5 s…")

        def refresh_ports(self):
            current = self.port.get().split(" | ", 1)[0].strip()
            try:
                available = sorted(list_ports.comports(), key=lambda p: p.device)
            except (OSError, serial.SerialException) as exc:
                messagebox.showerror("Porty COM", str(exc))
                return
            values = [f"{p.device} | {p.description}" for p in available]
            self.ports["values"] = values
            match = next((s for s in values if s.split(" | ", 1)[0] == current), None)
            if match:
                self.port.set(match)
            elif not current and values:
                self.port.set(values[0])
            if not values:
                self.status.set("Brak wykrytych portów. Podłącz adapter USB–UART i kliknij Odśwież. Możesz też wpisać port ręcznie.")
            else:
                self.status.set("Wybierz port adaptera USB–UART. Air530: domyślnie 9600 bit/s, 8N1.")

        def set_connected(self, active):
            self.ports.configure(state="disabled" if active else "normal")
            self.rates.configure(state="disabled" if active else "readonly")
            for button in (self.refresh_btn, self.folder_btn):
                button.configure(state="disabled" if active else "normal")
            self.connect_btn.configure(state="normal", text="Rozłącz" if active else "Połącz")

        def toggle(self):
            if self.capture:
                self.capture.stop()
                self.update_config_buttons()
                self.connect_btn.configure(state="disabled")
                self.status.set("Zamykanie połączenia i pliku…")
                return
            port = self.port.get().split(" | ", 1)[0].strip()
            if not port:
                messagebox.showwarning("Wybierz port", "Wybierz lub wpisz port, np. COM3.")
                return
            try:
                baud = int(self.baud.get())
                if baud <= 0:
                    raise ValueError
            except ValueError:
                messagebox.showerror("Prędkość portu", "Podaj dodatnią liczbę, np. 9600.")
                return
            self.clear()
            self.total_bytes = self.total_lines = 0
            self.stats.set("0 bajtów   |   0 wierszy zakończonych LF")
            self.opened_at = self.last_data_at = None
            self.capture = Capture(port, baud, new_log_path(self.folder))
            self.set_connected(True)
            self.status.set(f"Otwieranie {port}…")
            self.capture.start()

        def choose_folder(self):
            chosen = filedialog.askdirectory(title="Wybierz folder zapisu NMEA", initialdir=str(self.folder if self.folder.exists() else Path.home()))
            if chosen:
                self.folder = Path(chosen)
                self.log_name.set(f"Folder zapisu: {self.folder}")

        def open_folder(self):
            try:
                self.folder.mkdir(parents=True, exist_ok=True)
                if os.name == "nt":
                    os.startfile(str(self.folder))
                else:
                    subprocess.Popen(["open" if sys.platform == "darwin" else "xdg-open", str(self.folder)])
            except OSError as exc:
                messagebox.showerror("Folder zapisu", str(exc))

        def clear(self):
            self.text.configure(state="normal")
            self.text.delete("1.0", "end")
            self.text.configure(state="disabled")
            self.view_chars = 0

        def copy(self):
            self.root.clipboard_clear()
            self.root.clipboard_append(self.text.get("1.0", "end-1c"))

        def display(self, data):
            # Only the visual representation is decoded; the file is raw bytes.
            value = data.decode("ascii", errors="replace").replace("\r", "")
            self.text.configure(state="normal")
            self.text.insert("end", value)
            self.view_chars += len(value)
            excess = self.view_chars - self.MAX_VIEW_CHARS
            if excess > 0:
                self.text.delete("1.0", f"1.0+{excess}c")
                self.view_chars = self.MAX_VIEW_CHARS
            self.text.configure(state="disabled")
            if self.follow.get():
                self.text.see("end")

        def poll(self):
            if self.capture:
                chunks = []
                for _ in range(200):
                    try:
                        kind, value = self.capture.events.get_nowait()
                    except queue.Empty:
                        break
                    if kind == "configured":
                        cmd, baud = value
                        self.baud.set(str(baud))
                        self.config_baud.set(str(baud))
                        self.config_status.set(f"Wysłano komendę: {cmd}")
                        self.config_busy = False
                        self.update_config_buttons()
                    elif kind == "configuration_error":
                        self.config_status.set(f"Błąd konfiguracji: {value}")
                        self.config_busy = False
                        self.update_config_buttons()
                    elif kind == "firmware":
                        self.firmware.set(value)
                    elif kind == "opened":
                        self.config_ready = True
                        self.config_busy = False
                        self.config_baud.set(self.baud.get() if self.baud.get() in ("9600", "115200") else "9600")
                        self.config_status.set("Port otwarty. Możesz wysyłać komendy.")
                        self.update_config_buttons()
                        self.opened_at = time.monotonic()
                        self.log_name.set(f"Plik: {value}")
                        self.status.set("Połączono. Oczekiwanie na dane z odbiornika…")
                    elif kind == "data":
                        chunks.append(value)
                        self.total_bytes += len(value)
                        self.total_lines += value.count(b"\n")
                        self.last_data_at = time.monotonic()
                    elif kind == "error":
                        self.status.set("Błąd połączenia lub zapisu.")
                        if not self.closing:
                            messagebox.showerror("NMEA Reader", f"{value}\n\nSprawdź port i uprawnienia do folderu. Zamknij inne programy korzystające z tego portu COM.")
                    elif kind == "closed":
                        self.capture.thread.join(timeout=0.1)
                        self.capture = None
                        self.config_ready = False
                        self.config_busy = False
                        self.config_status.set("Port zamknięty. Konfiguracja niedostępna.")
                        self.update_config_buttons()
                        self.set_connected(False)
                        self.status.set("Zatrzymano z powodu błędu. Sprawdź ostatni plik." if value["failed"] else "Rozłączono. Plik został zamknięty.")
                        break
                if chunks:
                    self.display(b"".join(chunks))
                    self.stats.set(f"{self.total_bytes:,} bajtów   |   {self.total_lines:,} wierszy zakończonych LF".replace(",", " "))
                if self.capture and self.opened_at and not self.capture.stop_event.is_set():
                    since = time.monotonic() - (self.last_data_at or self.opened_at)
                    if since >= 5:
                        self.status.set(f"Brak nowych danych od {int(since)} s. Sprawdź port, prędkość i połączenie TX → RX.")
                    elif self.last_data_at:
                        self.status.set("Odbieranie i zapis całego strumienia NMEA.")
            if self.closing and self.capture is None:
                self.root.destroy()
                return
            self.root.after(80, self.poll)

        def close(self):
            self.closing = True
            self.update_config_buttons()
            if self.capture:
                self.capture.stop()
                self.status.set("Zamykanie pliku…")
                self.connect_btn.configure(state="disabled")
            else:
                self.root.destroy()

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        print(f"Nie mozna uruchomic okna: {exc}\nUzyj --console --port PORT.", file=sys.stderr)
        return 1
    Window(root)
    root.mainloop()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Odczyt i zapis pełnego strumienia NMEA; bez parametrów otwiera okno.")
    parser.add_argument("--console", action="store_true", help="odbiór w terminalu")
    parser.add_argument("--list-ports", action="store_true", help="lista portów i zakończenie")
    parser.add_argument("--port", help="port, np. COM3 lub /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600, help="prędkość portu, domyślnie 9600")
    parser.add_argument("--output", type=Path, help="nowy plik .nmea w trybie konsolowym; istniejący plik nie jest nadpisywany")
    args = parser.parse_args()
    if args.list_ports:
        ports = sorted(list_ports.comports(), key=lambda p: p.device)
        for port in ports:
            print(f"{port.device}\t{port.description}")
        if not ports:
            print("Brak wykrytych portow szeregowych.")
        return 0
    if args.console:
        if not args.port:
            parser.error("Tryb konsolowy wymaga --port, np. --port COM3")
        if args.baud <= 0:
            parser.error("--baud musi byc dodatnia liczba")
        return run_console(args.port, args.baud, args.output or new_log_path(default_folder()))
    if args.port or args.output or args.baud != 9600:
        parser.error("Parametry --port, --baud i --output wymagaja --console. W oknie wybierz je z kontrolek.")
    return run_gui()


if __name__ == "__main__":
    raise SystemExit(main())
