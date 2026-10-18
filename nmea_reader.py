#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Monitor and byte-for-byte logger for a GNSS receiver's serial output.

GUI: python nmea_reader.py
CLI: python nmea_reader.py --console --port COM3 --baud 9600
The application receives only: it never sends configuration commands.
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
        self.thread = threading.Thread(target=self._run, daemon=False)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()

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
                stopbits=serial.STOPBITS_ONE, timeout=0.2,
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
            while not self.stop_event.is_set():
                data = handle.read(min(max(handle.in_waiting, 1), 4096))
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
            frame = ttk.Frame(root, padding=16)
            frame.pack(fill="both", expand=True)
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
            self.refresh_ports()
            root.after(80, self.poll)

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
                    if kind == "opened":
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
