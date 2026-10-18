"""Run with DISPLAY set (for example under Xvfb); no GNSS hardware needed."""
import os
from pathlib import Path
import select
import tempfile
import time
import unittest
from unittest.mock import patch

import nmea_reader as app

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    tk = None


@unittest.skipUnless(tk and os.environ.get('DISPLAY') and os.name == 'posix',
                     'requires Tk, a display, and POSIX pseudo terminals')
class GuiIntegrationTests(unittest.TestCase):
    def test_configuration_and_receive_panel(self):
        import pty
        master, slave = pty.openpty()
        root = tk.Tk()
        captures = []
        original_capture = app.Capture

        def capture(*args):
            instance = original_capture(*args)
            captures.append(instance)
            return instance

        def children(widget):
            result = []
            for child in widget.winfo_children():
                result.append(child)
                result.extend(children(child))
            return result

        def wait_for(predicate, timeout=3):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                root.update()
                time.sleep(0.005)
            self.assertTrue(predicate())

        def exercise():
            widgets = children(root)
            notebook = next(w for w in widgets if isinstance(w, ttk.Notebook))
            self.assertEqual([notebook.tab(t, 'text') for t in notebook.tabs()],
                             ['Odbiór NMEA', 'Konfiguracja odbiornika'])
            receiver, config = notebook.winfo_children()
            config_widgets = children(config)
            buttons = {w.cget('text'): w for w in config_widgets if isinstance(w, ttk.Button)}
            self.assertEqual(len(buttons), 7)
            self.assertTrue(all(b.instate(['disabled']) for b in buttons.values()))
            checks = [w for w in config_widgets if isinstance(w, ttk.Checkbutton)]
            self.assertEqual(len(checks), 8)
            self.assertTrue(all(root.getvar(w['variable']) for w in checks))
            notebook.select(config)
            root.update()
            # All sections and the diagnostic label fit the default window.
            labels = [w for w in config_widgets if isinstance(w, ttk.Label)]
            self.assertTrue(all(w.winfo_y() + w.winfo_height() <= config.winfo_height()
                                for w in config.winfo_children()))
            rx_widgets = children(receiver)
            port, baud = [w for w in rx_widgets if isinstance(w, ttk.Combobox)]
            connect = next(w for w in rx_widgets if isinstance(w, ttk.Button)
                           and w.cget('text') == 'Połącz')
            port.set(os.ttyname(slave))
            connect.invoke()
            self.assertTrue(all(b.instate(['disabled']) for b in buttons.values()))
            wait_for(lambda: all(not b.instate(['disabled']) for b in buttons.values()))

            def status(prefix):
                return any(str(root.getvar(w['textvariable'])).startswith(prefix)
                           for w in labels if w['textvariable'])

            def send(text, expected):
                buttons[text].invoke()
                self.assertTrue(all(b.instate(['disabled']) for b in buttons.values()))
                wait_for(lambda: not buttons[text].instate(['disabled']))
                self.assertTrue(status('Wysłano komendę: ' + expected.decode().strip()))
                self.assertTrue(select.select([master], [], [], 0.5)[0])
                self.assertEqual(os.read(master, 4096), expected)

            config_combos = [w for w in config_widgets if isinstance(w, ttk.Combobox)]
            config_combos[0].set('115200')
            ticks = []

            def heartbeat():
                ticks.append(time.monotonic())
                root.after(10, heartbeat)

            root.after(10, heartbeat)
            send('Zmień prędkość', b'$PCAS01,5*19\r\n')
            self.assertGreater(len(ticks), 5)
            self.assertEqual(baud.get(), '115200')
            send('Zastosuj tryb', b'$PCAS04,3*1A\r\n')
            send('Ustaw częstotliwość', b'$PCAS02,1000*2E\r\n')
            send('Zapisz wybór zdań', b'$PCAS03,1,1,1,1,1,1,1,1,0,0,0,0,0*1E\r\n')
            send('Wykonaj restart', b'$PCAS10,0*1C\r\n')
            send('Odczytaj wersję FW', b'$PCAS06,0*1B\r\n')
            payload = b'$GPTXT,01,01,02,FW TEST*00\r\n\xffpartial'
            os.write(master, payload)
            wait_for(lambda: status('$GPTXT'))
            text = next(w for w in rx_widgets if isinstance(w, tk.Text))
            wait_for(lambda: 'FW TEST' in text.get('1.0', 'end'))
            self.assertEqual(captures[0].log_path.read_bytes(), payload)
            send('Zastosuj maskę', b'$PCAS15,1,00000001*35\r\n')
            entry = next(w for w in config_widgets if isinstance(w, ttk.Entry)
                         and not isinstance(w, ttk.Combobox))
            entry.delete(0, 'end')
            entry.insert(0, 'invalid')
            buttons['Zastosuj maskę'].invoke()
            wait_for(lambda: status('Błąd konfiguracji:'))
            self.assertFalse(select.select([master], [], [], 0)[0])
            connect.invoke()
            self.assertTrue(all(b.instate(['disabled']) for b in buttons.values()))
            wait_for(lambda: connect.cget('text') == 'Połącz')
            self.assertTrue(all(b.instate(['disabled']) for b in buttons.values()))
            self.assertFalse(captures[0].thread.is_alive())
            # Reconnect, then disconnect while a baud command is pending.
            connect.invoke()
            wait_for(lambda: not buttons['Zmień prędkość'].instate(['disabled']))
            config_combos[0].set('9600')
            buttons['Zmień prędkość'].invoke()
            connect.invoke()
            self.assertTrue(all(b.instate(['disabled']) for b in buttons.values()))
            wait_for(lambda: connect.cget('text') == 'Połącz')
            self.assertFalse(captures[1].thread.is_alive())
            self.assertTrue(all(b.instate(['disabled']) for b in buttons.values()))

        try:
            with tempfile.TemporaryDirectory() as folder, \
                    patch.object(app, 'default_folder', return_value=Path(folder)), \
                    patch.object(app, 'Capture', side_effect=capture), \
                    patch.object(tk, 'Tk', return_value=root), \
                    patch.object(root, 'mainloop', side_effect=exercise):
                self.assertEqual(app.run_gui(), 0)
        finally:
            for instance in captures:
                instance.stop()
                instance.thread.join(3)
            root.destroy()
            os.close(master)
            os.close(slave)
