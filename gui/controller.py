import threading
import time
import sys
from tkinter import Tk, ttk, StringVar, DoubleVar, Text, END, N, S, E, W

try:
    import serial
    from serial.tools import list_ports
except Exception as e:
    serial = None
    list_ports = None


BAUD = 115200


class SerialClient:
    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()
        self.rx_thread = None
        self.running = False
        self.last_data = ""

    def ports(self):
        if list_ports is None:
            return []
        return [p.device for p in list_ports.comports()]

    def connect(self, port):
        if serial is None:
            raise RuntimeError("pyserial no instalado")
        self.close()
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        # esperar READY
        t0 = time.time()
        buf = ""
        while time.time() - t0 < 2.0:
            line = self.readline()
            if line:
                buf += line
                if "READY" in buf:
                    return True
        return True  # continuar incluso si no se ve READY

    def _rx_loop(self):
        while self.running and self.ser:
            try:
                line = self.ser.readline().decode(errors='ignore')
                if line:
                    with self.lock:
                        self.last_data += line
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.1)

    def readline(self):
        with self.lock:
            if "\n" in self.last_data:
                i = self.last_data.find("\n")
                line = self.last_data[:i+1]
                self.last_data = self.last_data[i+1:]
                return line
        return ""

    def write_line(self, s, wait_ack=True, timeout=1.0):
        if not self.ser:
            raise RuntimeError("No conectado")
        if not s.endswith("\n"):
            s += "\n"
        self.ser.write(s.encode())
        if not wait_ack:
            return True
        t0 = time.time()
        while time.time() - t0 < timeout:
            line = self.readline()
            if not line:
                time.sleep(0.01)
                continue
            line = line.strip()
            if line == "ACK":
                return True
            if line.startswith("ERR"):
                return False
        return False

    def close(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


class App:
    def __init__(self, root):
        self.root = root
        root.title("DualMotor PID Controller")

        self.client = SerialClient()

        self.port_var = StringVar()
        self.status_var = StringVar(value="Desconectado")
        self.vel_var = DoubleVar(value=200.0)  # mm/s

        frm = ttk.Frame(root, padding=10)
        frm.grid(row=0, column=0, sticky=(N, S, E, W))
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        # Conexión
        ttk.Label(frm, text="Puerto").grid(row=0, column=0, sticky=W)
        self.cmb_ports = ttk.Combobox(frm, textvariable=self.port_var, width=20, state='readonly')
        self.cmb_ports.grid(row=0, column=1, sticky=W)
        ttk.Button(frm, text="Refrescar", command=self.refresh_ports).grid(row=0, column=2, padx=4)
        ttk.Button(frm, text="Conectar", command=self.connect).grid(row=0, column=3, padx=4)
        ttk.Label(frm, textvariable=self.status_var).grid(row=0, column=4, padx=8)

        # Control
        ttk.Label(frm, text="Velocidad (mm/s)").grid(row=1, column=0, sticky=W, pady=(8, 2))
        self.vel_entry = ttk.Entry(frm, textvariable=self.vel_var, width=10)
        self.vel_entry.grid(row=1, column=1, sticky=W)

        btns = ttk.Frame(frm)
        btns.grid(row=2, column=0, columnspan=5, pady=8)
        ttk.Button(btns, text="ADELANTE", command=self.cmd_adelante).grid(row=0, column=1, padx=4)
        ttk.Button(btns, text="GIRAR_IZQ", command=self.cmd_girar_izq).grid(row=1, column=0, padx=4)
        ttk.Button(btns, text="PARADA", command=self.cmd_parada).grid(row=1, column=1, padx=4)
        ttk.Button(btns, text="BRAKE", command=self.cmd_brake).grid(row=1, column=2, padx=4)
        ttk.Button(btns, text="GIRAR_DER", command=self.cmd_girar_der).grid(row=1, column=3, padx=4)
        ttk.Button(btns, text="ATRAS", command=self.cmd_atras).grid(row=2, column=1, padx=4)

        # Telemetría
        self.txt = Text(frm, width=80, height=12)
        self.txt.grid(row=3, column=0, columnspan=5, sticky=(N, S, E, W))
        frm.rowconfigure(3, weight=1)
        frm.columnconfigure(4, weight=1)

        self.refresh_ports()
        self.poll_data()

    def refresh_ports(self):
        ports = self.client.ports()
        self.cmb_ports['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def connect(self):
        port = self.port_var.get()
        if not port:
            self.status_var.set("Seleccione puerto")
            return
        try:
            ok = self.client.connect(port)
            self.status_var.set("Conectado" if ok else "Conectado (sin READY)")
        except Exception as e:
            self.status_var.set(f"Error: {e}")

    def send(self, cmd):
        try:
            ok = self.client.write_line(cmd, wait_ack=True)
            if not ok:
                self.status_var.set("ERR cmd")
            else:
                self.status_var.set("OK")
        except Exception as e:
            self.status_var.set(f"Error: {e}")

    # Botones
    def cmd_adelante(self):
        v = self.vel_var.get()
        self.send(f"ADELANTE {v}")

    def cmd_atras(self):
        v = self.vel_var.get()
        self.send(f"ATRAS {v}")

    def cmd_girar_izq(self):
        v = self.vel_var.get()
        self.send(f"GIRAR_IZQ {v}")

    def cmd_girar_der(self):
        v = self.vel_var.get()
        self.send(f"GIRAR_DER {v}")

    def cmd_parada(self):
        self.send("PARADA")

    def cmd_brake(self):
        self.send("BRAKE")

    def poll_data(self):
        # pedir telemetría periódicamente
        try:
            if self.client.ser:
                self.client.write_line("GET", wait_ack=False)
                # leer líneas disponibles
                while True:
                    line = self.client.readline()
                    if not line:
                        break
                    line = line.strip()
                    if line.startswith("DATA "):
                        self.txt.insert(END, line + "\n")
                        self.txt.see(END)
        except Exception:
            pass
        self.root.after(200, self.poll_data)


def main():
    root = Tk()
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
