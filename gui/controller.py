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
        self.pwm_var = DoubleVar(value=0.0)    # PWM actual (-255 a +255)
        self.pwm_display = StringVar(value="0.00")  # Para mostrar PWM con 2 decimales
        self.increment_var = DoubleVar(value=0.01)  # Variable para el incremento
        self.pwm_increment = 0.01              # Incremento por botón (cambiado de 0.2 a 0.01)
        self.pulses_per_rev_var = DoubleVar(value=55.0)  # Pulsos por revolución
        self.current_pulses_per_rev = 55.0     # Valor actual desde Arduino
        
        # Variables para mostrar contadores de pulsos
        self.pulses_L_display = StringVar(value="0")      # Pulsos acumulados motor L
        self.pulses_R_display = StringVar(value="0")      # Pulsos acumulados motor R
        self.measured_ppr_L_display = StringVar(value="55.0")  # PPR medido motor L
        self.measured_ppr_R_display = StringVar(value="55.0")  # PPR medido motor R

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
        ttk.Button(frm, text="Desconectar", command=self.disconnect).grid(row=0, column=4, padx=4)
        ttk.Label(frm, textvariable=self.status_var).grid(row=0, column=5, padx=8)

        # Control
        ttk.Label(frm, text="Velocidad (mm/s)").grid(row=1, column=0, sticky=W, pady=(8, 2))
        self.vel_entry = ttk.Entry(frm, textvariable=self.vel_var, width=10)
        self.vel_entry.grid(row=1, column=1, sticky=W)
        
        # Control PWM Incremental
        ttk.Label(frm, text="PWM Actual").grid(row=2, column=0, sticky=W, pady=(8, 2))
        self.pwm_label = ttk.Label(frm, textvariable=self.pwm_display, width=10, relief="sunken")
        self.pwm_label.grid(row=2, column=1, sticky=W)
        ttk.Button(frm, text="RESET PWM", command=self.reset_pwm).grid(row=2, column=2, padx=4)

        # Control de Incremento PWM
        ttk.Label(frm, text="Incremento PWM").grid(row=3, column=0, sticky=W, pady=(4, 2))
        self.increment_entry = ttk.Entry(frm, textvariable=self.increment_var, width=10)
        self.increment_entry.grid(row=3, column=1, sticky=W)
        self.increment_entry.bind('<Return>', self.update_increment)
        self.increment_entry.bind('<FocusOut>', self.update_increment)
        ttk.Button(frm, text="Aplicar", command=self.update_increment).grid(row=3, column=2, padx=4)

        # Control de Pulsos por Revolución
        ttk.Label(frm, text="Pulsos/Rev").grid(row=3, column=3, sticky=W, pady=(4, 2), padx=(8, 0))
        self.pulses_entry = ttk.Entry(frm, textvariable=self.pulses_per_rev_var, width=8)
        self.pulses_entry.grid(row=3, column=4, sticky=W)
        self.pulses_entry.bind('<Return>', self.update_pulses_per_rev)
        self.pulses_entry.bind('<FocusOut>', self.update_pulses_per_rev)
        ttk.Button(frm, text="Set PPR", command=self.update_pulses_per_rev).grid(row=3, column=5, padx=4)

        btns = ttk.Frame(frm)
        btns.grid(row=4, column=0, columnspan=6, pady=8)
        ttk.Button(btns, text="ADELANTE (+PWM)", command=self.cmd_adelante).grid(row=0, column=1, padx=4)
        ttk.Button(btns, text="GIRAR_IZQ", command=self.cmd_girar_izq).grid(row=1, column=0, padx=4)
        ttk.Button(btns, text="PARADA", command=self.cmd_parada).grid(row=1, column=1, padx=4)
        ttk.Button(btns, text="BRAKE", command=self.cmd_brake).grid(row=1, column=2, padx=4)
        ttk.Button(btns, text="GIRAR_DER", command=self.cmd_girar_der).grid(row=1, column=3, padx=4)
        ttk.Button(btns, text="ATRAS (-PWM)", command=self.cmd_atras).grid(row=2, column=1, padx=4)

        # Información de Pulsos
        pulse_frame = ttk.LabelFrame(frm, text="Contadores de Pulsos", padding=5)
        pulse_frame.grid(row=5, column=0, columnspan=6, sticky=(N, S, E, W), pady=(5, 0))

        # Motor Izquierdo
        ttk.Label(pulse_frame, text="Motor L:").grid(row=0, column=0, sticky=W)
        ttk.Label(pulse_frame, text="Pulsos:").grid(row=0, column=1, sticky=W, padx=(10, 2))
        ttk.Label(pulse_frame, textvariable=self.pulses_L_display, relief="sunken", width=8).grid(row=0, column=2, sticky=W)
        ttk.Label(pulse_frame, text="PPR Medido:").grid(row=0, column=3, sticky=W, padx=(10, 2))
        ttk.Label(pulse_frame, textvariable=self.measured_ppr_L_display, relief="sunken", width=6).grid(row=0, column=4, sticky=W)

        # Motor Derecho  
        ttk.Label(pulse_frame, text="Motor R:").grid(row=1, column=0, sticky=W)
        ttk.Label(pulse_frame, text="Pulsos:").grid(row=1, column=1, sticky=W, padx=(10, 2))
        ttk.Label(pulse_frame, textvariable=self.pulses_R_display, relief="sunken", width=8).grid(row=1, column=2, sticky=W)
        ttk.Label(pulse_frame, text="PPR Medido:").grid(row=1, column=3, sticky=W, padx=(10, 2))
        ttk.Label(pulse_frame, textvariable=self.measured_ppr_R_display, relief="sunken", width=6).grid(row=1, column=4, sticky=W)

        # Botón para resetear contadores
        ttk.Button(pulse_frame, text="Reset Pulsos", command=self.reset_pulse_counters).grid(row=0, column=5, rowspan=2, padx=(10, 0))

        # Telemetría
        self.txt = Text(frm, width=80, height=10)
        self.txt.grid(row=6, column=0, columnspan=6, sticky=(N, S, E, W))
        frm.rowconfigure(6, weight=1)
        frm.columnconfigure(5, weight=1)

        self.refresh_ports()
        self.poll_data()
        
        # Manejar cierre de ventana
        root.protocol("WM_DELETE_WINDOW", self.on_closing)

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

    def disconnect(self):
        """Desconectar del puerto serial y liberar recursos"""
        try:
            # Enviar comando PARADA antes de desconectar por seguridad
            if self.client.ser:
                try:
                    self.client.write_line("PARADA", wait_ack=False)
                    time.sleep(0.1)  # Dar tiempo para que se procese
                except Exception:
                    pass  # Ignorar errores al enviar PARADA
            
            self.client.close()
            self.status_var.set("Desconectado")
            self.update_pwm_value(0.0)  # Reset PWM display
            self.txt.insert(END, "--- DESCONECTADO ---\n")
            self.txt.see(END)
        except Exception as e:
            self.status_var.set(f"Error desconectando: {e}")

    def send(self, cmd):
        if not self.client.ser:
            self.status_var.set("No conectado")
            return
        try:
            ok = self.client.write_line(cmd, wait_ack=True)
            if not ok:
                self.status_var.set("ERR cmd")
            else:
                self.status_var.set("OK")
        except Exception as e:
            self.status_var.set(f"Error: {e}")

    def update_pwm_display(self):
        """Actualiza la visualización del PWM actual"""
        pwm_val = self.pwm_var.get()
        self.pwm_display.set(f"{pwm_val:.2f}")  # Formato con 2 decimales
    
    def update_pwm_value(self, new_value):
        """Helper para actualizar PWM y su visualización"""
        self.pwm_var.set(new_value)
        self.pwm_display.set(f"{new_value:.2f}")

    def update_increment(self, event=None):
        """Actualiza el incremento de PWM desde la interfaz"""
        try:
            new_increment = self.increment_var.get()
            if 0.001 <= new_increment <= 50:  # Límites razonables
                self.pwm_increment = new_increment
                self.txt.insert(END, f"Incremento PWM actualizado a: {new_increment:.3f}\n")
                self.txt.see(END)
            else:
                # Revertir a valor válido
                self.increment_var.set(self.pwm_increment)
                self.txt.insert(END, f"Error: Incremento debe estar entre 0.001 y 50\n")
                self.txt.see(END)
        except Exception as e:
            # Revertir a valor válido en caso de error
            self.increment_var.set(self.pwm_increment)
            self.txt.insert(END, f"Error al actualizar incremento: {e}\n")
            self.txt.see(END)

    def update_pulses_per_rev(self, event=None):
        """Actualiza los pulsos por revolución en el Arduino"""
        try:
            new_ppr = self.pulses_per_rev_var.get()
            if 1 <= new_ppr <= 1000:  # Límites razonables para sensores Hall
                self.send(f"PULSES {new_ppr}")
                self.current_pulses_per_rev = new_ppr
                self.txt.insert(END, f"Pulsos por revolución actualizados a: {new_ppr:.1f}\n")
                self.txt.see(END)
            else:
                # Revertir a valor válido
                self.pulses_per_rev_var.set(self.current_pulses_per_rev)
                self.txt.insert(END, f"Error: Pulsos/Rev debe estar entre 1 y 1000\n")
                self.txt.see(END)
        except Exception as e:
            # Revertir a valor válido en caso de error
            self.pulses_per_rev_var.set(self.current_pulses_per_rev)
            self.txt.insert(END, f"Error al actualizar pulsos/rev: {e}\n")
            self.txt.see(END)

    def reset_pulse_counters(self):
        """Resetea los contadores de pulsos en el Arduino"""
        try:
            self.send("RESET_PULSES")
            self.pulses_L_display.set("0")
            self.pulses_R_display.set("0")
            self.txt.insert(END, "Contadores de pulsos reseteados\n")
            self.txt.see(END)
        except Exception as e:
            self.txt.insert(END, f"Error al resetear contadores: {e}\n")
            self.txt.see(END)

    def reset_pwm(self):
        """Resetea PWM a 0 y envía comando PARADA"""
        self.update_pwm_value(0.0)
        self.send("PARADA")

    def send_pwm_command(self):
        """Envía comando PWM al Arduino con valores actuales"""
        pwm_val = self.pwm_var.get()
        # PWM simétrico: mismo valor para ambas ruedas
        pwm_L = int(abs(pwm_val)) if pwm_val >= 0 else 0
        pwm_R = int(abs(pwm_val)) if pwm_val >= 0 else 0
        
        if pwm_val == 0:
            self.send("PARADA")
        elif pwm_val > 0:
            # Adelante: ambas ruedas positivas
            self.send(f"PWM {pwm_L} {pwm_R}")
        else:
            # Atrás: cambiar dirección (esto requiere modificar el firmware)
            # Por ahora usamos PWM negativo
            self.send(f"PWM {-int(abs(pwm_val))} {-int(abs(pwm_val))}")

    # Botones modificados para control incremental
    def cmd_adelante(self):
        """Incrementa PWM en incremento configurado (adelante)"""
        current_pwm = self.pwm_var.get()
        new_pwm = min(current_pwm + self.pwm_increment, 255)  # Límite máximo 255
        self.update_pwm_value(new_pwm)
        self.send_pwm_command()

    def cmd_atras(self):
        """Decrementa PWM en incremento configurado (atrás)"""
        current_pwm = self.pwm_var.get()
        new_pwm = max(current_pwm - self.pwm_increment, -255)  # Límite mínimo -255
        self.update_pwm_value(new_pwm)
        self.send_pwm_command()

    def cmd_girar_izq(self):
        """Giro izquierda: PWM diferencial"""
        pwm_val = abs(self.pwm_var.get())
        if pwm_val == 0:
            pwm_val = 50  # PWM mínimo para giro
        # Izquierda hacia atrás, derecha hacia adelante
        self.send(f"PWM {-int(pwm_val)} {int(pwm_val)}")

    def cmd_girar_der(self):
        """Giro derecha: PWM diferencial"""
        pwm_val = abs(self.pwm_var.get())
        if pwm_val == 0:
            pwm_val = 50  # PWM mínimo para giro
        # Izquierda hacia adelante, derecha hacia atrás
        self.send(f"PWM {int(pwm_val)} {-int(pwm_val)}")

    def cmd_parada(self):
        """Parada con secuencia: reducir PWM gradualmente, luego STOP y BRAKE"""
        self.reduce_pwm_and_stop()

    def reduce_pwm_and_stop(self):
        """Reduce PWM gradualmente hasta 0, luego activa STOP y BRAKE"""
        current_pwm = self.pwm_var.get()
        
        if abs(current_pwm) > 10:
            # Reducir PWM en pasos de 10 hasta llegar cerca de 0
            if current_pwm > 0:
                new_pwm = max(current_pwm - 10, 0)
            else:
                new_pwm = min(current_pwm + 10, 0)
            
            self.update_pwm_value(new_pwm)
            self.send_pwm_command()
            
            # Programar siguiente reducción en 100ms
            self.root.after(100, self.reduce_pwm_and_stop)
        else:
            # PWM ya está cerca de 0, activar STOP y BRAKE
            self.update_pwm_value(0.0)
            self.send("PARADA")
            # Activar BRAKE después de un breve delay
            self.root.after(200, lambda: self.send("BRAKE"))

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
                        # Parsear PWM y PPR de la telemetría para sincronizar display
                        try:
                            parts = line.split()
                            
                            if len(parts) >= 19:  # Verificar que hay suficientes datos (19 campos + DATA)
                                pwm_L = int(parts[7])           # PWM L está en posición 7
                                ppr = float(parts[14])          # PULSES_PER_REV está en posición 14
                                pulses_L = int(parts[15])       # Pulsos acumulados L está en posición 15
                                pulses_R = int(parts[16])       # Pulsos acumulados R está en posición 16
                                measured_ppr_L = float(parts[17])  # PPR medido L está en posición 17
                                measured_ppr_R = float(parts[18])  # PPR medido R está en posición 18
                                
                                # Actualizar display PWM solo si cambió externamente
                                if abs(pwm_L - self.pwm_var.get()) > 0.005:
                                    self.update_pwm_value(pwm_L)
                                
                                # Actualizar display PPR solo si cambió externamente
                                if abs(ppr - self.pulses_per_rev_var.get()) > 0.1:
                                    self.pulses_per_rev_var.set(ppr)
                                    self.current_pulses_per_rev = ppr
                                
                                # Actualizar contadores de pulsos
                                self.pulses_L_display.set(str(pulses_L))
                                self.pulses_R_display.set(str(pulses_R))
                                
                                # Actualizar PPR medidos
                                self.measured_ppr_L_display.set(f"{measured_ppr_L:.1f}")
                                self.measured_ppr_R_display.set(f"{measured_ppr_R:.1f}")
                                
                        except (ValueError, IndexError) as e:
                            pass  # Ignorar errores de parsing
        except Exception:
            pass
        self.root.after(200, self.poll_data)

    def on_closing(self):
        """Función llamada al cerrar la ventana - desconecta y cierra limpiamente"""
        try:
            self.disconnect()  # Desconectar antes de cerrar
        except Exception:
            pass
        self.root.destroy()  # Cerrar la aplicación


def main():
    root = Tk()
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
