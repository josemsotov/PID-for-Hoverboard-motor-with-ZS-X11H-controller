import threading
import time
import sys
from tkinter import Tk, ttk, StringVar, DoubleVar, Text, END, N, S, E, W, messagebox

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
        
        # Variables para mostrar encoder óptico
        self.encoder_revs_display = StringVar(value="0")  # Revoluciones del encoder izquierdo
        self.encoder_pulses_display = StringVar(value="0")  # Pulsos del encoder izquierdo
        
        # Variables para mostrar encoder óptico derecho
        self.encoder_revs_r_display = StringVar(value="0")  # Revoluciones del encoder derecho
        self.encoder_pulses_r_display = StringVar(value="0")  # Pulsos del encoder derecho
        
        # Variables para odometría (distancia usando PPR específicos)
        self.distance_L_display = StringVar(value="0.0")  # Distancia motor L en mm
        self.distance_R_display = StringVar(value="0.0")  # Distancia motor R en mm
        self.revolutions_L_display = StringVar(value="0.00")  # Revoluciones motor L (calculadas)
        self.revolutions_R_display = StringVar(value="0.00")  # Revoluciones motor R (calculadas)
        
        # Variables para calibración
        self.calibration_active = False
        self.calibration_start_time = 0
        self.calibration_start_encoder = 0
        self.calibration_start_encoder_r = 0  # Para encoder derecho
        self.calibration_start_pulses_L = 0
        self.calibration_start_pulses_R = 0
        self.calibrated_ppr_L = StringVar(value="0.0")
        self.calibrated_ppr_R = StringVar(value="0.0")
        self.calibration_status = StringVar(value="Listo para calibrar")
        self.calibration_progress = StringVar(value="0/10 revoluciones")
        
        # Variables para control de velocidad global
        self.robot_velocity = 0.0  # Velocidad actual del robot (-100 a 100%)
        self.velocity_increment = 10.0  # Incremento de velocidad
        self.robot_direction = 1  # 1 = adelante, -1 = atrás
        self.robot_moving = False  # Estado de movimiento
        self.robot_velocity_display = StringVar(value="0.0%")
        self.velocity_increment_display = StringVar(value="10.0%")
        
        # Variables para alineación de velocidad
        self.alignment_enabled = True
        self.speed_correction_L = 1.0
        self.speed_correction_R = 1.0
        self.alignment_tolerance = 5.0
        self.alignment_status_display = StringVar(value="Habilitado")
        self.correction_L_display = StringVar(value="1.000")
        self.correction_R_display = StringVar(value="1.000")
        self.speed_difference_display = StringVar(value="0.0 RPM")

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
        
        # Botón rápido para motor L conocido (45 PPR)
        ttk.Button(frm, text="L=45PPR", command=self.apply_motor_l_ppr).grid(row=3, column=6, padx=2)

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

        # Encoder Óptico Izquierdo (Calibración)
        ttk.Label(pulse_frame, text="Encoder L:").grid(row=2, column=0, sticky=W)
        ttk.Label(pulse_frame, text="Revoluciones:").grid(row=2, column=1, sticky=W, padx=(10, 2))
        ttk.Label(pulse_frame, textvariable=self.encoder_revs_display, relief="sunken", width=8).grid(row=2, column=2, sticky=W)
        ttk.Label(pulse_frame, text="Pulsos:").grid(row=2, column=3, sticky=W, padx=(10, 2))
        ttk.Label(pulse_frame, textvariable=self.encoder_pulses_display, relief="sunken", width=6).grid(row=2, column=4, sticky=W)
        
        # Encoder Óptico Derecho (Calibración)
        ttk.Label(pulse_frame, text="Encoder R:").grid(row=3, column=0, sticky=W)
        ttk.Label(pulse_frame, text="Revoluciones:").grid(row=3, column=1, sticky=W, padx=(10, 2))
        ttk.Label(pulse_frame, textvariable=self.encoder_revs_r_display, relief="sunken", width=8).grid(row=3, column=2, sticky=W)
        ttk.Label(pulse_frame, text="Pulsos:").grid(row=3, column=3, sticky=W, padx=(10, 2))
        ttk.Label(pulse_frame, textvariable=self.encoder_pulses_r_display, relief="sunken", width=6).grid(row=3, column=4, sticky=W)

        # Botón para resetear contadores
        ttk.Button(pulse_frame, text="Reset Pulsos", command=self.reset_pulse_counters).grid(row=0, column=5, rowspan=6, padx=(10, 0))

        # Sección de Odometría (usando PPR específicos)
        odometry_frame = ttk.LabelFrame(frm, text="Odometría (PPR Específicos)", padding=5)
        odometry_frame.grid(row=5, column=0, columnspan=6, sticky=(N, S, E, W), pady=(5, 0))
        
        # Motor L - Odometría
        ttk.Label(odometry_frame, text="Motor L:").grid(row=0, column=0, sticky=W)
        ttk.Label(odometry_frame, text="Distancia:").grid(row=0, column=1, sticky=W, padx=(10, 2))
        ttk.Label(odometry_frame, textvariable=self.distance_L_display, relief="sunken", width=8).grid(row=0, column=2, sticky=W)
        ttk.Label(odometry_frame, text="mm").grid(row=0, column=3, sticky=W)
        ttk.Label(odometry_frame, text="Revoluciones:").grid(row=0, column=4, sticky=W, padx=(10, 2))
        ttk.Label(odometry_frame, textvariable=self.revolutions_L_display, relief="sunken", width=8).grid(row=0, column=5, sticky=W)
        
        # Motor R - Odometría
        ttk.Label(odometry_frame, text="Motor R:").grid(row=1, column=0, sticky=W)
        ttk.Label(odometry_frame, text="Distancia:").grid(row=1, column=1, sticky=W, padx=(10, 2))
        ttk.Label(odometry_frame, textvariable=self.distance_R_display, relief="sunken", width=8).grid(row=1, column=2, sticky=W)
        ttk.Label(odometry_frame, text="mm").grid(row=1, column=3, sticky=W)
        ttk.Label(odometry_frame, text="Revoluciones:").grid(row=1, column=4, sticky=W, padx=(10, 2))
        ttk.Label(odometry_frame, textvariable=self.revolutions_R_display, relief="sunken", width=8).grid(row=1, column=5, sticky=W)

        # Sección de Calibración
        cal_frame = ttk.LabelFrame(frm, text="Calibración PPR con Encoders Ópticos", padding=5)
        cal_frame.grid(row=6, column=0, columnspan=6, sticky=(N, S, E, W), pady=(5, 0))

        # Estado y progreso de calibración
        ttk.Label(cal_frame, text="Estado:").grid(row=0, column=0, sticky=W)
        ttk.Label(cal_frame, textvariable=self.calibration_status, width=20).grid(row=0, column=1, sticky=W, padx=(5, 10))
        ttk.Label(cal_frame, text="Progreso:").grid(row=0, column=2, sticky=W)
        ttk.Label(cal_frame, textvariable=self.calibration_progress, width=15).grid(row=0, column=3, sticky=W, padx=(5, 10))

        # Botones de calibración
        ttk.Button(cal_frame, text="Iniciar Calibración", command=self.start_calibration).grid(row=0, column=4, padx=5)
        ttk.Button(cal_frame, text="Detener", command=self.stop_calibration).grid(row=0, column=5, padx=5)

        # Resultados de calibración
        ttk.Label(cal_frame, text="PPR Calibrado L:").grid(row=1, column=0, sticky=W, pady=(5, 0))
        ttk.Label(cal_frame, textvariable=self.calibrated_ppr_L, relief="sunken", width=8).grid(row=1, column=1, sticky=W, pady=(5, 0))
        ttk.Label(cal_frame, text="PPR Calibrado R:").grid(row=1, column=2, sticky=W, pady=(5, 0), padx=(10, 0))
        ttk.Label(cal_frame, textvariable=self.calibrated_ppr_R, relief="sunken", width=8).grid(row=1, column=3, sticky=W, pady=(5, 0))
        ttk.Button(cal_frame, text="Aplicar PPR", command=self.apply_calibrated_ppr).grid(row=1, column=4, pady=(5, 0), padx=5)
        ttk.Button(cal_frame, text="Actualizar Firmware", command=self.update_firmware_manual).grid(row=1, column=5, pady=(5, 0), padx=5)

        # Sección de Control de Velocidad del Robot
        vel_frame = ttk.LabelFrame(frm, text="Control de Velocidad del Robot", padding=5)
        vel_frame.grid(row=7, column=0, columnspan=6, sticky=(N, S, E, W), pady=(5, 0))

        # Estado actual
        ttk.Label(vel_frame, text="Velocidad Actual:").grid(row=0, column=0, sticky=W)
        ttk.Label(vel_frame, textvariable=self.robot_velocity_display, relief="sunken", width=8).grid(row=0, column=1, sticky=W, padx=(5, 10))
        ttk.Label(vel_frame, text="Incremento:").grid(row=0, column=2, sticky=W, padx=(10, 0))
        ttk.Entry(vel_frame, textvariable=self.velocity_increment_display, width=8).grid(row=0, column=3, sticky=W, padx=(5, 5))
        ttk.Button(vel_frame, text="Configurar Inc.", command=self.set_velocity_increment).grid(row=0, column=4, padx=5)

        # Botones de control direccional
        ttk.Button(vel_frame, text="↑ +Velocidad", command=self.velocity_up).grid(row=1, column=1, pady=2)
        ttk.Button(vel_frame, text="← Adelante", command=self.velocity_forward).grid(row=2, column=0, padx=2)
        ttk.Button(vel_frame, text="PARAR", command=self.velocity_stop, style="Accent.TButton").grid(row=2, column=1, padx=2)
        ttk.Button(vel_frame, text="Atrás →", command=self.velocity_backward).grid(row=2, column=2, padx=2)
        ttk.Button(vel_frame, text="↓ -Velocidad", command=self.velocity_down).grid(row=3, column=1, pady=2)

        # Sección de Alineación de Velocidad
        align_frame = ttk.LabelFrame(frm, text="Alineación de Velocidad (Motores Sincronizados)", padding=5)
        align_frame.grid(row=8, column=0, columnspan=6, sticky=(N, S, E, W), pady=(5, 0))

        # Estado de alineación
        ttk.Label(align_frame, text="Estado:").grid(row=0, column=0, sticky=W)
        ttk.Label(align_frame, textvariable=self.alignment_status_display, relief="sunken", width=12).grid(row=0, column=1, sticky=W, padx=(5, 10))
        ttk.Label(align_frame, text="Diferencia:").grid(row=0, column=2, sticky=W, padx=(10, 0))
        ttk.Label(align_frame, textvariable=self.speed_difference_display, relief="sunken", width=10).grid(row=0, column=3, sticky=W, padx=(5, 10))

        # Factores de corrección
        ttk.Label(align_frame, text="Factor L:").grid(row=1, column=0, sticky=W, pady=(5, 0))
        ttk.Label(align_frame, textvariable=self.correction_L_display, relief="sunken", width=8).grid(row=1, column=1, sticky=W, pady=(5, 0), padx=(5, 5))
        ttk.Label(align_frame, text="Factor R:").grid(row=1, column=2, sticky=W, pady=(5, 0), padx=(10, 0))
        ttk.Label(align_frame, textvariable=self.correction_R_display, relief="sunken", width=8).grid(row=1, column=3, sticky=W, pady=(5, 0), padx=(5, 5))

        # Botones de control
        ttk.Button(align_frame, text="Habilitar", command=self.enable_alignment).grid(row=0, column=4, padx=5)
        ttk.Button(align_frame, text="Deshabilitar", command=self.disable_alignment).grid(row=0, column=5, padx=5)
        ttk.Button(align_frame, text="Calibrar Auto", command=self.calibrate_alignment).grid(row=1, column=4, pady=(5, 0), padx=5)
        ttk.Button(align_frame, text="Estado", command=self.get_alignment_status).grid(row=1, column=5, pady=(5, 0), padx=5)

        # Telemetría
        self.txt = Text(frm, width=80, height=10)
        self.txt.grid(row=9, column=0, columnspan=6, sticky=(N, S, E, W))
        frm.rowconfigure(9, weight=1)
        frm.columnconfigure(5, weight=1)

        self.refresh_ports()
        self.poll_data()
        
        # Configurar control por teclado
        root.focus_set()  # Asegurar que la ventana tenga foco
        root.bind('<Key>', self.on_key_press)
        
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
            self.encoder_revs_display.set("0")
            self.encoder_pulses_display.set("0")
            self.encoder_revs_r_display.set("0")
            self.encoder_pulses_r_display.set("0")
            self.txt.insert(END, "Contadores de pulsos y encoders reseteados\n")
            self.txt.see(END)
        except Exception as e:
            self.txt.insert(END, f"Error al resetear contadores: {e}\n")
            self.txt.see(END)

    def start_calibration(self):
        """Inicia el proceso de calibración automática"""
        try:
            if self.calibration_active:
                self.txt.insert(END, "Calibración ya en progreso\n")
                self.txt.see(END)
                return
            
            # Resetear contadores antes de empezar
            self.send("RESET_PULSES")
            
            # Inicializar variables de calibración
            self.calibration_active = True
            self.calibration_start_time = time.time()
            self.calibration_start_encoder = 0
            self.calibration_start_encoder_r = 0  # Reset para encoder derecho
            self.calibration_start_pulses_L = 0
            self.calibration_start_pulses_R = 0
            
            # Actualizar interfaz
            self.calibration_status.set("Calibrando ambos motores...")
            self.calibration_progress.set("L:0/10 R:0/10 revoluciones")
            self.calibrated_ppr_L.set("0.0")
            self.calibrated_ppr_R.set("0.0")
            
            self.txt.insert(END, "=== CALIBRACIÓN INICIADA ===\n")
            self.txt.insert(END, "Gira el motor manualmente para completar 10 revoluciones del encoder\n")
            self.txt.insert(END, "El encoder debe completar exactamente 10 pulsos (10 revoluciones)\n")
            self.txt.see(END)
            
        except Exception as e:
            self.txt.insert(END, f"Error al iniciar calibración: {e}\n")
            self.txt.see(END)

    def stop_calibration(self):
        """Detiene el proceso de calibración"""
        try:
            if not self.calibration_active:
                self.txt.insert(END, "No hay calibración en progreso\n")
                self.txt.see(END)
                return
            
            self.calibration_active = False
            self.calibration_status.set("Calibración detenida")
            self.calibration_progress.set("0/10 revoluciones")
            
            self.txt.insert(END, "=== CALIBRACIÓN DETENIDA ===\n")
            self.txt.see(END)
            
        except Exception as e:
            self.txt.insert(END, f"Error al detener calibración: {e}\n")
            self.txt.see(END)

    def update_calibration(self, encoder_revs, encoder_revs_r, pulses_L, pulses_R):
        """Actualiza el progreso de calibración basándose en los datos de ambos encoders"""
        if not self.calibration_active:
            return
        
        try:
            # Calcular progreso basándose en revoluciones de ambos encoders
            target_revolutions = 10
            current_revolutions_L = encoder_revs - self.calibration_start_encoder
            current_revolutions_R = encoder_revs_r - self.calibration_start_encoder_r
            
            # Actualizar progreso mostrando ambos encoders
            self.calibration_progress.set(f"L:{current_revolutions_L}/{target_revolutions} R:{current_revolutions_R}/{target_revolutions} revoluciones")
            
            # Si hemos completado las revoluciones objetivo en AMBOS encoders
            if current_revolutions_L >= target_revolutions and current_revolutions_R >= target_revolutions:
                # Calcular PPR calibrados
                delta_pulses_L = pulses_L - self.calibration_start_pulses_L
                delta_pulses_R = pulses_R - self.calibration_start_pulses_R
                
                if current_revolutions_L > 0:
                    calibrated_ppr_L = delta_pulses_L / current_revolutions_L
                    self.calibrated_ppr_L.set(f"{calibrated_ppr_L:.1f}")
                
                if current_revolutions_R > 0:
                    calibrated_ppr_R = delta_pulses_R / current_revolutions_R
                    self.calibrated_ppr_R.set(f"{calibrated_ppr_R:.1f}")
                    
                # Finalizar calibración
                self.calibration_active = False
                self.calibration_status.set("Calibración completada para ambos motores")
                
                self.txt.insert(END, f"=== CALIBRACIÓN COMPLETADA ===\n")
                self.txt.insert(END, f"Motor L: {delta_pulses_L} pulsos en {current_revolutions_L} revs → PPR = {calibrated_ppr_L:.1f}\n")
                self.txt.insert(END, f"Motor R: {delta_pulses_R} pulsos en {current_revolutions_R} revs → PPR = {calibrated_ppr_R:.1f}\n")
                self.txt.insert(END, f"Aplicando automáticamente los valores PPR calculados...\n")
                
                # Aplicar automáticamente los valores PPR calculados
                self.auto_apply_calibrated_ppr(calibrated_ppr_L, calibrated_ppr_R)
                
                self.txt.see(END)
            
        except Exception as e:
            self.txt.insert(END, f"Error en calibración: {e}\n")
            self.txt.see(END)

    def auto_apply_calibrated_ppr(self, ppr_L, ppr_R):
        """Aplica automáticamente los valores PPR calculados sin requerir intervención del usuario"""
        try:
            self.txt.insert(END, f"Enviando PPR_L {ppr_L:.1f}...\n")
            self.send(f"PPR_L {ppr_L:.1f}")
            time.sleep(0.1)  # Pequeña pausa entre comandos
            
            self.txt.insert(END, f"Enviando PPR_R {ppr_R:.1f}...\n")
            self.send(f"PPR_R {ppr_R:.1f}")
            time.sleep(0.1)
            
            # Actualizar el display PPR promedio en la GUI
            avg_ppr = (ppr_L + ppr_R) / 2.0
            self.pulses_per_rev_var.set(avg_ppr)
            self.current_pulses_per_rev = avg_ppr
            
            self.txt.insert(END, f"✓ PPR aplicados automáticamente: L={ppr_L:.1f}, R={ppr_R:.1f}, Promedio={avg_ppr:.1f}\n")
            self.calibration_status.set(f"PPR aplicados: L={ppr_L:.1f}, R={ppr_R:.1f}")
            
            # Opcionalmente, actualizar también el código fuente
            self.update_firmware_defaults(ppr_L, ppr_R)
            
            self.txt.see(END)
            
        except Exception as e:
            self.txt.insert(END, f"Error al aplicar PPR automáticamente: {e}\n")
            self.txt.see(END)

    def update_firmware_defaults(self, ppr_L, ppr_R):
        """Actualiza los valores PPR por defecto en el código fuente del firmware"""
        try:
            import os
            firmware_path = os.path.join("..", "src", "main.cpp")
            
            # Verificar si el archivo existe
            if not os.path.exists(firmware_path):
                self.txt.insert(END, f"Advertencia: No se pudo encontrar {firmware_path} para actualizar valores por defecto\n")
                return
            
            # Leer el archivo completo
            with open(firmware_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Crear respaldo
            backup_path = firmware_path + ".backup"
            with open(backup_path, 'w', encoding='utf-8') as f:
                f.write(content)
            
            # Reemplazar los valores PPR por defecto
            import re
            
            # Buscar y reemplazar PULSES_PER_REV_L
            pattern_L = r'float PULSES_PER_REV_L = \d+\.?\d*f;'
            replacement_L = f'float PULSES_PER_REV_L = {ppr_L:.1f}f;'
            content = re.sub(pattern_L, replacement_L, content)
            
            # Buscar y reemplazar PULSES_PER_REV_R  
            pattern_R = r'float PULSES_PER_REV_R = \d+\.?\d*f;'
            replacement_R = f'float PULSES_PER_REV_R = {ppr_R:.1f}f;'
            content = re.sub(pattern_R, replacement_R, content)
            
            # Escribir el archivo actualizado
            with open(firmware_path, 'w', encoding='utf-8') as f:
                f.write(content)
            
            self.txt.insert(END, f"✓ Valores por defecto actualizados en firmware: L={ppr_L:.1f}f, R={ppr_R:.1f}f\n")
            self.txt.insert(END, f"  Respaldo guardado como: {backup_path}\n")
            self.txt.insert(END, f"  Recompila y sube el firmware para que los nuevos valores por defecto tomen efecto\n")
            
        except Exception as e:
            self.txt.insert(END, f"Error al actualizar firmware: {e}\n")

    def update_firmware_manual(self):
        """Permite al usuario actualizar manualmente el firmware con los valores PPR calibrados"""
        try:
            ppr_L = float(self.calibrated_ppr_L.get())
            ppr_R = float(self.calibrated_ppr_R.get())
            
            if ppr_L <= 0 or ppr_R <= 0:
                self.txt.insert(END, "Error: No hay valores PPR calibrados válidos. Realiza la calibración primero.\n")
                self.txt.see(END)
                return
            
            self.txt.insert(END, f"Actualizando firmware con PPR calibrados: L={ppr_L:.1f}, R={ppr_R:.1f}...\n")
            self.update_firmware_defaults(ppr_L, ppr_R)
            self.txt.see(END)
            
        except Exception as e:
            self.txt.insert(END, f"Error en actualización manual del firmware: {e}\n")
            self.txt.see(END)

    def apply_calibrated_ppr(self):
        """Aplica los valores PPR calibrados al sistema"""
        try:
            ppr_L = float(self.calibrated_ppr_L.get())
            ppr_R = float(self.calibrated_ppr_R.get())
            
            if ppr_L <= 0 or ppr_R <= 0:
                self.txt.insert(END, "Error: Valores PPR inválidos. Realiza la calibración primero.\n")
                self.txt.see(END)
                return
            
            # Aplicar PPR individuales al Arduino
            self.send(f"PPR_L {ppr_L:.1f}")
            time.sleep(0.1)  # Pequeña pausa entre comandos
            self.send(f"PPR_R {ppr_R:.1f}")
            
            # Actualizar valor promedio en la interfaz
            average_ppr = (ppr_L + ppr_R) / 2.0
            self.pulses_per_rev_var.set(average_ppr)
            self.current_pulses_per_rev = average_ppr
            
            self.txt.insert(END, f"PPR aplicados individualmente:\n")
            self.txt.insert(END, f"  Motor L: {ppr_L:.1f} PPR\n")
            self.txt.insert(END, f"  Motor R: {ppr_R:.1f} PPR\n")
            self.txt.insert(END, f"  Promedio: {average_ppr:.1f} PPR\n")
            self.txt.see(END)
            
        except ValueError:
            self.txt.insert(END, "Error: Valores PPR inválidos\n")
            self.txt.see(END)
        except Exception as e:
            self.txt.insert(END, f"Error al aplicar PPR: {e}\n")
            self.txt.see(END)

    def apply_motor_l_ppr(self):
        """Aplica PPR específico para motor izquierdo (45 pulsos)"""
        try:
            ppr_L = 45.0  # PPR conocido del motor izquierdo
            self.send(f"PPR_L {ppr_L:.1f}")
            self.txt.insert(END, f"PPR Motor L aplicado: {ppr_L:.1f}\n")
            self.txt.see(END)
        except Exception as e:
            self.txt.insert(END, f"Error al aplicar PPR_L: {e}\n")
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

    # Funciones de control de velocidad del robot
    def velocity_up(self):
        """Incrementa la velocidad del robot"""
        self.send("VEL_UP")
        self.robot_velocity += self.velocity_increment
        if self.robot_velocity > 100.0:
            self.robot_velocity = 100.0
        self.robot_moving = True
        self.update_velocity_display()

    def velocity_down(self):
        """Decrementa la velocidad del robot"""
        self.send("VEL_DOWN") 
        self.robot_velocity -= self.velocity_increment
        if self.robot_velocity < -100.0:
            self.robot_velocity = -100.0
        self.robot_moving = True
        self.update_velocity_display()

    def velocity_forward(self):
        """Configura el robot para ir hacia adelante"""
        self.send("VEL_FORWARD")
        self.robot_direction = 1
        if self.robot_velocity == 0.0:
            self.robot_velocity = self.velocity_increment
        self.robot_moving = True
        self.update_velocity_display()

    def velocity_backward(self):
        """Configura el robot para ir hacia atrás"""
        self.send("VEL_BACKWARD")
        self.robot_direction = -1
        if self.robot_velocity == 0.0:
            self.robot_velocity = self.velocity_increment
        self.robot_moving = True
        self.update_velocity_display()

    def velocity_stop(self):
        """Detiene completamente el robot"""
        self.send("VEL_STOP")
        self.robot_velocity = 0.0
        self.robot_moving = False
        self.update_velocity_display()

    def set_velocity_increment(self):
        """Configura el incremento de velocidad"""
        try:
            inc_str = self.velocity_increment_display.get().replace('%', '')
            inc_value = float(inc_str)
            if 1.0 <= inc_value <= 50.0:
                self.velocity_increment = inc_value
                self.send(f"VEL_INC {inc_value}")
            else:
                messagebox.showerror("Error", "El incremento debe estar entre 1% y 50%")
        except ValueError:
            messagebox.showerror("Error", "Valor de incremento no válido")

    def update_velocity_display(self):
        """Actualiza el display de velocidad"""
        direction_str = "↑" if self.robot_direction > 0 else "↓"
        velocity_str = f"{direction_str}{abs(self.robot_velocity):.1f}%"
        self.robot_velocity_display.set(velocity_str)

    # Funciones de alineación de velocidad
    def enable_alignment(self):
        """Habilita la alineación automática de velocidad"""
        self.send("ALIGN_ENABLE")
        self.alignment_enabled = True
        self.alignment_status_display.set("Habilitado")

    def disable_alignment(self):
        """Deshabilita la alineación automática de velocidad"""
        self.send("ALIGN_DISABLE")
        self.alignment_enabled = False
        self.alignment_status_display.set("Deshabilitado")

    def calibrate_alignment(self):
        """Calibra automáticamente los factores de corrección"""
        self.send("ALIGN_CALIBRATE")
        # Solicitar estado actualizado después de calibrar
        self.root.after(500, self.get_alignment_status)

    def get_alignment_status(self):
        """Solicita el estado actual de alineación"""
        self.send("ALIGN_STATUS")

    def update_alignment_display(self, enabled, factor_L, factor_R, tolerance, difference, target_rpm):
        """Actualiza los displays de alineación"""
        self.alignment_status_display.set("Habilitado" if enabled else "Deshabilitado")
        self.correction_L_display.set(f"{factor_L:.3f}")
        self.correction_R_display.set(f"{factor_R:.3f}")
        self.speed_difference_display.set(f"{difference:.1f} RPM")
        
        # Actualizar variables internas
        self.alignment_enabled = enabled
        self.speed_correction_L = factor_L
        self.speed_correction_R = factor_R

    def on_key_press(self, event):
        """Maneja las teclas presionadas para control del robot"""
        key = event.keysym
        
        # Solo procesar si hay conexión
        if not self.client.ser:
            return
            
        # Control de velocidad con flechas
        if key == 'Up':
            self.velocity_up()
        elif key == 'Down':
            self.velocity_down()
        elif key == 'Left':
            self.velocity_forward()
        elif key == 'Right':
            self.velocity_backward()
        elif key == 'space':
            self.velocity_stop()
        # Teclas adicionales 
        elif key in ['w', 'W']:
            self.velocity_up()
        elif key in ['s', 'S']:
            self.velocity_down()
        elif key in ['a', 'A']:
            self.velocity_forward()
        elif key in ['d', 'D']:
            self.velocity_backward()
        elif key in ['x', 'X', 'Escape']:
            self.velocity_stop()

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
                            
                            if len(parts) >= 26:  # Verificar que hay suficientes datos (26 campos + DATA)
                                pwm_L = int(parts[7])           # PWM L está en posición 7
                                ppr = float(parts[14])          # PULSES_PER_REV está en posición 14
                                pulses_L = int(parts[15])       # Pulsos acumulados L está en posición 15
                                pulses_R = int(parts[16])       # Pulsos acumulados R está en posición 16
                                measured_ppr_L = float(parts[17])  # PPR medido L está en posición 17
                                measured_ppr_R = float(parts[18])  # PPR medido R está en posición 18
                                encoder_revs = int(parts[19])   # Revoluciones encoder izquierdo está en posición 19
                                encoder_pulses = int(parts[20]) # Pulsos encoder izquierdo está en posición 20
                                encoder_revs_r = int(parts[21]) # Revoluciones encoder derecho está en posición 21
                                encoder_pulses_r = int(parts[22]) # Pulsos encoder derecho está en posición 22
                                # Nuevos campos de odometría
                                dist_L_mm = float(parts[23])    # Distancia L en mm está en posición 23
                                dist_R_mm = float(parts[24])    # Distancia R en mm está en posición 24
                                revs_L = float(parts[25])       # Revoluciones L está en posición 25
                                revs_R = float(parts[26])       # Revoluciones R está en posición 26
                                
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
                                
                                # Actualizar datos del encoder izquierdo
                                self.encoder_revs_display.set(str(encoder_revs))
                                self.encoder_pulses_display.set(str(encoder_pulses))
                                
                                # Actualizar datos del encoder derecho
                                self.encoder_revs_r_display.set(str(encoder_revs_r))
                                self.encoder_pulses_r_display.set(str(encoder_pulses_r))
                                
                                # Actualizar datos de odometría
                                self.distance_L_display.set(f"{dist_L_mm:.1f}")
                                self.distance_R_display.set(f"{dist_R_mm:.1f}")
                                self.revolutions_L_display.set(f"{revs_L:.2f}")
                                self.revolutions_R_display.set(f"{revs_R:.2f}")
                                
                                # Procesar calibración si está activa
                                if self.calibration_active:
                                    # Inicializar valores de referencia en la primera lectura
                                    if self.calibration_start_encoder == 0:
                                        self.calibration_start_encoder = encoder_revs
                                        self.calibration_start_encoder_r = encoder_revs_r
                                        self.calibration_start_pulses_L = pulses_L
                                        self.calibration_start_pulses_R = pulses_R
                                    
                                    # Actualizar progreso de calibración
                                    self.update_calibration(encoder_revs, encoder_revs_r, pulses_L, pulses_R)
                                
                        except (ValueError, IndexError) as e:
                            pass  # Ignorar errores de parsing
                    
                    elif line.startswith("ALIGN_STATUS "):
                        # Procesar estado de alineación
                        self.txt.insert(END, line + "\n")
                        self.txt.see(END)
                        try:
                            parts = line.split()
                            if len(parts) >= 7:  # ALIGN_STATUS enabled factorL factorR tolerance difference target_rpm
                                enabled = bool(int(parts[1]))
                                factor_L = float(parts[2])
                                factor_R = float(parts[3])
                                tolerance = float(parts[4])
                                difference = float(parts[5])
                                target_rpm = float(parts[6])
                                
                                self.update_alignment_display(enabled, factor_L, factor_R, tolerance, difference, target_rpm)
                        except (ValueError, IndexError):
                            pass  # Ignorar errores de parsing
                    
                    else:
                        # Mostrar otras respuestas
                        self.txt.insert(END, line + "\n")
                        self.txt.see(END)
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
