#!/usr/bin/env python3
"""
Diagnóstico completo del sistema de control de velocidad
Analiza qué variables están afectando el control de velocidad
"""

import serial
import time

def comprehensive_diagnosis():
    """Diagnóstico exhaustivo del sistema de velocidad"""
    
    port = "COM12"
    baudrate = 115200
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        print("="*70)
        print("DIAGNÓSTICO COMPLETO DEL SISTEMA DE CONTROL DE VELOCIDAD")
        print("="*70)
        time.sleep(2)
        
        # 1. Verificar estado inicial
        print("\n1. ESTADO INICIAL DEL SISTEMA")
        print("-" * 40)
        ser.write(b"GET\n")
        time.sleep(0.5)
        initial_data = ser.readline().decode().strip()
        print(f"Telemetría inicial: {initial_data}")
        
        if initial_data.startswith("DATA "):
            parts = initial_data.split()
            print(f"RPM L inicial: {parts[1]}")
            print(f"RPM R inicial: {parts[2]}")
            print(f"Target RPM L: {parts[5]}")
            print(f"Target RPM R: {parts[6]}")
            print(f"PWM L: {parts[7]}")
            print(f"PWM R: {parts[8]}")
            print(f"Modo: {parts[13]} (0=PID, 1=PWM)")
        
        # 2. Verificar estado de alineación
        print("\n2. ESTADO DE ALINEACIÓN")
        print("-" * 40)
        ser.write(b"ALIGN_STATUS\n")
        time.sleep(0.5)
        align_data = ser.readline().decode().strip()
        print(f"Estado alineación: {align_data}")
        
        # 3. Configurar velocidad y monitorear paso a paso
        print("\n3. CONFIGURACIÓN Y MONITOREO PASO A PASO")
        print("-" * 40)
        
        commands = [
            ("ALIGN_ENABLE", "Habilitar alineación"),
            ("VEL_FORWARD", "Configurar dirección adelante"),
            ("VEL_UP", "Primera subida de velocidad"),
            ("GET", "Leer estado después de VEL_UP"),
            ("VEL_UP", "Segunda subida de velocidad"),
            ("GET", "Leer estado después de segunda VEL_UP"),
            ("ALIGN_CALIBRATE", "Calibrar alineación"),
            ("GET", "Estado después de calibración"),
            ("ALIGN_STATUS", "Estado de alineación después de calibrar")
        ]
        
        for cmd, desc in commands:
            print(f"\n--- {desc} ({cmd}) ---")
            ser.write(f"{cmd}\n".encode())
            time.sleep(1)
            
            # Leer todas las respuestas disponibles
            responses = []
            while ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                if response:
                    responses.append(response)
            
            for response in responses:
                print(f"<<< {response}")
                
                # Análisis detallado de DATA
                if response.startswith("DATA "):
                    parts = response.split()
                    if len(parts) >= 14:
                        rpm_L = float(parts[1])
                        rpm_R = float(parts[2])
                        mmps_L = float(parts[3])
                        mmps_R = float(parts[4])
                        target_rpm_L = float(parts[5])
                        target_rpm_R = float(parts[6])
                        pwm_L = int(parts[7])
                        pwm_R = int(parts[8])
                        kp = float(parts[9])
                        ki = float(parts[10])
                        kd = float(parts[11])
                        stopped = int(parts[12])
                        mode = int(parts[13])
                        
                        print(f"    ANÁLISIS DETALLADO:")
                        print(f"    RPM Real:     L={rpm_L:6.1f}   R={rpm_R:6.1f}   Diff={abs(rpm_L-rpm_R):6.1f}")
                        print(f"    Target RPM:   L={target_rpm_L:6.1f}   R={target_rpm_R:6.1f}")
                        print(f"    PWM Output:   L={pwm_L:3d}       R={pwm_R:3d}")
                        print(f"    Velocidad mm/s: L={mmps_L:6.1f}   R={mmps_R:6.1f}")
                        print(f"    PID Params:   Kp={kp:.3f} Ki={ki:.3f} Kd={kd:.3f}")
                        print(f"    Estado:       Stopped={stopped} Mode={mode} ({'PID' if mode==0 else 'PWM'})")
                        
                        # Detectar problemas potenciales
                        if abs(rpm_L - rpm_R) > 20:
                            print(f"    ⚠️  PROBLEMA: Diferencia de RPM muy alta: {abs(rpm_L-rpm_R):.1f}")
                        
                        if abs(target_rpm_L - target_rpm_R) > 5:
                            print(f"    ⚠️  PROBLEMA: Targets RPM diferentes: L={target_rpm_L:.1f} R={target_rpm_R:.1f}")
                        
                        if abs(pwm_L - pwm_R) > 20:
                            print(f"    ⚠️  PROBLEMA: PWM muy diferentes: L={pwm_L} R={pwm_R}")
                
                # Análisis de estado de alineación
                elif response.startswith("ALIGN_STATUS "):
                    parts = response.split()
                    if len(parts) >= 7:
                        enabled = bool(int(parts[1]))
                        factor_L = float(parts[2])
                        factor_R = float(parts[3])
                        tolerance = float(parts[4])
                        difference = float(parts[5])
                        target_rpm = float(parts[6])
                        
                        print(f"    ALINEACIÓN:")
                        print(f"    Habilitada:   {enabled}")
                        print(f"    Factores:     L={factor_L:.3f}   R={factor_R:.3f}")
                        print(f"    Tolerancia:   {tolerance:.1f} RPM")
                        print(f"    Diferencia:   {difference:.1f} RPM")
                        print(f"    Target RPM:   {target_rpm:.1f}")
                        
                        if not enabled and difference > 20:
                            print(f"    ⚠️  SUGERENCIA: Habilitar alineación - diferencia alta")
                        
                        if abs(factor_L - 1.0) > 0.1 or abs(factor_R - 1.0) > 0.1:
                            print(f"    ℹ️  INFO: Factores de corrección activos")
        
        # 4. Prueba manual de valores PWM directos
        print("\n4. PRUEBA DE CONTROL PWM DIRECTO")
        print("-" * 40)
        
        ser.write(b"VEL_STOP\n")
        time.sleep(1)
        
        # Limpiar buffer
        while ser.in_waiting > 0:
            ser.readline()
        
        print("Probando PWM directo para comparar...")
        ser.write(b"PWM 100 100\n")
        time.sleep(2)
        ser.write(b"GET\n")
        time.sleep(0.5)
        
        direct_pwm_data = ser.readline().decode().strip()
        print(f"Con PWM directo 100,100: {direct_pwm_data}")
        
        if direct_pwm_data.startswith("DATA "):
            parts = direct_pwm_data.split()
            rpm_L = float(parts[1])
            rpm_R = float(parts[2])
            print(f"RPM con PWM directo: L={rpm_L:.1f} R={rpm_R:.1f} Diff={abs(rpm_L-rpm_R):.1f}")
        
        # Detener motores
        ser.write(b"VEL_STOP\n")
        time.sleep(1)
        
        print("\n" + "="*70)
        print("DIAGNÓSTICO COMPLETADO")
        print("="*70)
        
        ser.close()
        
    except Exception as e:
        print(f"Error en diagnóstico: {e}")

if __name__ == "__main__":
    comprehensive_diagnosis()