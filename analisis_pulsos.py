import serial
import time

def analizar_pulsos():
    print("=== ANÁLISIS DETALLADO DE PULSOS ===")
    
    try:
        ser = serial.Serial('COM12', 115200, timeout=2)
        time.sleep(2)
        
        # Leer inicialización
        while ser.in_waiting:
            ser.readline()
        
        print("1. Reset inicial de contadores")
        ser.write("RESET_PULSES\n".encode())
        time.sleep(0.1)
        ser.readline()  # leer ACK
        
        print("\n2. Estado inicial (después del reset):")
        ser.write("GET\n".encode())
        time.sleep(0.1)
        resp = ser.readline().decode().strip()
        if resp.startswith('DATA '):
            parts = resp.split()
            print(f"   Pulsos L: {parts[15]}, R: {parts[16]}")
            print(f"   PPR configurado: {parts[14]}")
        
        print("\n3. Activando motor izquierdo con PWM 100 por diferentes tiempos:")
        
        # Prueba 1: 1 segundo
        print("\n   Prueba 1: Motor por 1 segundo")
        ser.write("PWM 100 0\n".encode())
        time.sleep(1)
        ser.write("PARADA\n".encode())
        time.sleep(0.1)
        
        ser.write("GET\n".encode())
        time.sleep(0.1)
        resp = ser.readline().decode().strip()
        if resp.startswith('DATA '):
            parts = resp.split()
            pulsos_1s = int(parts[15])
            print(f"   Pulsos después de 1s: {pulsos_1s}")
        
        # Reset para siguiente prueba
        ser.write("RESET_PULSES\n".encode())
        time.sleep(0.1)
        ser.readline()
        
        # Prueba 2: 2 segundos
        print("\n   Prueba 2: Motor por 2 segundos")
        ser.write("PWM 100 0\n".encode())
        time.sleep(2)
        ser.write("PARADA\n".encode())
        time.sleep(0.1)
        
        ser.write("GET\n".encode())
        time.sleep(0.1)
        resp = ser.readline().decode().strip()
        if resp.startswith('DATA '):
            parts = resp.split()
            pulsos_2s = int(parts[15])
            print(f"   Pulsos después de 2s: {pulsos_2s}")
        
        # Reset para siguiente prueba
        ser.write("RESET_PULSES\n".encode())
        time.sleep(0.1)
        ser.readline()
        
        # Prueba 3: 5 segundos
        print("\n   Prueba 3: Motor por 5 segundos")
        ser.write("PWM 100 0\n".encode())
        time.sleep(5)
        ser.write("PARADA\n".encode())
        time.sleep(0.1)
        
        ser.write("GET\n".encode())
        time.sleep(0.1)
        resp = ser.readline().decode().strip()
        if resp.startswith('DATA '):
            parts = resp.split()
            pulsos_5s = int(parts[15])
            print(f"   Pulsos después de 5s: {pulsos_5s}")
        
        print("\n4. ANÁLISIS:")
        print(f"   - Pulsos en 1s: {pulsos_1s}")
        print(f"   - Pulsos en 2s: {pulsos_2s}")
        print(f"   - Pulsos en 5s: {pulsos_5s}")
        
        if pulsos_1s > 0:
            pps_1s = pulsos_1s / 1.0  # pulsos por segundo
            pps_2s = pulsos_2s / 2.0
            pps_5s = pulsos_5s / 5.0
            
            print(f"   - Pulsos por segundo (1s): {pps_1s:.1f}")
            print(f"   - Pulsos por segundo (2s): {pps_2s:.1f}")
            print(f"   - Pulsos por segundo (5s): {pps_5s:.1f}")
            
            # Asumiendo 55 PPR configurado
            rpm_1s = (pps_1s / 55.0) * 60
            rpm_2s = (pps_2s / 55.0) * 60
            rpm_5s = (pps_5s / 55.0) * 60
            
            print(f"   - RPM estimado (1s): {rpm_1s:.1f}")
            print(f"   - RPM estimado (2s): {rpm_2s:.1f}")
            print(f"   - RPM estimado (5s): {rpm_5s:.1f}")
        
        # Verificar si los sensores Hall están conectados correctamente
        print("\n5. TEST DE CONECTIVIDAD DE SENSORES:")
        print("   Moviendo motor muy lentamente para verificar detección...")
        
        ser.write("RESET_PULSES\n".encode())
        time.sleep(0.1)
        ser.readline()
        
        # PWM muy bajo para movimiento lento
        ser.write("PWM 30 0\n".encode())
        
        for i in range(10):
            time.sleep(0.5)
            ser.write("GET\n".encode())
            time.sleep(0.1)
            resp = ser.readline().decode().strip()
            if resp.startswith('DATA '):
                parts = resp.split()
                print(f"   t={i*0.5:.1f}s: Pulsos={parts[15]}")
        
        ser.write("PARADA\n".encode())
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    analizar_pulsos()