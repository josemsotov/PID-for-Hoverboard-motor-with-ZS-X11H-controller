import serial
import time

def analizar_pulsos_simple():
    print("=== ANÁLISIS SIMPLE DE PULSOS ===")
    
    try:
        ser = serial.Serial('COM12', 115200, timeout=2)
        time.sleep(2)
        
        # Leer inicialización
        while ser.in_waiting:
            ser.readline()
        
        print("1. Reset y estado inicial:")
        ser.write("RESET_PULSES\n".encode())
        time.sleep(0.1)
        ser.readline()  # leer ACK
        
        ser.write("GET\n".encode())
        time.sleep(0.1)
        resp = ser.readline().decode().strip()
        print(f"   Respuesta inicial: {resp}")
        
        if resp.startswith('DATA '):
            parts = resp.split()
            print(f"   Pulsos L inicial: {parts[15]}")
            print(f"   PPR configurado: {parts[14]}")
        
        print("\n2. Activando motor por 3 segundos...")
        ser.write("PWM 80 0\n".encode())
        
        # Monitorear pulsos en tiempo real
        for i in range(6):  # 6 lecturas en 3 segundos
            time.sleep(0.5)
            ser.write("GET\n".encode())
            time.sleep(0.1)
            resp = ser.readline().decode().strip()
            if resp.startswith('DATA '):
                parts = resp.split()
                print(f"   t={i*0.5:.1f}s: Pulsos L={parts[15]}, R={parts[16]}")
        
        print("\n3. Parando motor...")
        ser.write("PARADA\n".encode())
        time.sleep(0.5)
        
        ser.write("GET\n".encode())
        time.sleep(0.1)
        resp = ser.readline().decode().strip()
        if resp.startswith('DATA '):
            parts = resp.split()
            pulsos_final = int(parts[15])
            print(f"   Pulsos finales: {pulsos_final}")
            
            if pulsos_final > 0:
                pps = pulsos_final / 3.0  # pulsos por segundo
                rpm = (pps / 55.0) * 60   # asumiendo 55 PPR
                print(f"   Pulsos por segundo: {pps:.1f}")
                print(f"   RPM estimado: {rpm:.1f}")
            else:
                print("   ⚠️  NO SE DETECTARON PULSOS - Posible problema de conexión")
        
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    analizar_pulsos_simple()