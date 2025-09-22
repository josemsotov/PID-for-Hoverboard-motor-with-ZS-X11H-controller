import serial
import time

def test_nuevo_motor():
    print("=== PRUEBA ESPECÍFICA DEL NUEVO MOTOR ===")
    
    try:
        ser = serial.Serial('COM12', 115200, timeout=2)
        time.sleep(2)
        
        # Leer inicialización
        while ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                print(f"Init: {line}")
        
        print("\n1. VERIFICACIÓN DE CONEXIONES FÍSICAS:")
        print("   - ¿Sensor Hall del nuevo motor conectado al pin 2 del Arduino?")
        print("   - ¿Sensor recibe alimentación (5V y GND)?")
        print("   - ¿Cables bien soldados/conectados?\n")
        
        print("2. Estado inicial de sensores Hall:")
        for i in range(3):
            ser.write("HALL_DEBUG\n".encode())
            time.sleep(0.1)
            resp = ser.readline().decode().strip()
            print(f"   Lectura {i+1}: {resp}")
            time.sleep(0.5)
        
        print("\n3. RESET y verificación:")
        ser.write("RESET_PULSES\n".encode())
        time.sleep(0.1)
        ack = ser.readline().decode().strip()
        print(f"   Reset: {ack}")
        
        ser.write("HALL_DEBUG\n".encode())
        time.sleep(0.1)
        resp = ser.readline().decode().strip()
        print(f"   Después reset: {resp}")
        
        print("\n4. PRUEBA DE MOVIMIENTO - Motor Izquierdo (PWM 60):")
        print("   Observa el motor físicamente durante esta prueba...")
        
        ser.write("PWM 60 0\n".encode())
        
        for i in range(8):
            time.sleep(1)
            ser.write("HALL_DEBUG\n".encode())
            time.sleep(0.1)
            resp = ser.readline().decode().strip()
            print(f"   t={i+1}s: {resp}")
            
            # Analizar cambios
            if "PulsesL=" in resp:
                pulsos = resp.split("PulsesL=")[1].split()[0]
                if int(pulsos) > 0:
                    print(f"   ✅ ¡DETECTANDO PULSOS! Contador: {pulsos}")
                    break
        
        print("\n5. Parando motor:")
        ser.write("PARADA\n".encode())
        time.sleep(0.5)
        
        # Estado final
        ser.write("HALL_DEBUG\n".encode())
        time.sleep(0.1)
        resp = ser.readline().decode().strip()
        print(f"   Estado final: {resp}")
        
        # Análisis
        if "PulsesL=" in resp:
            pulsos_final = resp.split("PulsesL=")[1].split()[0]
            if int(pulsos_final) > 0:
                print(f"\n✅ ÉXITO: Se detectaron {pulsos_final} pulsos")
                print("   → Los sensores Hall del NUEVO motor SÍ funcionan")
                print("   → El problema era del motor ANTERIOR")
            else:
                print("\n❌ PROBLEMA PERSISTE: No se detectaron pulsos")
                print("   → Revisar conexiones físicas")
                print("   → Verificar alimentación del sensor")
        
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_nuevo_motor()