import serial
import time

def test_pines_hall():
    print("=== TEST DE PINES HALL ===")
    
    try:
        ser = serial.Serial('COM12', 115200, timeout=2)
        time.sleep(2)
        
        # Leer inicialización
        while ser.in_waiting:
            ser.readline()
        
        print("1. Leyendo estado de pines Hall sin movimiento:")
        for i in range(5):
            # Comando especial para leer pines digitales
            ser.write("GET\n".encode())
            time.sleep(0.1)
            resp = ser.readline().decode().strip()
            print(f"   Lectura {i+1}: Datos disponibles")
            time.sleep(0.5)
        
        print("\n2. IMPORTANTE: Verificar conexiones físicas:")
        print("   - Pin 2 (HALL_L) debe estar conectado al sensor Hall del motor izquierdo")
        print("   - Pin 3 (HALL_R) debe estar conectado al sensor Hall del motor derecho")
        print("   - Los sensores Hall necesitan alimentación (5V y GND)")
        print("   - Verificar que los motores TENGAN sensores Hall incorporados")
        
        print("\n3. Test de movimiento lento para debug:")
        print("   Mover motor muy lentamente y observar...")
        
        ser.write("RESET_PULSES\n".encode())
        time.sleep(0.1)
        ser.readline()
        
        # PWM muy bajo para rotación lenta manual
        print("   Activando PWM bajo (30) por 10 segundos...")
        print("   OBSERVA si el motor gira y si hay cambios en los pulsos")
        
        ser.write("PWM 30 0\n".encode())
        
        for i in range(10):
            time.sleep(1)
            ser.write("GET\n".encode())
            time.sleep(0.1)
            resp = ser.readline().decode().strip()
            if resp.startswith('DATA '):
                parts = resp.split()
                print(f"   Segundo {i+1}: Pulsos L={parts[15]}, R={parts[16]}")
            else:
                print(f"   Segundo {i+1}: Sin respuesta válida")
        
        ser.write("PARADA\n".encode())
        print("\n4. DIAGNÓSTICO:")
        print("   Si NO se detectan pulsos durante el movimiento:")
        print("   → Los sensores Hall NO están conectados o NO funcionan")
        print("   → Los motores pueden no tener sensores Hall")
        print("   → Verificar cableado y alimentación de sensores")
        
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_pines_hall()