import serial
import time

def test_pulses():
    ser = serial.Serial('COM12', 115200, timeout=2)
    time.sleep(1)
    
    print("=== PRUEBA DE CONTADORES DE PULSOS ===")
    
    # 1. Verificar estado inicial
    print("\n1. Estado inicial:")
    ser.write(b'GET\n')
    time.sleep(0.5)
    response = ser.read(1000).decode('utf-8', errors='ignore')
    lines = [l.strip() for l in response.split('\n') if l.strip()]
    for line in lines:
        if line.startswith("DATA"):
            parts = line.split()
            if len(parts) >= 18:
                print(f"  Pulsos L: {parts[14]}, R: {parts[15]}")
                print(f"  PPR medido L: {parts[16]}, R: {parts[17]}")
    
    # 2. Activar motor izquierdo con PWM bajo
    print("\n2. Activando motor izquierdo (PWM 50):")
    ser.write(b'PWM 50 0\n')
    time.sleep(0.5)
    
    # Esperar 3 segundos para que se generen pulsos
    print("  Esperando 3 segundos...")
    time.sleep(3)
    
    # Parar motores
    ser.write(b'PARADA\n')
    time.sleep(0.5)
    
    # 3. Verificar contadores después del movimiento
    print("\n3. Contadores después del movimiento:")
    ser.write(b'GET\n')
    time.sleep(0.5)
    response = ser.read(1000).decode('utf-8', errors='ignore')
    lines = [l.strip() for l in response.split('\n') if l.strip()]
    for line in lines:
        if line.startswith("DATA"):
            parts = line.split()
            if len(parts) >= 18:
                print(f"  Pulsos L: {parts[14]}, R: {parts[15]}")
                print(f"  PPR medido L: {parts[16]}, R: {parts[17]}")
    
    # 4. Reset contadores
    print("\n4. Reseteando contadores:")
    ser.write(b'RESET_PULSES\n')
    time.sleep(0.5)
    
    # 5. Verificar reset
    print("\n5. Contadores después del reset:")
    ser.write(b'GET\n')
    time.sleep(0.5)
    response = ser.read(1000).decode('utf-8', errors='ignore')
    lines = [l.strip() for l in response.split('\n') if l.strip()]
    for line in lines:
        if line.startswith("DATA"):
            parts = line.split()
            if len(parts) >= 18:
                print(f"  Pulsos L: {parts[14]}, R: {parts[15]}")
                print(f"  PPR medido L: {parts[16]}, R: {parts[17]}")
    
    print("\n=== FIN DE PRUEBA ===")
    ser.close()

if __name__ == "__main__":
    test_pulses()