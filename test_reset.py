import serial
import time

def test_reset():
    print("=== PRUEBA DE COMANDO RESET ===")
    
    try:
        ser = serial.Serial('COM12', 115200, timeout=2)
        time.sleep(2)
        
        # Leer inicialización
        while ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                print(f"Init: {line}")
        
        print("\n1. Estado inicial:")
        ser.write("GET\n".encode())
        time.sleep(0.1)
        response = ser.readline().decode().strip()
        if response.startswith("DATA "):
            parts = response.split()
            print(f"  Pulsos L: {parts[15]}, R: {parts[16]}")
        
        print("\n2. Enviando comando RESET_PULSES:")
        ser.write("RESET_PULSES\n".encode())
        time.sleep(0.1)
        
        # Leer respuesta del reset
        response = ser.readline().decode().strip()
        print(f"  Respuesta: '{response}'")
        
        print("\n3. Estado después del reset:")
        ser.write("GET\n".encode())
        time.sleep(0.1)
        response = ser.readline().decode().strip()
        if response.startswith("DATA "):
            parts = response.split()
            print(f"  Pulsos L: {parts[15]}, R: {parts[16]}")
        
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_reset()