import serial
import time

def test_hall_debug():
    print("=== TEST DEBUG DE SENSORES HALL ===")
    
    try:
        ser = serial.Serial('COM12', 115200, timeout=2)
        time.sleep(2)
        
        # Leer inicialización
        while ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                print(f"Init: {line}")
        
        print("\n1. Leyendo estado de sensores Hall SIN movimiento:")
        for i in range(5):
            ser.write("HALL_DEBUG\n".encode())
            time.sleep(0.1)
            resp = ser.readline().decode().strip()
            print(f"   {resp}")
            time.sleep(0.5)
        
        print("\n2. Activando motor lentamente y monitoreando sensores:")
        ser.write("PWM 40 0\n".encode())
        
        for i in range(15):  # 15 segundos de monitoreo
            time.sleep(1)
            ser.write("HALL_DEBUG\n".encode())
            time.sleep(0.1)
            resp = ser.readline().decode().strip()
            print(f"   t={i+1}s: {resp}")
        
        print("\n3. Parando motor:")
        ser.write("PARADA\n".encode())
        time.sleep(0.5)
        
        ser.write("HALL_DEBUG\n".encode())
        time.sleep(0.1)
        resp = ser.readline().decode().strip()
        print(f"   Final: {resp}")
        
        print("\n=== INTERPRETACIÓN DE RESULTADOS ===")
        print("Si L y R siempre muestran el mismo valor (0 o 1):")
        print("  → Los sensores NO están funcionando o NO están conectados")
        print("\nSi L y R cambian durante el movimiento:")
        print("  → Los sensores SÍ funcionan y detectan el movimiento")
        print("\nSi PulsesL/PulsesR incrementan:")
        print("  → Las interrupciones SÍ están funcionando")
        
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_hall_debug()