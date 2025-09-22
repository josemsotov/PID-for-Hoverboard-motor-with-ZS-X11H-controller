import serial
import time

def debug_telemetry():
    print("=== DEBUG DETALLADO DE TELEMETRÍA ===")
    
    try:
        ser = serial.Serial('COM12', 115200, timeout=2)
        time.sleep(2)  # Esperar más tiempo para inicialización
        
        # Leer mensajes de inicialización
        print("Leyendo inicialización:")
        for i in range(5):
            line = ser.readline().decode().strip()
            if line:
                print(f"  Init: {line}")
        
        print("\n1. Verificando formato de datos:")
        ser.write("GET\n".encode())
        time.sleep(0.2)
        
        response = ser.readline().decode().strip()
        print(f"Respuesta completa: '{response}'")
        
        if response.startswith("DATA "):
            parts = response.split()
            print(f"Número total de campos: {len(parts)} (incluyendo 'DATA')")
            
            for i, part in enumerate(parts):
                field_name = "?"
                if i == 0: field_name = "COMMAND"
                elif i == 1: field_name = "rpmL"
                elif i == 2: field_name = "rpmR"
                elif i == 3: field_name = "mmpsL"
                elif i == 4: field_name = "mmpsR"
                elif i == 5: field_name = "tgtRpmL"
                elif i == 6: field_name = "tgtRpmR"
                elif i == 7: field_name = "pwmL"
                elif i == 8: field_name = "pwmR"
                elif i == 9: field_name = "kp"
                elif i == 10: field_name = "ki"
                elif i == 11: field_name = "kd"
                elif i == 12: field_name = "stopped"
                elif i == 13: field_name = "mode"
                elif i == 14: field_name = "ppr"
                elif i == 15: field_name = "pulsesL"
                elif i == 16: field_name = "pulsesR"
                elif i == 17: field_name = "measPPR_L"
                elif i == 18: field_name = "measPPR_R"
                
                print(f"  Campo {i:2d}: {field_name:12s} = '{part}'")
        
        print("\n2. Activando motor izquierdo (PWM 50) y monitoreando:")
        ser.write("PWM 50 0\n".encode())
        time.sleep(2)
        
        ser.write("GET\n".encode())
        time.sleep(0.1)
        response = ser.readline().decode().strip()
        
        if response.startswith("DATA "):
            parts = response.split()
            print(f"Después de activar motor izquierdo:")
            print(f"  pulsesL (campo 15): {parts[15] if len(parts) > 15 else 'N/A'}")
            print(f"  pulsesR (campo 16): {parts[16] if len(parts) > 16 else 'N/A'}")
            print(f"  measPPR_L (campo 17): {parts[17] if len(parts) > 17 else 'N/A'}")
            print(f"  measPPR_R (campo 18): {parts[18] if len(parts) > 18 else 'N/A'}")
        
        print("\n3. Parando motores:")
        ser.write("PARADA\n".encode())
        time.sleep(0.5)
        
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    debug_telemetry()