import serial
import time

def test_with_gui():
    print("=== PRUEBA CON GUI EJECUTÁNDOSE ===")
    print("La GUI debería estar mostrando la interfaz ahora.")
    print("\nPara probar los contadores de pulsos:")
    print("1. Abre la GUI si no está visible")
    print("2. Observa la sección 'Contadores de Pulsos'")
    print("3. Usa los botones de control de PWM para mover el motor")
    print("4. Verifica si los contadores incrementan")
    print("5. Prueba el botón 'Reset Contadores'")
    
    print("\nSi la GUI no responde, podemos hacer una prueba directa...")
    
    # Prueba rápida para verificar conexión
    try:
        print("\nPROBANDO CONEXIÓN SERIAL...")
        ser = serial.Serial('COM12', 115200, timeout=1)
        time.sleep(1)
        
        # Leer cualquier mensaje de inicialización
        while ser.in_waiting:
            ser.readline()
        
        print("Enviando comando HALL_DEBUG...")
        ser.write("HALL_DEBUG\n".encode())
        time.sleep(0.1)
        resp = ser.readline().decode().strip()
        print(f"Respuesta: {resp}")
        
        ser.close()
        print("\n✅ Conexión serial OK - La GUI puede usar el puerto")
        
    except serial.SerialException:
        print("\n⚠️  Puerto COM12 ocupado - GUI probablemente conectada")
        print("Esto es normal si la GUI está ejecutándose")
    except Exception as e:
        print(f"\n❌ Error: {e}")

if __name__ == "__main__":
    test_with_gui()