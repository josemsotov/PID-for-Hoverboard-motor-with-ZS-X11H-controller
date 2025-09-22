#!/usr/bin/env python3
"""
Test de Conexión Básica - Sistema de Control de Motores
Verifica la comunicación serial básica con el Arduino
"""

import serial
import time

def test_connection():
    """Prueba básica de conexión y comunicación"""
    try:
        print("=== TEST DE CONEXIÓN BÁSICA ===")
        print("Conectando a COM12...")
        
        ser = serial.Serial('COM12', 115200, timeout=2)
        time.sleep(2)  # Esperar inicialización
        
        print("✓ Conexión establecida")
        
        # Prueba básica de comunicación
        print("\nPrueba de comunicación:")
        ser.write(b'GET\n')
        response = ser.readline().decode().strip()
        
        if response.startswith('DATA'):
            print("✓ Comunicación exitosa")
            print(f"  Respuesta: {response[:50]}...")
            
            # Extraer RPM básicos
            parts = response.split()
            if len(parts) > 2:
                rpm_l, rpm_r = parts[0], parts[1]
                print(f"  RPM: L={rpm_l} R={rpm_r}")
                
        elif response:
            print(f"✓ Dispositivo responde: {response}")
        else:
            print("⚠ No hay respuesta del dispositivo")
            
        ser.close()
        print("\n✓ Test completado exitosamente")
        return True
        
    except serial.SerialException as e:
        print(f"✗ Error de conexión serial: {e}")
        print("  Verifica que:")
        print("  - El Arduino esté conectado al puerto COM12")
        print("  - No hay otras aplicaciones usando el puerto")
        return False
        
    except Exception as e:
        print(f"✗ Error inesperado: {e}")
        return False

if __name__ == "__main__":
    test_connection()