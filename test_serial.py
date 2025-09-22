import serial
import time

ser = serial.Serial('COM12', 115200, timeout=3)
time.sleep(2)
ser.flushInput()
print("Enviando GET...")
ser.write(b'GET\n')
time.sleep(2)
response = ser.read(2000).decode('utf-8', errors='ignore')
print("Respuesta completa:")
print(response)
print("-----")
lines = [l.strip() for l in response.split('\n') if l.strip()]
print(f"Total líneas: {len(lines)}")
for i, line in enumerate(lines):
    print(f"{i}: {line}")
    if line.startswith("DATA"):
        parts = line.split()
        print(f"  Campos DATA: {len(parts)-1} (sin contar 'DATA')")
        for j, part in enumerate(parts[1:]):  # Skip 'DATA'
            print(f"    {j}: {part}")
ser.close()