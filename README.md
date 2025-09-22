# Sistema de Control PID para Motores Hoverboard con ZS-X11H

Sistema completo de control PID para dos motores de hoverboard usando controladores ZS-X11H, con control de velocidad diferencial, alineación automática de velocidades y interfaz gráfica.

## 🚀 Características Principales

- **Control PID dual** con parámetros independientes para cada motor
- **Sistema de alineación automática** que compensa diferencias de velocidad entre motores
- **Control de velocidad por incrementos** con comandos direccionales
- **Interfaz gráfica completa** con monitoreo en tiempo real
- **Encoders duales** (Hall + ópticos) para medición precisa de velocidad
- **PPR individuales** configurables para cada motor
- **Control de dirección diferencial** para movimiento de robot

## 📁 Estructura del Proyecto

```
├── src/main.cpp              # Firmware principal (Arduino)
├── gui/controller.py         # Interfaz gráfica (Python/Tkinter)
├── platformio.ini           # Configuración PlatformIO
├── diagnostic_speed.py      # Diagnóstico del sistema de alineación
├── test_connection.py       # Test básico de comunicación
├── HB_Motor_PID.ino        # Backup del firmware (Arduino IDE)
└── README.md               # Este archivo
```

## ⚡ Inicio Rápido

### 1. Compilar y Subir Firmware
```bash
pio run --target upload
```

### 2. Ejecutar Interfaz Gráfica
```bash
cd gui
python controller.py
```

### 3. Test de Conexión
```bash
python test_connection.py
```

## 🔧 Configuración de Hardware

### Pines Arduino Uno (Físicamente Corregidos)
```
Motor Izquierdo (Físico):
  HALL_L  -> Pin 3 (INT1)
  PWM_L   -> Pin 9 (PWM)
  DIR_L   -> Pin 8 (HIGH=adelante, LOW=atrás)
  ENCODER_L -> Pin 4 (Encoder óptico)

Motor Derecho (Físico):  
  HALL_R  -> Pin 2 (INT0)
  PWM_R   -> Pin 6 (PWM)
  DIR_R   -> Pin 7 (LOW=adelante, HIGH=atrás)
  ENCODER_R -> Pin 5 (Encoder óptico)
```

### PPR (Pulsos por Revolución)
- Motor Izquierdo: 55 PPR (más rápido naturalmente)
- Motor Derecho: 45 PPR (más lento naturalmente)

## 🎮 Comandos del Sistema

### Control de Velocidad Robot
- `VEL_FORWARD` - Configurar dirección adelante
- `VEL_BACKWARD` - Configurar dirección atrás  
- `VEL_UP` - Incrementar velocidad
- `VEL_DOWN` - Decrementar velocidad
- `VEL_STOP` - Detener motores

### Sistema de Alineación
- `ALIGN_ENABLE` - Habilitar alineación automática
- `ALIGN_DISABLE` - Deshabilitar alineación
- `ALIGN_CALIBRATE` - Calibrar factores de corrección
- `ALIGN_STATUS` - Ver estado de alineación

### Comandos Básicos
- `GET` - Obtener telemetría completa
- `PWM pL pR` - Control PWM directo
- `STOP` - Parada de emergencia
- `KP/KI/KD valor` - Ajustar parámetros PID

## 📊 Interfaz Gráfica

La interfaz incluye:
- **Monitor de velocidad** en tiempo real (RPM y mm/s)
- **Control de velocidad robot** con botones direccionales
- **Sistema de alineación** con calibración automática
- **Encoders duales** con contadores de pulsos
- **Odometría** con posición y distancia acumulada
- **Control de teclado** (WASD/Flechas)

## 🔍 Diagnóstico

Para verificar el sistema de alineación:
```bash
python diagnostic_speed.py
```

Este script ejecuta una prueba completa que:
1. Verifica PPR individuales
2. Prueba control de velocidad robot
3. Calibra alineación automática
4. Muestra factores de corrección

## 📈 Sistema de Alineación

El sistema detecta automáticamente qué motor va más rápido y aplica factores de corrección:
- **Factor L**: Corrección para motor izquierdo
- **Factor R**: Corrección para motor derecho
- **Tolerancia**: 5.0 RPM por defecto
- **Calibración automática**: Ajusta factores en tiempo real

## 🛠️ Requisitos

### Hardware
- Arduino Uno
- 2x Controladores ZS-X11H
- 2x Motores de hoverboard
- 2x Encoders ópticos adicionales
- Conexiones según diagrama de pines

### Software
- PlatformIO (recomendado) o Arduino IDE
- Python 3.x
- Librerías: tkinter, serial, threading

## 📝 Notas Importantes

- **Asignaciones físicas corregidas**: Los nombres L/R coinciden con la realidad física
- **Motor izquierdo naturalmente más rápido**: El sistema compensa automáticamente
- **Control diferencial**: Robot se mueve correctamente adelante/atrás/giros
- **Seguridad**: Comando STOP disponible en todo momento

## 🔄 Historial de Versiones

- **v5.0**: Sistema completo con alineación automática y asignaciones físicas corregidas
- **v4.x**: Implementación de encoders duales y PPR individuales  
- **v3.x**: Sistema de control de velocidad robot con incrementos
- **v2.x**: Interfaz gráfica y control PID dual
- **v1.x**: Control PID básico para motores individuales

---

**Proyecto basado en el trabajo original de [oracid](https://github.com/oracid/PID-for-Hoverboard-motor-with-ZS-X11H-controller)**