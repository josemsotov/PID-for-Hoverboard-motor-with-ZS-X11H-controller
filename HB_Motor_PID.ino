// DualMotor_PID - Control PID para 2 motores con ZS-X11H - Arduino Uno
// Serie a 115200 baudios, control por comandos seriales y GUI Python
// Fecha: 2025-09-19

/*
Resumen de pines (Arduino Uno):
  Motor Izquierdo (FÍSICO):
    HALL_L  -> 3  (INT1) 
    PWM_L   -> 9  (PWM)
    DIR_L   -> 8  (HIGH=adelante, LOW=atrás) *** INVERSO ***
    BRAKE_L -> 11 (Digital PWM-capable)
    STOP_L  -> 13 (Digital)
  Motor Derecho (FÍSICO):
    HALL_R  -> 2  (INT0)
    PWM_R   -> 6  (PWM)
    DIR_R   -> 7  (LOW=adelante, HIGH=atrás)
    BRAKE_R -> 10 (Digital)
    STOP_R  -> 12 (Digital)
  Encoders:
    ENCODER_L -> 5 (Motor lógicamente izquierdo, físicamente derecho)
    ENCODER_R -> 4 (Motor lógicamente derecho, físicamente izquierdo)
  Reservados: UART USB: 0,1 (no se usan para motores)

LÓGICA DIRECCIONAL ROBOT DIFERENCIAL (INTERCAMBIADO L ↔ R):
  ADELANTE: DIR_L=LOW, DIR_R=HIGH  (motores en sentidos opuestos)
  ATRÁS:    DIR_L=HIGH,  DIR_R=LOW (motores en sentidos opuestos)
  GIRO_IZQ: DIR_L=HIGH,  DIR_R=HIGH  (izq atrás, der adelante)
  GIRO_DER: DIR_L=LOW, DIR_R=LOW (izq adelante, der atrás)

Características físicas:
  - Diámetro rueda: 22 cm (0.22 m)
  - Pulsos por revolución: 55 (3 sensores Hall con resolución medida)

Protocolo serial (líneas ASCII terminadas en \n):
  - ADELANTE v_mmps       -> ambas ruedas a +v
  - ATRAS v_mmps          -> ambas ruedas a -v
  - GIRAR_IZQ v_mmps      -> L = -v, R = +v
  - GIRAR_DER v_mmps      -> L = +v, R = -v
  - PARADA                -> STOP activo, PWMs a 0
  - BRAKE                 -> BRAKE activo (freno electromagnético)
  - VEL vL_mmps vR_mmps   -> fija velocidades objetivo por rueda
  - PWM pL pR             -> modo abierto: fija PWM directo [0..255] por rueda
  - KP x | KI y | KD z    -> tuning PID (ambas ruedas)
  - GET                   -> devuelve telemetría
Respuestas: "ACK" o "ERR <code>" + (en GET, datos)
*/

#include <Arduino.h>

// ------------------------ Configuración de pines ------------------------
// INTERCAMBIADAS L ↔ R - ENCODER IZQUIERDO FÍSICO TENÍA PROBLEMA
const uint8_t HALL_L = 2;   // INT0 - Motor lógicamente izquierdo (físicamente derecho)
const uint8_t HALL_R = 3;   // INT1 - Motor lógicamente derecho (físicamente izquierdo)
const uint8_t ENCODER_L = 5; // Encoder óptico rueda lógicamente izquierda (físicamente derecha)
const uint8_t ENCODER_R = 4; // Encoder óptico rueda lógicamente derecha (físicamente izquierda)
const uint8_t PWM_L  = 6;   // PWM (Timer0) - Motor lógicamente izquierdo (físicamente derecho)
const uint8_t DIR_L  = 7;   // Motor lógicamente izquierdo (físicamente derecho)
const uint8_t PWM_R  = 9;   // PWM (Timer1) - Motor lógicamente derecho (físicamente izquierdo)
const uint8_t DIR_R  = 8;   // Motor lógicamente derecho (físicamente izquierdo)
const uint8_t BRAKE_L = 10; // Digital - BRAKE motor lógicamente izquierdo (físicamente derecho)
const uint8_t BRAKE_R = 11; // Digital PWM-capable - BRAKE motor lógicamente derecho (físicamente izquierdo)
const uint8_t STOP_L = 12;  // Digital - STOP motor lógicamente izquierdo (físicamente derecho)
const uint8_t STOP_R = 13;  // Digital - STOP motor lógicamente derecho (físicamente izquierdo)

// Lógica del driver:
// BRAKE activo: HIGH = freno electromagnético activo
// STOP activo: LOW = motor detenido/deshabilitado  
// RUN normal: STOP=HIGH + BRAKE=LOW = motor habilitado para funcionar
const uint8_t RUN_LEVEL   = HIGH;   // Para STOP en funcionamiento normal
const uint8_t STOP_LEVEL  = LOW;    // Para detener motores (STOP activo bajo)
const uint8_t BRAKE_LEVEL = HIGH;   // Para freno electromagnético (BRAKE activo alto)
const uint8_t BRAKE_OFF   = LOW;    // Para liberar freno

// ------------------------ Parámetros físicos/kinemáticos ----------------
const float WHEEL_DIAMETER_M = 0.22f;     // 22 cm
const float WHEEL_CIRCUMF_M  = WHEEL_DIAMETER_M * PI; // ~0.690 m
const float PULSES_PER_REV   = 55.0f;

// PPR individuales por motor (pueden ser diferentes)
// INTERCAMBIADOS L ↔ R - ENCODER IZQUIERDO FÍSICO TENÍA PROBLEMA
float PULSES_PER_REV_L = 45.0f;  // Motor lógicamente izquierdo (físicamente derecho): 45 pulsos/rev
float PULSES_PER_REV_R = 55.0f;  // Motor lógicamente derecho (físicamente izquierdo): 55 pulsos/rev

// Conversión: mm/s <-> RPM
inline float rpm_from_mmps(float mmps) { return (mmps / 1000.0f) * 60.0f / WHEEL_CIRCUMF_M; }
inline float mmps_from_rpm(float rpm) { return (rpm * WHEEL_CIRCUMF_M / 60.0f) * 1000.0f; }

// Conversión: pulsos <-> posición (usando PPR específicos)
inline float distance_mm_from_pulses_L(uint32_t pulses) { 
  return (float(pulses) / PULSES_PER_REV_L) * (WHEEL_CIRCUMF_M * 1000.0f); 
}
inline float distance_mm_from_pulses_R(uint32_t pulses) { 
  return (float(pulses) / PULSES_PER_REV_R) * (WHEEL_CIRCUMF_M * 1000.0f); 
}

// Conversión: revoluciones <-> pulsos (usando PPR específicos)
inline float revolutions_from_pulses_L(uint32_t pulses) { 
  return float(pulses) / PULSES_PER_REV_L; 
}
inline float revolutions_from_pulses_R(uint32_t pulses) { 
  return float(pulses) / PULSES_PER_REV_R; 
}

// ------------------------ Estado de control -----------------------------
volatile uint32_t lastTickL = 0, lastTickR = 0; // ms de última detección
volatile uint32_t intervalL = 0, intervalR = 0; // ms entre tics

// Contadores de pulsos acumulados (para odometría)
volatile uint32_t pulsesCountL = 0, pulsesCountR = 0;
volatile uint32_t lastPulsesL = 0, lastPulsesR = 0;   // Para medición de PPR
volatile uint32_t lastRevTimeL = 0, lastRevTimeR = 0; // Para medición de PPR
float measuredPPR_L = 55.0f, measuredPPR_R = 45.0f; // PPR medido en tiempo real - INTERCAMBIADO

// Contadores de activaciones ISR para debug
volatile uint32_t isrCountL = 0, isrCountR = 0;

// ------------------------ Control de Velocidad Global del Robot ------------------------
float robot_velocity = 0.0f;        // Velocidad actual del robot (-100 a 100%)
float velocity_increment = 10.0f;    // Incremento de velocidad por comando (%)
int robot_direction = 1;             // 1 = adelante, -1 = atrás
bool robot_moving = false;           // Estado de movimiento del robot

// ------------------------ Sistema de Alineación de Velocidad ------------------------
float speed_correction_L = 1.0f;    // Factor de corrección para motor lógicamente izquierdo (físicamente derecho)
float speed_correction_R = 1.0f;    // Factor de corrección para motor lógicamente derecho (físicamente izquierdo)  
float target_speed_rpm = 0.0f;      // Velocidad objetivo común en RPM
bool speed_alignment_enabled = true; // Habilitar alineación automática
float alignment_tolerance = 5.0f;   // Tolerancia de alineación en RPM

// Encoder óptico (1 pulso por revolución)
volatile uint32_t encoderRevolutions = 0; // Contador de revoluciones
volatile uint32_t encoderPulses = 0;      // Contador total de pulsos del encoder
volatile uint32_t lastEncoderTime = 0;    // Tiempo del último pulso
bool lastEncoderState = HIGH;             // Estado anterior del pin (para polling)
uint32_t lastEncoderCheck = 0;           // Último tiempo de verificación

// Encoder óptico derecho (1 pulso por revolución)
volatile uint32_t encoderRevolutionsR = 0; // Contador de revoluciones derecho
volatile uint32_t encoderPulsesR = 0;      // Contador total de pulsos del encoder derecho
volatile uint32_t lastEncoderTimeR = 0;    // Tiempo del último pulso derecho
bool lastEncoderStateR = HIGH;             // Estado anterior del pin derecho (para polling)
uint32_t lastEncoderCheckR = 0;           // Último tiempo de verificación derecho

// Medidas filtradas/calculadas (actualizadas en loop)
float rpmL = 0, rpmR = 0;    // medido
float tgtRpmL = 0, tgtRpmR = 0; // objetivo

// PID por rueda
float kp = 0.15f, ki = 0.7f, kd = 0.001f;   // base del repo, ajustar en campo
float errSumL = 0, prevErrL = 0, errSumR = 0, prevErrR = 0;
float maxSum = 50.0f;                        // anti-windup

// Buffer para comandos seriales
String inbuf = "";

// Límites PWM
const int PWM_MIN = 0;       // puede colocarse >0 para superar fricción estática
const int PWM_MAX = 255;

// Límites de RPM objetivo por seguridad (ajustables)
const float RPM_ABS_MAX = 400.0f; // depende del motor/controlador

// Modo de control
enum Mode { MODE_PID = 0, MODE_PWM_DIRECT = 1 };
Mode mode = MODE_PID;

// Estado de ejecución
bool stopped = false;  // STOP activo
bool braked = false;   // BRAKE activo
int current_pwm_L = 0, current_pwm_R = 0;  // PWM actual con signo

// Tiempo
uint32_t prevMillis = 0;

// ------------------------ Utilidades ------------------------
inline void setStop(bool active) {
  stopped = active;
  digitalWrite(STOP_L, active ? STOP_LEVEL : RUN_LEVEL);
  digitalWrite(STOP_R, active ? STOP_LEVEL : RUN_LEVEL);
}

inline void setBrake(bool active) {
  braked = active;
  digitalWrite(BRAKE_L, active ? BRAKE_LEVEL : BRAKE_OFF);
  digitalWrite(BRAKE_R, active ? BRAKE_LEVEL : BRAKE_OFF);
  if (active) {
    // BRAKE activo: también poner PWM a 0 para freno electromagnético completo
    analogWrite(PWM_L, 0);
    analogWrite(PWM_R, 0);
  }
}

inline void setDirAndPwmLeft(int dir, int pwm) {
  digitalWrite(DIR_L, dir);
  analogWrite(PWM_L, constrain(pwm, PWM_MIN, PWM_MAX));
}

inline void setDirAndPwmRight(int dir, int pwm) {
  digitalWrite(DIR_R, dir);
  analogWrite(PWM_R, constrain(pwm, PWM_MIN, PWM_MAX));
}

// Seguridad: si no hay pulsos por cierto tiempo, considerar velocidad 0
const uint32_t HALL_TIMEOUT_MS = 200; // ajustable

// ------------------------ ISR Hall --------------------------
void isrLeft() {
  isrCountL++;  // Contador de activaciones para debug
  
  uint32_t now = millis();
  intervalL = now - lastTickL;
  lastTickL = now;
  
  // Contar pulsos para odometría
  pulsesCountL++;
  
  // Medición de PPR en tiempo real (cada revolución aproximada)
  if (pulsesCountL > 0 && (pulsesCountL % 40) == 0) { // cada ~40 pulsos (cerca del PPR del motor L)
    uint32_t timeDiff = now - lastRevTimeL;
    if (timeDiff > 1000) { // mínimo 1 segundo entre mediciones
      uint32_t actualPulses = pulsesCountL - lastPulsesL;
      if (actualPulses > 10) { // filtro de ruido
        float revs = (float)actualPulses / PULSES_PER_REV_L;  // Usar PPR específico del motor L
        if (revs >= 0.8f && revs <= 1.2f) { // Solo aceptar si es ~1 revolución
          measuredPPR_L = 0.8f * measuredPPR_L + 0.2f * (float)actualPulses; // Filtro
        }
        lastPulsesL = pulsesCountL;
        lastRevTimeL = now;
      }
    }
  }
}

void isrRight() {
  isrCountR++;  // Contador de activaciones para debug
  
  uint32_t now = millis();
  intervalR = now - lastTickR;
  lastTickR = now;
  
  // Contar pulsos para odometría
  pulsesCountR++;
  
  // Medición de PPR en tiempo real (cada revolución aproximada)
  if (pulsesCountR > 0 && (pulsesCountR % 50) == 0) { // cada ~50 pulsos (cerca del PPR del motor R)
    uint32_t timeDiff = now - lastRevTimeR;
    if (timeDiff > 1000) { // mínimo 1 segundo entre mediciones
      uint32_t actualPulses = pulsesCountR - lastPulsesR;
      if (actualPulses > 10) { // filtro de ruido
        float revs = (float)actualPulses / PULSES_PER_REV_R;  // Usar PPR específico del motor R
        if (revs >= 0.8f && revs <= 1.2f) { // Solo aceptar si es ~1 revolución
          measuredPPR_R = 0.8f * measuredPPR_R + 0.2f * (float)actualPulses; // Filtro
        }
        lastPulsesR = pulsesCountR;
        lastRevTimeR = now;
      }
    }
  }
}

// ------------------------ ISR Encoder Óptico ---------------
void checkEncoder() {
  // Función de polling para el encoder izquierdo en pin 5
  // Se llama frecuentemente desde el loop principal
  bool currentState = digitalRead(ENCODER_L);
  
  // Detectar flanco descendente (HIGH → LOW)
  if (lastEncoderState == HIGH && currentState == LOW) {
    uint32_t now = millis();
    lastEncoderTime = now;
    
    // Incrementar contadores
    encoderPulses++;
    encoderRevolutions++;  // 1 pulso = 1 revolución
  }
  
  lastEncoderState = currentState;
}

void checkEncoderR() {
  // Función de polling para el encoder derecho en pin 4
  // Se llama frecuentemente desde el loop principal
  bool currentState = digitalRead(ENCODER_R);
  
  // Detectar flanco descendente (HIGH → LOW)
  if (lastEncoderStateR == HIGH && currentState == LOW) {
    uint32_t now = millis();
    lastEncoderTimeR = now;
    
    // Incrementar contadores
    encoderPulsesR++;
    encoderRevolutionsR++;  // 1 pulso = 1 revolución
  }
  
  lastEncoderStateR = currentState;
}

// ------------------------ Cálculo de RPM --------------------
inline float rpm_from_interval(uint32_t interval_ms) {
  if (interval_ms == 0) return 0.0f;
  // rpm = 60000 / (interval_ms * pulsesPerRev)
  return 60000.0f / (float(interval_ms) * PULSES_PER_REV);
}

inline float rpm_from_interval_ppr(uint32_t interval_ms, float ppr) {
  if (interval_ms == 0) return 0.0f;
  // rpm = 60000 / (interval_ms * pulsesPerRev_specific)
  return 60000.0f / (float(interval_ms) * ppr);
}

// ------------------------ PID -------------------------------
float pidStep(float tgtRpm, float measRpm, float &errSum, float &prevErr, float dt) {
  float error = tgtRpm - measRpm;
  float P = kp * error;
  errSum += error * dt;
  errSum = constrain(errSum, -maxSum, maxSum);
  float I = ki * errSum;
  float D = kd * (error - prevErr) / (dt > 0 ? dt : 1e-3f);
  prevErr = error;
  return P + I + D; // salida en "unidades pwm" aproximadas
}

// ------------------------ Serial parsing --------------------

void sendACK() { Serial.println(F("ACK")); }
void sendERR(const __FlashStringHelper* msg) { Serial.print(F("ERR ")); Serial.println(msg); }

void cmd_parada() {
  setBrake(false);  // Liberar BRAKE primero
  setStop(true);    // Activar STOP
  setDirAndPwmLeft(0, 0);
  setDirAndPwmRight(0, 0);
  current_pwm_L = current_pwm_R = 0;  // Reset PWM tracking
  tgtRpmL = tgtRpmR = 0;
  sendACK();
}

void cmd_brake() {
  setStop(false);   // Liberar STOP primero  
  setBrake(true);   // Activar BRAKE
  tgtRpmL = tgtRpmR = 0;
  sendACK();
}

void cmd_pwm(int pL, int pR) {
  mode = MODE_PWM_DIRECT;
  setStop(false);   // Deshabilitar STOP
  setBrake(false);  // Deshabilitar BRAKE
  
  // Manejar PWM con signo: valores negativos invierten dirección
  int pwmL_abs = abs(pL);
  int pwmR_abs = abs(pR);
  
  // Direcciones CORREGIDAS para movimiento correcto del robot diferencial
  // Motor izquierdo físico: HIGH=adelante, LOW=atrás
  // Motor derecho físico: LOW=adelante, HIGH=atrás  
  bool dirL = (pL >= 0) ? HIGH : LOW;   // Izquierdo físico: HIGH=adelante
  bool dirR = (pR >= 0) ? LOW : HIGH;   // Derecho físico: LOW=adelante
  
  // Guardar PWM actual con signo para telemetría
  current_pwm_L = pL;
  current_pwm_R = pR;
  
  // Aplicar dirección y PWM
  digitalWrite(DIR_L, dirL);
  digitalWrite(DIR_R, dirR);
  analogWrite(PWM_L, constrain(pwmL_abs, PWM_MIN, PWM_MAX));
  analogWrite(PWM_R, constrain(pwmR_abs, PWM_MIN, PWM_MAX));
  
  sendACK();
}

void set_vel_mmps(float vL_mmps, float vR_mmps) {
  mode = MODE_PID;
  setStop(false);   // Deshabilitar STOP
  setBrake(false);  // Deshabilitar BRAKE
  // Convertir a RPM y fijar dirección
  float rL = rpm_from_mmps(fabs(vL_mmps));
  float rR = rpm_from_mmps(fabs(vR_mmps));
  rL = constrain(rL, 0, RPM_ABS_MAX);
  rR = constrain(rR, 0, RPM_ABS_MAX);
  tgtRpmL = rL;
  tgtRpmR = rR;
  
  // Direcciones INTERCAMBIADAS L ↔ R - ENCODER IZQUIERDO FÍSICO TENÍA PROBLEMA
  // Motor lógicamente izquierdo (físicamente derecho): LOW=adelante, HIGH=atrás  
  // Motor lógicamente derecho (físicamente izquierdo): HIGH=adelante, LOW=atrás
  digitalWrite(DIR_L, (vL_mmps >= 0) ? LOW : HIGH);    // Lógicamente izquierdo (físicamente derecho)
  digitalWrite(DIR_R, (vR_mmps >= 0) ? HIGH : LOW);    // Lógicamente derecho (físicamente izquierdo)
}

void cmd_vel(float vL_mmps, float vR_mmps) { set_vel_mmps(vL_mmps, vR_mmps); sendACK(); }

void cmd_adelante(float v_mmps) { set_vel_mmps(v_mmps, v_mmps); sendACK(); }
void cmd_atras(float v_mmps)    { set_vel_mmps(-v_mmps, -v_mmps); sendACK(); }
void cmd_girar_izq(float v_mmps){ set_vel_mmps(-v_mmps,  v_mmps); sendACK(); }
void cmd_girar_der(float v_mmps){ set_vel_mmps( v_mmps, -v_mmps); sendACK(); }

void cmd_kp(float v){ kp = v; sendACK(); }
void cmd_ki(float v){ ki = v; sendACK(); }
void cmd_kd(float v){ kd = v; sendACK(); }

void cmd_pulses(float v) { 
  // NOTA: Esta función cambiaría PULSES_PER_REV pero es const en este código
  // Para hacerlo variable necesitarías cambiar const float a float
  sendACK(); 
}

void cmd_ppr_l(float v) {
  // Actualizar PPR del motor izquierdo
  if (v > 10 && v < 200) {  // Rango válido
    PULSES_PER_REV_L = v;
    measuredPPR_L = v;  // También actualizar el medido
    sendACK();
  } else {
    Serial.println(F("ERR: PPR_L fuera de rango (10-200)"));
  }
}

void cmd_ppr_r(float v) {
  // Actualizar PPR del motor derecho
  if (v > 10 && v < 200) {  // Rango válido
    PULSES_PER_REV_R = v;
    measuredPPR_R = v;  // También actualizar el medido
    sendACK();
  } else {
    Serial.println(F("ERR: PPR_R fuera de rango (10-200)"));
  }
}

void cmd_reset_pulses() {
  pulsesCountL = pulsesCountR = 0;
  lastPulsesL = lastPulsesR = 0;
  lastRevTimeL = lastRevTimeR = millis();
  measuredPPR_L = PULSES_PER_REV_L;  // Reset PPR medido L al valor específico
  measuredPPR_R = PULSES_PER_REV_R;  // Reset PPR medido R al valor específico
  isrCountL = isrCountR = 0;  // Reset contadores ISR
  
  // Reset encoders
  encoderRevolutions = encoderPulses = 0;
  encoderRevolutionsR = encoderPulsesR = 0;
  lastEncoderTime = lastEncoderTimeR = millis();
  
  sendACK();
}

void cmd_hall_debug() {
  // Comando para debug de sensores Hall
  int hallL_state = digitalRead(HALL_L);
  int hallR_state = digitalRead(HALL_R);
  Serial.print(F("HALL_DEBUG L="));
  Serial.print(hallL_state);
  Serial.print(F(" R="));
  Serial.print(hallR_state);
  Serial.print(F(" PulsesL="));
  Serial.print(pulsesCountL);
  Serial.print(F(" PulsesR="));
  Serial.print(pulsesCountR);
  Serial.print(F(" ISR_L="));
  Serial.print(isrCountL);
  Serial.print(F(" ISR_R="));
  Serial.println(isrCountR);
}

void cmd_odometry() {
  // Comando para información de odometría usando PPR específicos
  float revs_L = revolutions_from_pulses_L(pulsesCountL);
  float revs_R = revolutions_from_pulses_R(pulsesCountR);
  float dist_L_mm = distance_mm_from_pulses_L(pulsesCountL);
  float dist_R_mm = distance_mm_from_pulses_R(pulsesCountR);
  
  Serial.print(F("ODOMETRY PulsesL="));
  Serial.print(pulsesCountL);
  Serial.print(F(" RevsL="));
  Serial.print(revs_L, 2);
  Serial.print(F(" DistL="));
  Serial.print(dist_L_mm, 1);
  Serial.print(F("mm PulsesR="));
  Serial.print(pulsesCountR);
  Serial.print(F(" RevsR="));
  Serial.print(revs_R, 2);
  Serial.print(F(" DistR="));
  Serial.print(dist_R_mm, 1);
  Serial.print(F("mm PPR_L="));
  Serial.print(PULSES_PER_REV_L, 1);
  Serial.print(F(" PPR_R="));
  Serial.println(PULSES_PER_REV_R, 1);
}

// ------------------------ Funciones de Control de Velocidad Global ------------------------
void apply_robot_velocity() {
  // Aplica la velocidad actual del robot considerando dirección y alineación
  float base_vel_mmps = (robot_velocity / 100.0f) * robot_direction * 200.0f; // Velocidad máxima 200 mm/s
  
  if (robot_moving && abs(robot_velocity) > 0.1f) {
    if (speed_alignment_enabled) {
      // Aplicar factores de corrección para alineación
      float vel_L = base_vel_mmps * speed_correction_L;
      float vel_R = base_vel_mmps * speed_correction_R;
      
      // Configurar modo PID y direcciones
      mode = MODE_PID;
      setStop(false);   // Deshabilitar STOP
      setBrake(false);  // Deshabilitar BRAKE
      
      // Convertir a RPM y aplicar directamente a targets
      float target_rpm_L = rpm_from_mmps(fabs(vel_L));
      float target_rpm_R = rpm_from_mmps(fabs(vel_R));
      
      // Aplicar límites
      tgtRpmL = constrain(target_rpm_L, 0, RPM_ABS_MAX);
      tgtRpmR = constrain(target_rpm_R, 0, RPM_ABS_MAX);
      
      // Configurar direcciones (INTERCAMBIADAS L ↔ R)
      digitalWrite(DIR_L, (vel_L >= 0) ? LOW : HIGH);    // Lógicamente izquierdo (físicamente derecho): LOW=adelante
      digitalWrite(DIR_R, (vel_R >= 0) ? HIGH : LOW);    // Lógicamente derecho (físicamente izquierdo): HIGH=adelante
      
      // Actualizar velocidad objetivo para referencia
      target_speed_rpm = rpm_from_mmps(abs(base_vel_mmps));
    } else {
      // Sin alineación - velocidades iguales usando función original
      set_vel_mmps(base_vel_mmps, base_vel_mmps);
    }
  } else {
    cmd_parada();
    robot_moving = false;
    target_speed_rpm = 0.0f;
  }
}

void cmd_vel_up() {
  robot_velocity += velocity_increment;
  if (robot_velocity > 100.0f) robot_velocity = 100.0f;
  robot_moving = true;
  apply_robot_velocity();
  sendACK();
}

void cmd_vel_down() {
  robot_velocity -= velocity_increment;
  if (robot_velocity < -100.0f) robot_velocity = -100.0f;
  robot_moving = true;
  apply_robot_velocity();
  sendACK();
}

void cmd_vel_forward() {
  robot_direction = 1;
  if (robot_velocity == 0.0f) robot_velocity = velocity_increment;
  robot_moving = true;
  apply_robot_velocity();
  sendACK();
}

void cmd_vel_backward() {
  robot_direction = -1;
  if (robot_velocity == 0.0f) robot_velocity = velocity_increment;
  robot_moving = true;
  apply_robot_velocity();
  sendACK();
}

void cmd_vel_stop() {
  robot_velocity = 0.0f;
  robot_moving = false;
  cmd_parada();
  sendACK();
}

void cmd_vel_increment(float inc) {
  velocity_increment = constrain(inc, 1.0f, 50.0f);
  sendACK();
}

// ------------------------ Funciones de Alineación de Velocidad ------------------------
void calibrate_speed_alignment() {
  // Calibra los factores de corrección basándose en las velocidades actuales
  if (abs(rpmL) > 5.0f && abs(rpmR) > 5.0f) {
    // Usar el motor más lento como referencia (factor 1.0)
    float min_rpm = min(abs(rpmL), abs(rpmR));
    speed_correction_L = min_rpm / abs(rpmL);
    speed_correction_R = min_rpm / abs(rpmR);
    
    // Limitar factores de corrección a rangos razonables
    speed_correction_L = constrain(speed_correction_L, 0.5f, 1.5f);
    speed_correction_R = constrain(speed_correction_R, 0.5f, 1.5f);
  }
}

void cmd_align_calibrate() {
  calibrate_speed_alignment();
  // Aplicar inmediatamente la corrección si el robot está en movimiento
  if (robot_moving) {
    apply_robot_velocity();
  }
  sendACK();
}

void cmd_align_enable(bool enable) {
  speed_alignment_enabled = enable;
  sendACK();
}

void cmd_align_factors(float factorL, float factorR) {
  speed_correction_L = constrain(factorL, 0.5f, 1.5f);
  speed_correction_R = constrain(factorR, 0.5f, 1.5f);
  sendACK();
}

void cmd_align_tolerance(float tolerance) {
  alignment_tolerance = constrain(tolerance, 1.0f, 20.0f);
  sendACK();
}

void cmd_align_status() {
  Serial.print(F("ALIGN_STATUS "));
  Serial.print(speed_alignment_enabled ? 1 : 0); Serial.print(' ');
  Serial.print(speed_correction_L, 3); Serial.print(' ');
  Serial.print(speed_correction_R, 3); Serial.print(' ');
  Serial.print(alignment_tolerance, 1); Serial.print(' ');
  Serial.print(abs(rpmL - rpmR), 2); Serial.print(' '); // Diferencia actual
  Serial.print(target_speed_rpm, 2);
  Serial.println();
}

void cmd_get() {
  // Telemetría extendida: rpmL rpmR mmpsL mmpsR tgtRpmL tgtRpmR pwmL pwmR kp ki kd stopped mode ppr pulsesL pulsesR measPPR_L measPPR_R encoderRevs encoderPulses encoderRevsR encoderPulsesR distL_mm distR_mm revs_L revs_R
  float mmpsL = mmps_from_rpm(rpmL);
  float mmpsR = mmps_from_rpm(rpmR);
  Serial.print(F("DATA "));
  Serial.print(rpmL, 3); Serial.print(' ');            // 0: rpmL
  Serial.print(rpmR, 3); Serial.print(' ');            // 1: rpmR
  Serial.print(mmpsL, 3); Serial.print(' ');           // 2: mmpsL
  Serial.print(mmpsR, 3); Serial.print(' ');           // 3: mmpsR
  Serial.print(tgtRpmL, 3); Serial.print(' ');         // 4: tgtRpmL
  Serial.print(tgtRpmR, 3); Serial.print(' ');         // 5: tgtRpmR
  Serial.print(current_pwm_L); Serial.print(' ');      // 6: pwmL
  Serial.print(current_pwm_R); Serial.print(' ');      // 7: pwmR
  Serial.print(kp, 3); Serial.print(' ');              // 8: kp
  Serial.print(ki, 3); Serial.print(' ');              // 9: ki
  Serial.print(kd, 3); Serial.print(' ');              // 10: kd
  Serial.print(stopped ? 1 : 0); Serial.print(' ');   // 11: stopped
  Serial.print(mode == MODE_PID ? 0 : 1); Serial.print(' '); // 12: mode
  float avg_ppr = (PULSES_PER_REV_L + PULSES_PER_REV_R) / 2.0f;
  Serial.print(avg_ppr, 1); Serial.print(' '); // 13: ppr (promedio L+R)
  Serial.print(pulsesCountL); Serial.print(' ');       // 14: pulsesL
  Serial.print(pulsesCountR); Serial.print(' ');       // 15: pulsesR
  Serial.print(measuredPPR_L, 1); Serial.print(' ');   // 16: measPPR_L
  Serial.print(measuredPPR_R, 1); Serial.print(' ');   // 17: measPPR_R
  Serial.print(encoderRevolutions); Serial.print(' '); // 18: encoderRevs (izquierdo)
  Serial.print(encoderPulses); Serial.print(' ');      // 19: encoderPulses (izquierdo)
  Serial.print(encoderRevolutionsR); Serial.print(' '); // 20: encoderRevsR (derecho)
  Serial.print(encoderPulsesR); Serial.print(' ');      // 21: encoderPulsesR (derecho)
  
  // Datos de odometría usando PPR específicos
  float dist_L_mm = distance_mm_from_pulses_L(pulsesCountL);
  float dist_R_mm = distance_mm_from_pulses_R(pulsesCountR);
  float revs_L = revolutions_from_pulses_L(pulsesCountL);
  float revs_R = revolutions_from_pulses_R(pulsesCountR);
  Serial.print(dist_L_mm, 1); Serial.print(' ');       // 22: distL_mm (usando PPR_L)
  Serial.print(dist_R_mm, 1); Serial.print(' ');       // 23: distR_mm (usando PPR_R)
  Serial.print(revs_L, 2); Serial.print(' ');          // 24: revs_L (usando PPR_L)
  Serial.println(revs_R, 2);                           // 25: revs_R (usando PPR_R)
}

void process_line(String s) {
  s.trim();
  if (s.length() == 0) return;
  s.toUpperCase();

  // Tokenización simple
  int sp1 = s.indexOf(' ');
  String cmd = (sp1 < 0) ? s : s.substring(0, sp1);
  String rest = (sp1 < 0) ? "" : s.substring(sp1 + 1);

  if (cmd == F("PARADA")) { cmd_parada(); return; }
  if (cmd == F("BRAKE"))  { cmd_brake(); return; }
  if (cmd == F("GET"))    { cmd_get(); return; }
  if (cmd == F("HALL_DEBUG")) { cmd_hall_debug(); return; }
  if (cmd == F("ODOMETRY")) { cmd_odometry(); return; }
  if (cmd == F("KP"))     { float v = rest.toFloat(); cmd_kp(v); return; }
  if (cmd == F("KI"))     { float v = rest.toFloat(); cmd_ki(v); return; }
  if (cmd == F("KD"))     { float v = rest.toFloat(); cmd_kd(v); return; }
  if (cmd == F("PULSES")) { float v = rest.toFloat(); cmd_pulses(v); return; }
  if (cmd == F("PPR_L"))  { float v = rest.toFloat(); cmd_ppr_l(v); return; }
  if (cmd == F("PPR_R"))  { float v = rest.toFloat(); cmd_ppr_r(v); return; }
  if (cmd == F("RESET_PULSES")) { cmd_reset_pulses(); return; }
  if (cmd == F("ADELANTE")) { float v = rest.toFloat(); cmd_adelante(v); return; }
  if (cmd == F("ATRAS"))    { float v = rest.toFloat(); cmd_atras(v); return; }
  if (cmd == F("GIRAR_IZQ")){ float v = rest.toFloat(); cmd_girar_izq(v); return; }
  if (cmd == F("GIRAR_DER")){ float v = rest.toFloat(); cmd_girar_der(v); return; }

  if (cmd == F("VEL")) {
    int sp = rest.indexOf(' ');
    if (sp < 0) { sendERR(F("VEL args")); return; }
    float vL = rest.substring(0, sp).toFloat();
    float vR = rest.substring(sp + 1).toFloat();
    cmd_vel(vL, vR);
    return;
  }

  if (cmd == F("PWM")) {
    int sp = rest.indexOf(' ');
    if (sp < 0) { sendERR(F("PWM args")); return; }
    int pL = rest.substring(0, sp).toInt();
    int pR = rest.substring(sp + 1).toInt();
    cmd_pwm(pL, pR);
    return;
  }

  // Comandos de control de velocidad del robot
  if (cmd == F("VEL_UP"))     { cmd_vel_up(); return; }
  if (cmd == F("VEL_DOWN"))   { cmd_vel_down(); return; }
  if (cmd == F("VEL_FORWARD")) { cmd_vel_forward(); return; }
  if (cmd == F("VEL_BACKWARD")) { cmd_vel_backward(); return; }
  if (cmd == F("VEL_STOP"))   { cmd_vel_stop(); return; }
  if (cmd == F("VEL_INC"))    { float v = rest.toFloat(); cmd_vel_increment(v); return; }

  // Comandos de alineación de velocidad
  if (cmd == F("ALIGN_CALIBRATE")) { cmd_align_calibrate(); return; }
  if (cmd == F("ALIGN_ENABLE"))  { cmd_align_enable(true); return; }
  if (cmd == F("ALIGN_DISABLE")) { cmd_align_enable(false); return; }
  if (cmd == F("ALIGN_STATUS"))  { cmd_align_status(); return; }
  if (cmd == F("ALIGN_TOLERANCE")) { float v = rest.toFloat(); cmd_align_tolerance(v); return; }
  if (cmd == F("ALIGN_FACTORS")) {
    int sp = rest.indexOf(' ');
    if (sp > 0) {
      float factorL = rest.substring(0, sp).toFloat();
      float factorR = rest.substring(sp + 1).toFloat();
      cmd_align_factors(factorL, factorR);
    } else {
      sendERR(F("ALIGN_FACTORS args"));
    }
    return;
  }

  sendERR(F("unknown"));
}

void serialEvent() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inbuf.length()) { process_line(inbuf); inbuf = ""; }
    } else {
      if (inbuf.length() < 80) inbuf += c; // limitar tamaño
    }
  }
}

// ------------------------ Setup/Loop ------------------------
void setup() {
  Serial.begin(115200);
  // Pines
  pinMode(HALL_L, INPUT_PULLUP);
  pinMode(HALL_R, INPUT_PULLUP);
  pinMode(ENCODER_L, INPUT_PULLUP);  // Pin 5 para encoder óptico izquierdo
  pinMode(ENCODER_R, INPUT_PULLUP);  // Pin 4 para encoder óptico derecho
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(BRAKE_L, OUTPUT);
  pinMode(BRAKE_R, OUTPUT);
  pinMode(STOP_L, OUTPUT);
  pinMode(STOP_R, OUTPUT);

  // Estado inicial - INTERCAMBIADO L ↔ R
  digitalWrite(DIR_L, LOW);    // Lógicamente izquierdo (físicamente derecho) en posición "adelante"
  digitalWrite(DIR_R, HIGH);   // Lógicamente derecho (físicamente izquierdo) en posición "adelante"
  setStop(true); // iniciar en STOP por seguridad
  setBrake(false); // BRAKE liberado al inicio
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);

  // Interrupciones - Solo Hall sensors (encoder usa polling)
  attachInterrupt(digitalPinToInterrupt(HALL_L), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_R), isrRight, RISING);

  lastTickL = lastTickR = millis();
  
  // Inicializar contadores de pulsos y encoder
  pulsesCountL = pulsesCountR = 0;
  lastPulsesL = lastPulsesR = 0;
  lastRevTimeL = lastRevTimeR = millis();
  encoderRevolutions = encoderPulses = 0;
  encoderRevolutionsR = encoderPulsesR = 0;
  lastEncoderTime = lastEncoderTimeR = millis();
  
  // Inicializar PPR medidos con valores específicos
  measuredPPR_L = PULSES_PER_REV_L;  // 45 para motor L
  measuredPPR_R = PULSES_PER_REV_R;  // 55 para motor R

  Serial.println(F("READY - Hall pins 2,3 + Encoders pins 4,5 (polling)"));
  Serial.print(F("PPR_L="));
  Serial.print(PULSES_PER_REV_L, 1);
  Serial.print(F(" PPR_R="));
  Serial.println(PULSES_PER_REV_R, 1);
}

void loop() {
  uint32_t now = millis();
  float dt = (now - prevMillis) / 1000.0f;
  prevMillis = now;

  // Verificar encoders por polling (pines 4 y 5 no soportan interrupciones externas)
  checkEncoder();    // Encoder izquierdo pin 5
  checkEncoderR();   // Encoder derecho pin 4

  // Sistema de alineación automática en tiempo real
  static uint32_t lastAlignmentCheck = 0;
  if (speed_alignment_enabled && robot_moving && (now - lastAlignmentCheck) > 1000) { // Cada 1 segundo
    float speed_diff = abs(rpmL - rpmR);
    if (speed_diff > alignment_tolerance && abs(rpmL) > 5.0f && abs(rpmR) > 5.0f) {
      // Aplicar ajuste suave gradual
      float adjustment_factor = 0.95f; // Ajuste del 5% por vez
      if (abs(rpmL) > abs(rpmR)) {
        speed_correction_L *= adjustment_factor;
      } else {
        speed_correction_R *= adjustment_factor;
      }
      
      // Limitar factores de corrección
      speed_correction_L = constrain(speed_correction_L, 0.5f, 1.5f);
      speed_correction_R = constrain(speed_correction_R, 0.5f, 1.5f);
      
      // Aplicar la nueva corrección
      apply_robot_velocity();
    }
    lastAlignmentCheck = now;
  }

  // Leer comandos del puerto serial
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inbuf.length()) { process_line(inbuf); inbuf = ""; }
    } else {
      if (inbuf.length() < 80) inbuf += c; // limitar tamaño
    }
  }

  // Actualizar RPM medido a partir de intervalos ISR
  uint32_t iL, iR, tL, tR;
  noInterrupts();
    iL = intervalL; iR = intervalR; tL = lastTickL; tR = lastTickR;
  interrupts();

  if (now - tL > HALL_TIMEOUT_MS) rpmL = 0; else rpmL = rpm_from_interval_ppr(iL, PULSES_PER_REV_L);
  if (now - tR > HALL_TIMEOUT_MS) rpmR = 0; else rpmR = rpm_from_interval_ppr(iR, PULSES_PER_REV_R);

  if (stopped) {
    // mantener salida a 0
    current_pwm_L = current_pwm_R = 0;
    analogWrite(PWM_L, 0);
    analogWrite(PWM_R, 0);
    return;
  }

  if (braked) {
    // En modo BRAKE, mantener freno electromagnético activo
    // Los pines ya están en BRAKE_LEVEL y PWM en 0
    return;
  }

  if (mode == MODE_PWM_DIRECT) {
    // En modo PWM directo, nada que hacer aquí; salidas se fijan en el comando PWM
    return;
  }

  // Modo PID: calcular PWM basado en error de RPM
  float outL = pidStep(tgtRpmL, rpmL, errSumL, prevErrL, dt);
  float outR = pidStep(tgtRpmR, rpmR, errSumR, prevErrR, dt);

  // Convertir salida PID a duty (0..255); se puede usar un feed-forward simple proporcional al tgt
  float ffL = map(constrain((long)round(tgtRpmL), 0, (long)RPM_ABS_MAX), 0, (long)RPM_ABS_MAX, 0, PWM_MAX);
  float ffR = map(constrain((long)round(tgtRpmR), 0, (long)RPM_ABS_MAX), 0, (long)RPM_ABS_MAX, 0, PWM_MAX);

  int pwmL = (int)constrain(ffL + outL, PWM_MIN, PWM_MAX);
  int pwmR = (int)constrain(ffR + outR, PWM_MIN, PWM_MAX);

  // Actualizar variables de seguimiento de PWM
  current_pwm_L = pwmL;
  current_pwm_R = pwmR;

  analogWrite(PWM_L, pwmL);
  analogWrite(PWM_R, pwmR);
}
