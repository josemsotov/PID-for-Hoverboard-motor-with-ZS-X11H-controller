// DualMotor_PID - Control PID para 2 motores con ZS-X11H - Arduino Uno
// Serie a 115200 baudios, control por comandos seriales y GUI Python
// Fecha: 2025-09-19

/*
Resumen de pines (Arduino Uno):
  Motor Izquierdo:
    HALL_L  -> 2  (INT0)
    PWM_L   -> 6  (PWM)
    DIR_L   -> 7
    BRAKE_L -> 10 (Digital)
    STOP_L  -> 12 (Digital)
  Motor Derecho:
    HALL_R  -> 3  (INT1)
    PWM_R   -> 9  (PWM)
    DIR_R   -> 8
    BRAKE_R -> 11 (Digital PWM-capable)
    STOP_R  -> 13 (Digital)
  Reservados: 4, 5 (no se usan) | UART USB: 0,1 (no se usan para motores)

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
const uint8_t HALL_L = 2;   // INT0
const uint8_t HALL_R = 3;   // INT1
const uint8_t PWM_L  = 6;   // PWM (Timer0)
const uint8_t DIR_L  = 7;
const uint8_t PWM_R  = 9;   // PWM (Timer1)
const uint8_t DIR_R  = 8;
const uint8_t BRAKE_L = 10;  // Digital - BRAKE motor izquierdo
const uint8_t BRAKE_R = 11;  // Digital PWM-capable - BRAKE motor derecho
const uint8_t STOP_L = 12;   // Digital - STOP motor izquierdo
const uint8_t STOP_R = 13;   // Digital - STOP motor derecho

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

// Conversión: mm/s <-> RPM
inline float rpm_from_mmps(float mmps) { return (mmps / 1000.0f) * 60.0f / WHEEL_CIRCUMF_M; }
inline float mmps_from_rpm(float rpm) { return (rpm * WHEEL_CIRCUMF_M / 60.0f) * 1000.0f; }

// ------------------------ Estado de control -----------------------------
volatile uint32_t lastTickL = 0, lastTickR = 0; // ms de última detección
volatile uint32_t intervalL = 0, intervalR = 0; // ms entre tics

// Medidas filtradas/calculadas (actualizadas en loop)
float rpmL = 0, rpmR = 0;    // medido
float tgtRpmL = 0, tgtRpmR = 0; // objetivo

// PID por rueda
float kp = 0.15f, ki = 0.7f, kd = 0.001f;   // base del repo, ajustar en campo
float errSumL = 0, prevErrL = 0, errSumR = 0, prevErrR = 0;
float maxSum = 50.0f;                        // anti-windup

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
  uint32_t now = millis();
  intervalL = now - lastTickL;
  lastTickL = now;
}

void isrRight() {
  uint32_t now = millis();
  intervalR = now - lastTickR;
  lastTickR = now;
}

// ------------------------ Cálculo de RPM --------------------
inline float rpm_from_interval(uint32_t interval_ms) {
  if (interval_ms == 0) return 0.0f;
  // rpm = 60000 / (interval_ms * pulsesPerRev)
  return 60000.0f / (float(interval_ms) * PULSES_PER_REV);
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
String inbuf;

void sendACK() { Serial.println(F("ACK")); }
void sendERR(const __FlashStringHelper* msg) { Serial.print(F("ERR ")); Serial.println(msg); }

void cmd_parada() {
  setBrake(false);  // Liberar BRAKE primero
  setStop(true);    // Activar STOP
  setDirAndPwmLeft(0, 0);
  setDirAndPwmRight(0, 0);
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
  // Mantener dirección previa en función de signo deseado no aplica en PWM directo; usar DIR por GUI si se desea cambiar sentido.
  setDirAndPwmLeft(digitalRead(DIR_L), constrain(pL, PWM_MIN, PWM_MAX));
  setDirAndPwmRight(digitalRead(DIR_R), constrain(pR, PWM_MIN, PWM_MAX));
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
  // Dirección: 0 = adelante, 1 = atrás (ajusta según cableado)
  digitalWrite(DIR_L, (vL_mmps >= 0) ? LOW : HIGH);
  digitalWrite(DIR_R, (vR_mmps >= 0) ? LOW : HIGH);
}

void cmd_vel(float vL_mmps, float vR_mmps) { set_vel_mmps(vL_mmps, vR_mmps); sendACK(); }

void cmd_adelante(float v_mmps) { set_vel_mmps(v_mmps, v_mmps); sendACK(); }
void cmd_atras(float v_mmps)    { set_vel_mmps(-v_mmps, -v_mmps); sendACK(); }
void cmd_girar_izq(float v_mmps){ set_vel_mmps(-v_mmps,  v_mmps); sendACK(); }
void cmd_girar_der(float v_mmps){ set_vel_mmps( v_mmps, -v_mmps); sendACK(); }

void cmd_kp(float v){ kp = v; sendACK(); }
void cmd_ki(float v){ ki = v; sendACK(); }
void cmd_kd(float v){ kd = v; sendACK(); }

void cmd_get() {
  // Telemetría: rpmL rpmR mmpsL mmpsR tgtRpmL tgtRpmR kp ki kd stopped mode
  float mmpsL = mmps_from_rpm(rpmL);
  float mmpsR = mmps_from_rpm(rpmR);
  Serial.print(F("DATA "));
  Serial.print(rpmL, 3); Serial.print(' ');
  Serial.print(rpmR, 3); Serial.print(' ');
  Serial.print(mmpsL, 3); Serial.print(' ');
  Serial.print(mmpsR, 3); Serial.print(' ');
  Serial.print(tgtRpmL, 3); Serial.print(' ');
  Serial.print(tgtRpmR, 3); Serial.print(' ');
  Serial.print(kp, 3); Serial.print(' ');
  Serial.print(ki, 3); Serial.print(' ');
  Serial.print(kd, 3); Serial.print(' ');
  Serial.print(stopped ? 1 : 0); Serial.print(' ');
  Serial.println(mode == MODE_PID ? 0 : 1);
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
  if (cmd == F("KP"))     { float v = rest.toFloat(); cmd_kp(v); return; }
  if (cmd == F("KI"))     { float v = rest.toFloat(); cmd_ki(v); return; }
  if (cmd == F("KD"))     { float v = rest.toFloat(); cmd_kd(v); return; }
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
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(BRAKE_L, OUTPUT);
  pinMode(BRAKE_R, OUTPUT);
  pinMode(STOP_L, OUTPUT);
  pinMode(STOP_R, OUTPUT);

  // Estado inicial
  digitalWrite(DIR_L, LOW);
  digitalWrite(DIR_R, LOW);
  setStop(true); // iniciar en STOP por seguridad
  setBrake(false); // BRAKE liberado al inicio
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);

  // Interrupciones
  attachInterrupt(digitalPinToInterrupt(HALL_L), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_R), isrRight, RISING);

  lastTickL = lastTickR = millis();

  Serial.println(F("READY"));
}

void loop() {
  uint32_t now = millis();
  float dt = (now - prevMillis) / 1000.0f;
  prevMillis = now;

  // Actualizar RPM medido a partir de intervalos ISR
  uint32_t iL, iR, tL, tR;
  noInterrupts();
    iL = intervalL; iR = intervalR; tL = lastTickL; tR = lastTickR;
  interrupts();

  if (now - tL > HALL_TIMEOUT_MS) rpmL = 0; else rpmL = rpm_from_interval(iL);
  if (now - tR > HALL_TIMEOUT_MS) rpmR = 0; else rpmR = rpm_from_interval(iR);

  if (stopped) {
    // mantener salida a 0
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

  analogWrite(PWM_L, pwmL);
  analogWrite(PWM_R, pwmR);
}
