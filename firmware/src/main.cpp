// eyesteer MCU firmware: reads quadrature encoders, runs per-wheel PID at PID_RATE_HZ, drives the
// motor driver, enforces the 200 ms command watchdog and the physical e-stop, and (walker build)
// reads the HX711 handle load cells + the grip sensor. All protocol/PID/watchdog logic lives in
// lib/eyesteer_fw (shared with the host tests and the HIL emulator).
#include <Arduino.h>
#include <limits.h>

#include "eyesteer_fw.h"
#include "robot_constants.h"

#if defined(EYESTEER_BOARD_ESP32)
#define ES_ISR_ATTR IRAM_ATTR
#else
#define ES_ISR_ATTR
#endif

static es_state_t S;
static volatile int32_t isr_l = 0, isr_r = 0;
static uint32_t next_pid_ms = 0;
static uint32_t next_sensor_ms = 0;

static void ES_ISR_ATTR isr_left() {
  bool a = digitalRead(PIN_LEFT_ENC_A), b = digitalRead(PIN_LEFT_ENC_B);
  isr_l += (a == b) ? (INVERT_LEFT_ENCODER ? -1 : 1) : (INVERT_LEFT_ENCODER ? 1 : -1);
}

static void ES_ISR_ATTR isr_right() {
  bool a = digitalRead(PIN_RIGHT_ENC_A), b = digitalRead(PIN_RIGHT_ENC_B);
  isr_r += (a == b) ? (INVERT_RIGHT_ENCODER ? -1 : 1) : (INVERT_RIGHT_ENCODER ? 1 : -1);
}

static void drive(uint8_t pwm_pin, uint8_t dir_pin, int32_t pwm, bool invert) {
  if (invert) pwm = -pwm;
  bool fwd = pwm >= 0;
  int32_t mag = fwd ? pwm : -pwm;
  if (mag > MAX_PWM) mag = MAX_PWM;
  // PWM + direction driver (Cytron MDD10A style). For an L298N wire IN1 = dir pin, IN2 = dir pin through
  // an inverter (e.g. one NPN stage), EN = pwm pin (see DEPLOY.md, "Motor driver wiring").
  digitalWrite(dir_pin, fwd ? HIGH : LOW);
  analogWrite(pwm_pin, (int)mag);
}

#if IS_WALKER
// Minimal HX711 bit-bang reader (channel A, gain 128): no library dependency.
static long hx711_read(uint8_t dout, uint8_t sck) {
  if (digitalRead(dout) == HIGH) return LONG_MIN;   // not ready
  long v = 0;
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(sck, HIGH);
    delayMicroseconds(1);
    v = (v << 1) | digitalRead(dout);
    digitalWrite(sck, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(sck, HIGH);
  delayMicroseconds(1);
  digitalWrite(sck, LOW);
  if (v & 0x800000L) v |= ~0xFFFFFFL;   // sign-extend 24 bit
  return v;
}
static long hx_off_l = 0, hx_off_r = 0;
static float force_l = 0, force_r = 0;
#endif

void setup() {
  Serial.begin(BAUD_RATE);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT_PULLUP);     // e-stop: normally-closed to GND; open (HIGH) = pressed / wire cut
  analogWrite(PIN_LEFT_PWM, 0);
  analogWrite(PIN_RIGHT_PWM, 0);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), isr_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), isr_right, CHANGE);
  es_init(&S, PID_KP, PID_KD, PID_KI, PID_KO, MAX_PWM, MAX_COUNTS_PER_LOOP, MAX_ACCEL_COUNTS_PER_LOOP2,
          WATCHDOG_MS, BAUD_RATE, ROBOT_NAME);
#if IS_WALKER
  pinMode(PIN_GRIP, INPUT_PULLUP);
  pinMode(PIN_HX_LEFT_DOUT, INPUT);
  pinMode(PIN_HX_RIGHT_DOUT, INPUT);
  pinMode(PIN_HX_LEFT_SCK, OUTPUT);
  pinMode(PIN_HX_RIGHT_SCK, OUTPUT);
  delay(400);   // tare with no hands on the handles
  long l = LONG_MIN, r = LONG_MIN;
  for (uint8_t i = 0; i < 50 && (l == LONG_MIN || r == LONG_MIN); i++) {
    if (l == LONG_MIN) l = hx711_read(PIN_HX_LEFT_DOUT, PIN_HX_LEFT_SCK);
    if (r == LONG_MIN) r = hx711_read(PIN_HX_RIGHT_DOUT, PIN_HX_RIGHT_SCK);
    delay(15);
  }
  hx_off_l = (l == LONG_MIN) ? 0 : l;
  hx_off_r = (r == LONG_MIN) ? 0 : r;
#endif
  next_pid_ms = millis();
}

void loop() {
  uint32_t now = millis();
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (es_feed_char(&S, c, now)) Serial.print(es_take_response(&S));
  }
  // physical e-stop: independent of ROS, cuts the motors immediately
  es_set_estop_input(&S, digitalRead(PIN_ESTOP) == HIGH);
  if ((int32_t)(now - next_pid_ms) >= 0) {
    next_pid_ms += 1000 / PID_RATE_HZ;
    noInterrupts();
    int32_t dl = isr_l, dr = isr_r;
    isr_l = isr_r = 0;
    interrupts();
    es_add_encoder(&S, dl, dr);
    es_pid_step(&S, now);
    drive(PIN_LEFT_PWM, PIN_LEFT_DIR, es_get_pwm(&S, 0), INVERT_LEFT_MOTOR);
    drive(PIN_RIGHT_PWM, PIN_RIGHT_DIR, es_get_pwm(&S, 1), INVERT_RIGHT_MOTOR);
  }
#if IS_WALKER
  if ((int32_t)(now - next_sensor_ms) >= 0) {
    next_sensor_ms = now + 25;   // 40 Hz
    long l = hx711_read(PIN_HX_LEFT_DOUT, PIN_HX_LEFT_SCK);
    long r = hx711_read(PIN_HX_RIGHT_DOUT, PIN_HX_RIGHT_SCK);
    if (l != LONG_MIN) force_l = (float)(l - hx_off_l) / LOAD_CELL_SCALE_LEFT;
    if (r != LONG_MIN) force_r = (float)(r - hx_off_r) / LOAD_CELL_SCALE_RIGHT;
    bool grip = GRIP_ACTIVE_LOW ? digitalRead(PIN_GRIP) == LOW : digitalRead(PIN_GRIP) == HIGH;
    es_set_walker_sensors(&S, force_l, force_r, grip ? 1 : 0);
  }
#endif
}
