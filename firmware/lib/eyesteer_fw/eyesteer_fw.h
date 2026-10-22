// eyesteer_fw: portable firmware core (no Arduino dependencies).
//
// Compiled three ways from this one source:
//   1. into the Arduino sketch (src/main.cpp) for mega2560 / uno / teensy41 / esp32,
//   2. natively for the host unit tests (test/test_core.cpp, `make firmware-test`),
//   3. as a shared library loaded by the MCU emulator (firmware/emulator/mcu_emulator.py) via ctypes,
// so the emulator used in HIL simulation runs exactly the firmware's parser, PID and watchdog.
//
// Serial protocol: ROSArduinoBridge, as used by diffdrive_arduino (branch humble):
//   "e\r"          -> "<left_counts> <right_counts>\r\n"
//   "m <l> <r>\r"  -> "OK\r\n"   target speed in encoder counts per PID loop (PID_RATE_HZ)
//   "o <l> <r>\r"  -> "OK\r\n"   raw PWM (open loop, motor_test.py)
//   "r\r"          -> "OK\r\n"   reset encoders
//   "u p:d:i:o\r"  -> "OK\r\n"   PID gains
//   "b\r"          -> "<baud>\r\n"
// eyesteer extensions (used by mcu_bridge_node, never by diffdrive_arduino):
//   "s\r"          -> "S <estop_latched> <estop_input> <watchdog_tripped> <moving> <ms_since_cmd>\r\n"
//   "h\r"          -> "H <force_left_N> <force_right_N> <grip>\r\n"   (walker build; zeros otherwise)
//   "x\r"          -> "OK\r\n" clears a latched e-stop if the button is released, else "ESTOP\r\n"
//   "v\r"          -> "V eyesteer <robot> <version>\r\n"
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ES_LINE_MAX 48
#define ES_RESP_MAX 96

typedef struct {
  int32_t target;      // counts per loop
  int32_t ramped;      // acceleration-limited target
  int32_t enc;         // total encoder counts
  int32_t prev_enc;
  int32_t prev_input;
  int32_t iterm;
  int32_t output;      // PWM, signed
} es_wheel_t;

typedef struct {
  es_wheel_t w[2];
  int32_t kp, kd, ki, ko;
  int32_t max_pwm;
  int32_t max_counts;
  int32_t max_accel;
  uint32_t watchdog_ms;
  uint32_t last_cmd_ms;
  uint8_t moving;
  uint8_t open_loop;
  uint8_t watchdog_tripped;
  uint8_t estop_latched;
  uint8_t estop_input;
  float force_l, force_r;
  uint8_t grip;
  char line[ES_LINE_MAX];
  uint8_t line_len;
  char resp[ES_RESP_MAX];
  uint8_t resp_ready;
  long baud;
  const char *robot;
} es_state_t;

void es_init(es_state_t *s, int32_t kp, int32_t kd, int32_t ki, int32_t ko, int32_t max_pwm,
             int32_t max_counts, int32_t max_accel, uint32_t watchdog_ms, long baud, const char *robot);
// Feed one received byte. Returns 1 when a response is ready in s->resp (then call es_take_response).
int es_feed_char(es_state_t *s, char c, uint32_t now_ms);
const char *es_take_response(es_state_t *s);
// Encoder deltas since the last call (from ISRs on the MCU, from MuJoCo in the emulator).
void es_add_encoder(es_state_t *s, int32_t dl, int32_t dr);
// One PID tick at PID_RATE_HZ. Updates s->w[i].output (PWM). Applies watchdog + e-stop.
void es_pid_step(es_state_t *s, uint32_t now_ms);
void es_set_estop_input(es_state_t *s, uint8_t pressed);
void es_set_walker_sensors(es_state_t *s, float force_l, float force_r, uint8_t grip);
int32_t es_get_pwm(const es_state_t *s, int wheel);
int32_t es_get_enc(const es_state_t *s, int wheel);
int es_state_size(void);

#ifdef __cplusplus
}
#endif
