// Host unit tests for lib/eyesteer_fw: parser, PID step response (with a simulated motor), watchdog,
// e-stop latch, walker extension. Build + run: `make firmware-test` (g++), or `pio test -e native`.
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../lib/eyesteer_fw/eyesteer_fw.h"

static int failures = 0;
#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);            \
      failures++;                                                         \
    }                                                                     \
  } while (0)

static const char *send(es_state_t *s, const char *cmd, uint32_t now) {
  static char out[ES_RESP_MAX];
  out[0] = 0;
  for (const char *p = cmd; *p; p++)
    if (es_feed_char(s, *p, now)) strcpy(out, es_take_response(s));
  return out;
}

static void init(es_state_t *s) { es_init(s, 20, 12, 0, 50, 255, 40, 4, 200, 57600, "field_robot"); }

// first-order motor: counts/loop = gain * pwm, with lag
struct Motor { double v; };
static int32_t motor_step(Motor *m, int32_t pwm) {
  double target = 0.12 * pwm;
  m->v += 0.5 * (target - m->v);
  return (int32_t)lround(m->v);
}

static void test_parser() {
  es_state_t s; init(&s);
  CHECK(strcmp(send(&s, "e\r", 0), "0 0\r\n") == 0);
  es_add_encoder(&s, 123, -45);
  CHECK(strcmp(send(&s, "e\r", 0), "123 -45\r\n") == 0);
  CHECK(strcmp(send(&s, "m 10 -10\r", 0), "OK\r\n") == 0);
  CHECK(s.w[0].target == 10 && s.w[1].target == -10 && s.moving);
  CHECK(strcmp(send(&s, "m 1000 0\r", 0), "OK\r\n") == 0);
  CHECK(s.w[0].target == 40);   // clamped to max counts/loop
  CHECK(strcmp(send(&s, "u 30:5:1:40\r", 0), "OK\r\n") == 0);
  CHECK(s.kp == 30 && s.kd == 5 && s.ki == 1 && s.ko == 40);
  CHECK(strcmp(send(&s, "r\r", 0), "OK\r\n") == 0);
  CHECK(s.w[0].enc == 0);
  CHECK(strcmp(send(&s, "b\r", 0), "57600\r\n") == 0);
  CHECK(strcmp(send(&s, "zz\r", 0), "Invalid Command\r\n") == 0);
  CHECK(strcmp(send(&s, "m 5\r", 0), "Invalid Command\r\n") == 0);
  CHECK(strncmp(send(&s, "v\r", 0), "V eyesteer field_robot", 22) == 0);
  CHECK(strcmp(send(&s, "\r", 0), "\r\n") == 0);   // diffdrive_arduino send_empty_msg
  // CRLF tolerated, split packets assembled
  send(&s, "m 3 ", 0);
  CHECK(strcmp(send(&s, "4\r\n", 0), "OK\r\n") == 0 && s.w[1].target == 4);
}

static void test_pid_step_response() {
  es_state_t s; init(&s);
  Motor ml = {0}, mr = {0};
  send(&s, "m 20 20\r", 0);
  int32_t last = 0;
  for (int k = 0; k < 60; k++) {
    uint32_t now = k * 33;
    if (k % 3 == 0) send(&s, "m 20 20\r", now);   // keep the watchdog fed
    es_pid_step(&s, now);
    int32_t dl = motor_step(&ml, es_get_pwm(&s, 0));
    int32_t dr = motor_step(&mr, es_get_pwm(&s, 1));
    es_add_encoder(&s, dl, dr);
    last = dl;
  }
  printf("  PID steady-state counts/loop: %d (target 20)\n", (int)last);
  CHECK(abs(last - 20) <= 2);
  CHECK(es_get_pwm(&s, 0) > 0 && es_get_pwm(&s, 0) <= 255);
}

static void test_small_command_moves() {
  // walker gains (Kp 30, Ko 50): a 1-count-per-loop target must still produce PWM (no truncation)
  es_state_t s; es_init(&s, 30, 10, 0, 50, 255, 42, 4, 200, 57600, "walker");
  Motor m = {0};
  int32_t moved = 0;
  for (int k = 0; k < 90; k++) {
    uint32_t now = k * 33;
    if (k % 3 == 0) send(&s, "m 1 1\r", now);
    es_pid_step(&s, now);
    int32_t d = motor_step(&m, es_get_pwm(&s, 0));
    es_add_encoder(&s, d, d);
    moved += d;
  }
  printf("  1 count/loop target: moved %d counts in 90 loops, pwm %d\n", (int)moved, (int)es_get_pwm(&s, 0));
  CHECK(moved >= 45 && es_get_pwm(&s, 0) > 0);
}

static void test_watchdog() {
  es_state_t s; init(&s);
  Motor m = {0};
  send(&s, "m 20 20\r", 0);
  for (int k = 0; k < 6; k++) { es_pid_step(&s, k * 33); es_add_encoder(&s, motor_step(&m, es_get_pwm(&s, 0)), 0); }
  CHECK(es_get_pwm(&s, 0) != 0 && !s.watchdog_tripped);
  es_pid_step(&s, 199);
  CHECK(!s.watchdog_tripped);
  es_pid_step(&s, 233);   // > 200 ms since the last 'm'
  CHECK(s.watchdog_tripped && es_get_pwm(&s, 0) == 0 && es_get_pwm(&s, 1) == 0 && !s.moving);
  // 'e' polling alone does NOT feed the watchdog
  send(&s, "m 20 20\r", 300);
  es_pid_step(&s, 310);
  CHECK(!s.watchdog_tripped);
  for (uint32_t t = 320; t < 700; t += 33) { send(&s, "e\r", t); es_pid_step(&s, t); }
  CHECK(s.watchdog_tripped && es_get_pwm(&s, 0) == 0);
  // open loop (motor_test) also guarded
  send(&s, "o 100 100\r", 1000);
  es_pid_step(&s, 1010);
  CHECK(es_get_pwm(&s, 0) == 100);
  es_pid_step(&s, 1300);
  CHECK(es_get_pwm(&s, 0) == 0);
}

static void test_estop_latch() {
  es_state_t s; init(&s);
  send(&s, "m 20 20\r", 0);
  es_pid_step(&s, 10);
  CHECK(es_get_pwm(&s, 0) != 0);
  es_set_estop_input(&s, 1);
  es_pid_step(&s, 20);
  CHECK(s.estop_latched && es_get_pwm(&s, 0) == 0);
  send(&s, "m 20 20\r", 30);
  es_pid_step(&s, 40);
  CHECK(es_get_pwm(&s, 0) == 0);                          // commands ignored while latched
  CHECK(strcmp(send(&s, "x\r", 50), "ESTOP\r\n") == 0);   // button still pressed -> cannot clear
  es_set_estop_input(&s, 0);
  CHECK(s.estop_latched);                                  // released, but still latched
  CHECK(strcmp(send(&s, "s\r", 60), "S 1 0 0 0 30\r\n") == 0);
  CHECK(strcmp(send(&s, "x\r", 70), "OK\r\n") == 0 && !s.estop_latched);
  send(&s, "m 20 20\r", 80);
  es_pid_step(&s, 90);
  CHECK(es_get_pwm(&s, 0) != 0);
}

static void test_walker_extension() {
  es_state_t s; init(&s);
  es_set_walker_sensors(&s, 12.34f, -5.0f, 1);
  CHECK(strcmp(send(&s, "h\r", 0), "H 123 -50 1\r\n") == 0);
}

int main() {
  printf("eyesteer_fw host tests (state %d bytes)\n", es_state_size());
  test_parser();
  test_pid_step_response();
  test_small_command_moves();
  test_watchdog();
  test_estop_latch();
  test_walker_extension();
  printf(failures ? "FAILED: %d\n" : "ALL PASSED\n", failures);
  return failures ? 1 : 0;
}
