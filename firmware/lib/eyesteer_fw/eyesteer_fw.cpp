// eyesteer_fw: see eyesteer_fw.h. Pure C-style C++ so it builds on AVR (no STL, no exceptions).
#include "eyesteer_fw.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FW_VERSION "1.0.0"

static int32_t clamp32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void reset_pid(es_state_t *s) {
  for (int i = 0; i < 2; i++) {
    es_wheel_t *w = &s->w[i];
    w->target = 0;
    w->ramped = 0;
    w->prev_enc = w->enc;
    w->prev_input = 0;
    w->iterm = 0;
    w->output = 0;
    w->acc = 0;
  }
}

void es_init(es_state_t *s, int32_t kp, int32_t kd, int32_t ki, int32_t ko, int32_t max_pwm,
             int32_t max_counts, int32_t max_accel, uint32_t watchdog_ms, long baud, const char *robot) {
  memset(s, 0, sizeof(*s));
  s->kp = kp; s->kd = kd; s->ki = ki; s->ko = ko > 0 ? ko : 1;
  s->max_pwm = max_pwm;
  s->max_counts = max_counts;
  s->max_accel = max_accel > 0 ? max_accel : 1;
  s->watchdog_ms = watchdog_ms;
  s->baud = baud;
  s->robot = robot;
  s->grip = 0;
  reset_pid(s);
}

static void respond(es_state_t *s, const char *txt) {
  strncpy(s->resp, txt, ES_RESP_MAX - 3);
  s->resp[ES_RESP_MAX - 3] = 0;
  strcat(s->resp, "\r\n");
  s->resp_ready = 1;
}

// parse "<int> <int>" after the command letter; returns number of ints parsed
static int parse_two(const char *p, long *a, long *b) {
  char *end;
  while (*p == ' ') p++;
  if (!*p) return 0;
  *a = strtol(p, &end, 10);
  if (end == p) return 0;
  p = end;
  while (*p == ' ') p++;
  if (!*p) return 1;
  *b = strtol(p, &end, 10);
  return end == p ? 1 : 2;
}

static void exec_line(es_state_t *s, uint32_t now_ms) {
  char buf[ES_RESP_MAX];
  const char *ln = s->line;
  while (*ln == ' ') ln++;
  char cmd = ln[0];
  const char *args = cmd ? ln + 1 : ln;
  long a = 0, b = 0;
  switch (cmd) {
    case 'e':
      snprintf(buf, sizeof(buf), "%ld %ld", (long)s->w[0].enc, (long)s->w[1].enc);
      respond(s, buf);
      break;
    case 'm':
      if (parse_two(args, &a, &b) != 2) { respond(s, "Invalid Command"); break; }
      s->last_cmd_ms = now_ms;
      s->watchdog_tripped = 0;
      s->open_loop = 0;
      if (s->estop_latched) { respond(s, "OK"); break; }   // accepted but ignored while e-stopped
      s->w[0].target = clamp32((int32_t)a, -s->max_counts, s->max_counts);
      s->w[1].target = clamp32((int32_t)b, -s->max_counts, s->max_counts);
      // a zero command still ramps down under PID control (deceleration limit)
      s->moving = (a != 0 || b != 0 || s->w[0].ramped != 0 || s->w[1].ramped != 0);
      respond(s, "OK");
      break;
    case 'o':
      if (parse_two(args, &a, &b) != 2) { respond(s, "Invalid Command"); break; }
      s->last_cmd_ms = now_ms;
      s->watchdog_tripped = 0;
      if (s->estop_latched) { respond(s, "OK"); break; }
      reset_pid(s);
      s->open_loop = 1;
      s->w[0].acc = (int32_t)a * s->ko;
      s->w[1].acc = (int32_t)b * s->ko;
      s->moving = (a != 0 || b != 0);
      s->w[0].output = clamp32((int32_t)a, -s->max_pwm, s->max_pwm);
      s->w[1].output = clamp32((int32_t)b, -s->max_pwm, s->max_pwm);
      respond(s, "OK");
      break;
    case 'r':
      s->w[0].enc = s->w[1].enc = 0;
      s->w[0].prev_enc = s->w[1].prev_enc = 0;
      respond(s, "OK");
      break;
    case 'u': {
      long v[4] = {0, 0, 0, 0};
      const char *p = args;
      int n = 0;
      while (*p == ' ') p++;
      while (n < 4 && *p) {
        char *end;
        v[n] = strtol(p, &end, 10);
        if (end == p) break;
        n++;
        p = end;
        if (*p == ':') p++;
      }
      if (n != 4) { respond(s, "Invalid Command"); break; }
      s->kp = v[0]; s->kd = v[1]; s->ki = v[2]; s->ko = v[3] > 0 ? v[3] : 1;
      respond(s, "OK");
      break;
    }
    case 'b':
      snprintf(buf, sizeof(buf), "%ld", s->baud);
      respond(s, buf);
      break;
    case 's':
      snprintf(buf, sizeof(buf), "S %d %d %d %d %lu", s->estop_latched, s->estop_input, s->watchdog_tripped,
               s->moving, (unsigned long)(now_ms - s->last_cmd_ms));
      respond(s, buf);
      break;
    case 'h': {
      // forces as integer decinewtons (AVR snprintf has no %f)
      long fl = (long)(s->force_l * 10.0f), fr = (long)(s->force_r * 10.0f);
      snprintf(buf, sizeof(buf), "H %ld %ld %d", fl, fr, s->grip);
      respond(s, buf);
      break;
    }
    case 'x':
      if (s->estop_input) { respond(s, "ESTOP"); break; }
      s->estop_latched = 0;
      reset_pid(s);
      respond(s, "OK");
      break;
    case 'f':
      if (parse_two(args, &a, &b) < 1 || a < 0) { respond(s, "Invalid Command"); break; }
      s->ff_counts = (int32_t)a;
      respond(s, "OK");
      break;
    case 'v':
      snprintf(buf, sizeof(buf), "V eyesteer %s %s", s->robot ? s->robot : "?", FW_VERSION);
      respond(s, buf);
      break;
    case 0:
      respond(s, "");   // diffdrive_arduino's send_empty_msg sends a bare "\r"
      break;
    default:
      respond(s, "Invalid Command");
  }
}

int es_feed_char(es_state_t *s, char c, uint32_t now_ms) {
  if (c == '\n') return 0;          // tolerate CRLF
  if (c == '\r') {
    s->line[s->line_len] = 0;
    exec_line(s, now_ms);
    s->line_len = 0;
    return s->resp_ready;
  }
  if (s->line_len < ES_LINE_MAX - 1) s->line[s->line_len++] = c;
  else s->line_len = 0;              // overflow: drop the line
  return 0;
}

const char *es_take_response(es_state_t *s) {
  s->resp_ready = 0;
  return s->resp;
}

void es_add_encoder(es_state_t *s, int32_t dl, int32_t dr) {
  s->w[0].enc += dl;
  s->w[1].enc += dr;
}

void es_set_estop_input(es_state_t *s, uint8_t pressed) {
  s->estop_input = pressed ? 1 : 0;
  if (pressed && !s->estop_latched) {
    s->estop_latched = 1;
    reset_pid(s);
    s->moving = 0;
  }
}

void es_set_feedforward(es_state_t *s, int32_t ff_counts) { s->ff_counts = ff_counts > 0 ? ff_counts : 0; }

void es_set_walker_sensors(es_state_t *s, float force_l, float force_r, uint8_t grip) {
  s->force_l = force_l;
  s->force_r = force_r;
  s->grip = grip;
}

void es_pid_step(es_state_t *s, uint32_t now_ms) {
  // watchdog (D9): no valid motion command for watchdog_ms -> stop
  if ((s->moving || s->open_loop) && (now_ms - s->last_cmd_ms) > s->watchdog_ms) {
    s->watchdog_tripped = 1;
    s->moving = 0;
    s->open_loop = 0;
    reset_pid(s);
  }
  if (s->estop_latched) {
    reset_pid(s);
    return;
  }
  if (s->open_loop) return;
  for (int i = 0; i < 2; i++) {
    es_wheel_t *w = &s->w[i];
    int32_t input = w->enc - w->prev_enc;
    w->prev_enc = w->enc;
    if (!s->moving) {
      // keep the derivative term from kicking on restart
      if (w->prev_input != 0 || w->output != 0 || w->ramped != 0) { w->prev_input = 0; w->output = 0; w->acc = 0; w->iterm = 0; w->ramped = 0; }
      continue;
    }
    // acceleration limit on the setpoint
    int32_t d = clamp32(w->target - w->ramped, -s->max_accel, s->max_accel);
    w->ramped += d;
    // ROSArduinoBridge velocity-form PID, but accumulated in units of PWM*Ko: the original divides by Ko
    // every loop, so Kp*err < Ko (e.g. a 1-count error with Kp 30 / Ko 50) never moves the output and a
    // slow command (a slow in-place turn) is ignored forever.
    int32_t err = w->ramped - input;
    int32_t out;
    int32_t acc = w->acc + s->kp * err - s->kd * (input - w->prev_input) + w->iterm;
    int32_t lim = s->max_pwm * s->ko;
    if (acc >= lim) acc = lim;
    else if (acc <= -lim) acc = -lim;
    else w->iterm += s->ki * err;
    w->acc = acc;
    int32_t ff = s->ff_counts > 0 ? w->ramped * s->max_pwm / s->ff_counts : 0;
    out = ff + acc / s->ko;
    if (out > s->max_pwm) out = s->max_pwm;
    else if (out < -s->max_pwm) out = -s->max_pwm;
    w->output = out;
    w->prev_input = input;
  }
  if (s->w[0].target == 0 && s->w[1].target == 0 && s->w[0].ramped == 0 && s->w[1].ramped == 0) {
    s->moving = 0;
    s->w[0].output = s->w[1].output = 0;
  }
}

int32_t es_get_pwm(const es_state_t *s, int wheel) { return s->w[wheel & 1].output; }
int32_t es_get_enc(const es_state_t *s, int wheel) { return s->w[wheel & 1].enc; }
int es_state_size(void) { return (int)sizeof(es_state_t); }
