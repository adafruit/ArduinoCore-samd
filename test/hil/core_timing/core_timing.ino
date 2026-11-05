// SPDX-License-Identifier: MIT
//
// HIL sketch (.claude/skills/hil-contract/SKILL.md): the core's time base, read
// by the host against its own monotonic clock. One command per line on Serial
// (the DUT's USB CDC), one reply line each carrying the command's nonce;
// anything else is answered REJECT why=<reason>.
//
// clang-format off
//   INFO <nonce>        HIL_INFO <nonce> stack=<arduino|tinyusb> chip=<samd21|samd51>
//                         f_cpu=<hz> crystalless=<0|1>
//   TIME <nonce>        HIL_RESULT <nonce> time PASS ms=<millis()> us=<micros()>
//   DELAY <nonce> <ms>  HIL_RESULT <nonce> delay PASS ms=<ms> us=<micros() across delay(ms)>
// clang-format on

#include <Arduino.h>

#ifdef USE_TINYUSB
#define STACK "tinyusb"
#else
#define STACK "arduino"
#endif

#ifdef __SAMD51__
#define CHIP "samd51"
#else
#define CHIP "samd21"
#endif

#ifdef CRYSTALLESS
#define IS_CRYSTALLESS 1
#else
#define IS_CRYSTALLESS 0
#endif

#define DELAY_MAX 5000u

static void handle(char *line) {
  char *cmd = strtok(line, " ");
  char *n = strtok(NULL, " ");
  char *a = strtok(NULL, " ");
  char *extra = strtok(NULL, " ");
  if (!cmd || !n || extra) {
    Serial.println("HIL_RESULT - ? REJECT why=malformed");
    return;
  }
  if (!strcmp(cmd, "INFO") && !a) {
    Serial.printf("HIL_INFO %s stack=%s chip=%s f_cpu=%lu crystalless=%d\r\n",
                  n, STACK, CHIP, (unsigned long)F_CPU, IS_CRYSTALLESS);
    return;
  }
  if (!strcmp(cmd, "TIME") && !a) {
    uint32_t ms = millis();
    uint32_t us = micros();
    Serial.printf("HIL_RESULT %s time PASS ms=%lu us=%lu\r\n", n,
                  (unsigned long)ms, (unsigned long)us);
    return;
  }
  if (!strcmp(cmd, "DELAY") && a) {
    char *end;
    unsigned long ms = strtoul(a, &end, 10);
    if (*end || a[0] == '-' || ms == 0 || ms > DELAY_MAX) {
      Serial.printf("HIL_RESULT %s delay REJECT why=length\r\n", n);
      return;
    }
    uint32_t t0 = micros();
    delay(ms);
    uint32_t t1 = micros();
    Serial.printf("HIL_RESULT %s delay PASS ms=%lu us=%lu\r\n", n, ms,
                  (unsigned long)(t1 - t0));
    return;
  }
  Serial.printf("HIL_RESULT %s ? REJECT why=unknown\r\n", n);
}

void setup() { Serial.begin(115200); }

void loop() {
  static char line[64];
  static size_t n;
  // discarding an overlong line through its newline: its tail is no command
  static bool overlong;
  while (Serial.available()) {
    int c = Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      line[n] = 0;
      if (!overlong) {
        handle(line);
      }
      n = 0;
      overlong = false;
    } else if (n < sizeof(line) - 1) {
      line[n++] = (char)c;
    } else {
      overlong = true;
    }
  }
}
