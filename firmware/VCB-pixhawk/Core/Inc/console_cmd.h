/*
 * console_cmd.h - line assembly and parsing for the console (UART4) command
 * input. No HAL dependency, so it can be unit-tested on the host.
 *
 * Commands (case-insensitive, whitespace separated, newline terminated):
 *   bench on          console becomes the command source (only while disarmed)
 *   bench off         back to the PWM inputs
 *   bench R L         thrust in permille for right (ch1) and left (ch2), -1000..1000;
 *                     clamped to +-BENCH_LIMIT_PERMILLE by the caller; must repeat
 *                     at least every BENCH_TIMEOUT_MS or the command counts as lost
 *   stop              fault 9: contactors open, thrusters stopped, latched
 *   clear             un-latch fault 9 (recovery then follows the normal neutral hold)
 *   help              list the commands
 */
#ifndef CONSOLE_CMD_H
#define CONSOLE_CMD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONSOLE_LINE_MAX 48

typedef enum {
  CMD_NONE = 0,       /* empty line */
  CMD_BENCH_ON,
  CMD_BENCH_OFF,
  CMD_BENCH_SET,      /* right_permille / left_permille filled in */
  CMD_STOP,
  CMD_CLEAR,
  CMD_HELP,
  CMD_UNKNOWN         /* text kept in `line` for the error message */
} ConsoleCmdKind;

typedef struct {
  ConsoleCmdKind kind;
  int32_t right_permille;
  int32_t left_permille;
  char line[CONSOLE_LINE_MAX];   /* the line as received, NUL terminated */
} ConsoleCmd;

typedef struct {
  char buf[CONSOLE_LINE_MAX];
  uint8_t len;
  bool overflow;      /* current line too long: discarded at the next newline */
} ConsoleLine;

/* Feed one received byte. Returns true when a complete line is available in
 * `out` (CR and LF both terminate; empty lines return CMD_NONE). A line
 * longer than CONSOLE_LINE_MAX-1 is discarded and reported as CMD_UNKNOWN. */
bool Console_Feed(ConsoleLine* st, char c, ConsoleCmd* out);

/* Parse one NUL-terminated line into `out`. Returns out->kind. */
ConsoleCmdKind Console_Parse(const char* line, ConsoleCmd* out);

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_CMD_H */
