/*
 * console_cmd.c - see console_cmd.h
 */
#include "console_cmd.h"
#include <string.h>

static int lower(int c) { return (c >= 'A' && c <= 'Z') ? c + ('a' - 'A') : c; }
static bool is_space(char c) { return c == ' ' || c == '\t'; }

/* next token: skips spaces, copies up to n-1 chars, returns pointer past it */
static const char* token(const char* p, char* out, int n)
{
  int i = 0;
  while (*p && is_space(*p)) p++;
  while (*p && !is_space(*p)) {
    if (i < n - 1) out[i++] = (char)lower((unsigned char)*p);
    p++;
  }
  out[i] = '\0';
  return p;
}

/* strict signed decimal integer, at most 5 digits; false on anything else */
static bool parse_int(const char* s, int32_t* out)
{
  if (!*s) return false;
  bool neg = false;
  if (*s == '-' || *s == '+') { neg = (*s == '-'); s++; }
  if (!*s) return false;
  int32_t v = 0; int digits = 0;
  while (*s) {
    if (*s < '0' || *s > '9' || ++digits > 5) return false;
    v = v * 10 + (*s - '0');
    s++;
  }
  *out = neg ? -v : v;
  return true;
}

ConsoleCmdKind Console_Parse(const char* line, ConsoleCmd* out)
{
  memset(out, 0, sizeof(*out));
  strncpy(out->line, line, CONSOLE_LINE_MAX - 1);

  char w1[16], w2[16], w3[16], w4[4];
  const char* p = token(line, w1, sizeof w1);
  p = token(p, w2, sizeof w2);
  p = token(p, w3, sizeof w3);
  token(p, w4, sizeof w4);   /* anything after the third word makes the line invalid */

  if (!w1[0]) { out->kind = CMD_NONE; return out->kind; }
  out->kind = CMD_UNKNOWN;
  if (w4[0]) return out->kind;

  if (!strcmp(w1, "bench")) {
    if (!strcmp(w2, "on")  && !w3[0]) out->kind = CMD_BENCH_ON;
    else if (!strcmp(w2, "off") && !w3[0]) out->kind = CMD_BENCH_OFF;
    else if (w2[0] && w3[0] &&
             parse_int(w2, &out->right_permille) && parse_int(w3, &out->left_permille))
      out->kind = CMD_BENCH_SET;
  } else if (!w2[0]) {
    if (!strcmp(w1, "stop"))  out->kind = CMD_STOP;
    else if (!strcmp(w1, "clear")) out->kind = CMD_CLEAR;
    else if (!strcmp(w1, "help") || !strcmp(w1, "?")) out->kind = CMD_HELP;
  }
  return out->kind;
}

bool Console_Feed(ConsoleLine* st, char c, ConsoleCmd* out)
{
  if (c == '\n' || c == '\r') {
    if (st->len == 0 && !st->overflow) return false;   /* CRLF second half, or blank */
    st->buf[st->len] = '\0';
    if (st->overflow) {
      memset(out, 0, sizeof(*out));
      out->kind = CMD_UNKNOWN;
      strncpy(out->line, st->buf, CONSOLE_LINE_MAX - 1);
    } else {
      Console_Parse(st->buf, out);
    }
    st->len = 0;
    st->overflow = false;
    return out->kind != CMD_NONE;
  }
  if (c == '\0' || (unsigned char)c >= 0x80) return false;   /* noise */
  if (st->len >= CONSOLE_LINE_MAX - 1) { st->overflow = true; return false; }
  st->buf[st->len++] = c;
  return false;
}
