#include "eprop.h"
#include "config.h"

static_assert(EPROP_FULL_SCALE_VALUE >= 1 && EPROP_FULL_SCALE_VALUE <= 0x7F,
              "the ePropulsion value byte is 7 bits wide");

void EProp::init(UART_HandleTypeDef* huart) {
  huart_ = huart;
  last_cmd_[RIGHT] = last_cmd_[LEFT] = 0;
}

void EProp::sendOne(uint8_t address, Side side, int32_t permille) {
  /* Fence 1: never more than 100 % in either direction. */
  if (permille > 1000)  permille = 1000;
  if (permille < -1000) permille = -1000;
  bool reverse = (permille < 0);
  int32_t mag = reverse ? -permille : permille;

  /* 0..1000 -> 0..EPROP_FULL_SCALE_VALUE, nearest integer, no floats. */
  int32_t value = (mag * EPROP_FULL_SCALE_VALUE + 500) / 1000;

  /* Fence 2: the byte itself. Redundant by construction; kept so that a later
   * change to the mapping cannot reintroduce an out-of-range byte. */
  if (value > 0x7F) value = 0x7F;
  if (value < 0)    value = 0;

  uint8_t frame[8];
  frame[0] = 0x28;                                      // Head code
  frame[1] = address;                                   // EPROP_ADDR_RIGHT / EPROP_ADDR_LEFT
  frame[2] = 0x03;                                      // Data length
  frame[3] = 0x2C;                                      // Throttle command
  frame[4] = reverse ? 0x00 : 0x01;                     // Direction
  frame[5] = static_cast<uint8_t>(value);               // Throttle value
  frame[6] = frame[2] ^ frame[3] ^ frame[4] ^ frame[5]; // Checksum: XOR of bytes 2..5
  frame[7] = 0x29;                                      // End code

  HAL_UART_Transmit(huart_, frame, sizeof(frame), UART_TX_TIMEOUT_MS);
  last_cmd_[side] = static_cast<int16_t>(reverse ? -value : value);
}

/* Addresses as measured on the boat 2026-10-25: address 1 turns the right
 * (starboard) motor, address 2 the left (port) one. cttdev's comment had them
 * the other way round; the frame bytes are otherwise unchanged. */
void EProp::sendThrottleCommand(int32_t right_permille, int32_t left_permille) {
  sendOne(EPROP_ADDR_RIGHT, RIGHT, right_permille);
  sendOne(EPROP_ADDR_LEFT, LEFT, left_permille);
}

void EProp::sendStop(void) {
  sendOne(EPROP_ADDR_RIGHT, RIGHT, 0);
  sendOne(EPROP_ADDR_LEFT, LEFT, 0);
}
