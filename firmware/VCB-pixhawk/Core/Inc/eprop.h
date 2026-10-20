#ifndef EPROP_H
#define EPROP_H

#include "main.h"
#include <stdint.h>

/*
 * ePropulsion protocol-converter driver (UART5, RS-485, 38400 8N1).
 * Frame (from cttdev/VCB): 28 addr 03 2C dir value xor 29
 *   addr  1 = left thruster, 2 = right thruster
 *   dir   1 = forward, 0 = reverse
 *   value 0..EPROP_FULL_SCALE_VALUE (0x7F in cttdev's code = 100 %)
 *   xor   = bytes 2..5 XORed
 *
 * The value byte must never leave 0..0x7F. The deployed PWM build is reported
 * to have sent out-of-range values when a pulse read a few us past the end of
 * the mapped range (1900 commanded, 1905 measured), and the converter's
 * reaction was to drop the motors: loss of thrust at full command. So this
 * driver takes integer permille, clamps to +-1000 first, converts with
 * integer math and clamps the byte again; the main loop pins the pulse width
 * to full scale before it gets here (width_to_permille). Three independent
 * fences, each sufficient on its own.
 */
class EProp {
  public:
    /* Index for lastCommand(). RIGHT = address 2 = VCB ch1, LEFT = address 1 = VCB ch2. */
    enum Side { RIGHT = 0, LEFT = 1 };

    void init(UART_HandleTypeDef* huart);

    /* Thrust per side in permille, -1000..+1000 (negative = reverse). Values
     * out of range are clamped, never rejected. Sends right (address 2) first,
     * then left (address 1), as cttdev/VCB did. */
    void sendThrottleCommand(int32_t right_permille, int32_t left_permille);

    /* Both thrusters to 0, forward direction bit. */
    void sendStop(void);

    /* Last value byte sent for `side`, signed: -value when the direction byte
     * was reverse, +value forward. -127..+127. For the telemetry. */
    int16_t lastCommand(Side side) const { return last_cmd_[side]; }

  private:
    void sendOne(uint8_t address, Side side, int32_t permille);
    UART_HandleTypeDef* huart_ = nullptr;
    int16_t last_cmd_[2] = {0, 0};
};

#endif /* EPROP_H */
