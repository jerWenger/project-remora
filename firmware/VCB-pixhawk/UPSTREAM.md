# Provenance

Upstream: https://github.com/cttdev/VCB, commit `aa3dbf85dff51dff5bd71f38ac347260dad482fa`
("Initial commit."), cloned 2026-10-16.

Unchanged from upstream: `Drivers/` (STM32Cube FW_H7 V1.12.1 HAL + CMSIS), `Core/Startup/`,
`Core/Src/{syscalls.c,sysmem.c,system_stm32h7xx.c}`, `Core/Inc/{main.h,stm32h7xx_hal_conf.h}`,
`STM32H723VGTX_RAM.ld`, `.project`, `.cproject`, `.mxproject`.

| File | Change |
|---|---|
| `Core/Src/main.cpp` | Rewritten application code; CubeMX init functions kept verbatim except USART2 removed and TIM2/TIM5 added |
| `Core/Src/stm32h7xx_hal_msp.c` | USART2 MSP removed; `HAL_TIM_IC_MspInit/DeInit` for TIM2/TIM5 added |
| `Core/Src/stm32h7xx_it.c`, `Core/Inc/stm32h7xx_it.h` | USART2 and DMA1 stream 0 handlers removed; TIM2/TIM5 handlers added |
| `Core/Src/eprop.cpp`, `Core/Inc/eprop.h` | integer-permille API with the value byte clamped to 0..0x7F (`EPROP_FULL_SCALE_VALUE`), `sendStop()`, `lastCommand()` readback for the telemetry, bounded UART timeout, one frame builder for both addresses; frame bytes unchanged |
| `Core/Src/mcb.cpp`, `Core/Inc/mcb.h` | bounded UART timeouts, dead code removed |
| `Core/Src/crsf.cpp`, `Core/Inc/crsf.h`, `Core/Src/sm.cpp`, `Core/Inc/sm.h` | removed (RC input, generic state machine) |
| `Core/Inc/config.h`, `Core/Inc/pwm_in.h`, `Core/Src/pwm_in.c` | new |
| `Core/Inc/console_cmd.h`, `Core/Src/console_cmd.c` | new: console line assembly and command parser (bench mode, stop, clear); HAL-free |
| `VCB.ioc` | hand-edited to match (not opened in CubeMX) |
| `STM32H723VGTX_FLASH.ld` | `.dma_buffer` made `(NOLOAD)`; upstream's loadable section at 0x38000000 made `objcopy -O binary` emit an 805 MB file |
| `.gitignore` | `build/` added |
| `Makefile`, `README.md`, `UPSTREAM.md` | new |

To diff against upstream:

```
git clone https://github.com/cttdev/VCB /tmp/VCB && git -C /tmp/VCB checkout aa3dbf8
diff -ruN --exclude=Drivers --exclude=build --exclude=.git /tmp/VCB firmware/VCB-pixhawk
```
