/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : VCB firmware - Pixhawk PWM input, reversible thrusters.
  *
  * Derived from github.com/cttdev/VCB (commit aa3dbf8, CRSF/RC input version).
  * What changed, in one paragraph: the RC receiver (CRSF on USART2) is gone.
  * The two CONT-connector pins PA2/PA3 are now servo-PWM inputs from the
  * Pixhawk (TIM2/TIM5 input capture, pwm_in.c). 1500 us is stop, 2000 us full
  * forward, 1000 us full reverse, limited to +-THRUST_LIMIT_PERMILLE. A width
  * past an end point pins to full scale (never an out-of-range converter
  * byte), and a sign change passes through DIRECTION_CHANGE_STOP_MS of zero.
  * The board arms by itself when both inputs are valid and at neutral for 1 s,
  * then runs the unchanged cttdev contactor start-up sequence (LSC, wait, HSC,
  * bus check) and in RUN forwards the two channels to the ePropulsion
  * converter. PWM loss, or any interlock failure, opens the contactors, sends
  * stop, and disarms (FAULT). FAULT recovers to IDLE after the inputs have
  * been valid and neutral for FAULT_RECOVER_HOLD_MS. All numbers: config.h.
  * Telemetry on UART4 (115200) keeps the deployed bench build's CSV format
  * (mapping / pwm / status lines) so scripts/vcb_logger.py keeps working.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "pwm_in.h"
#include "mcb.h"
#include "eprop.h"
#include "console_cmd.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if defined( __ICCARM__ )
  #define DMA_BUFFER \
      _Pragma("location=\".dma_buffer\"")
#else
  #define DMA_BUFFER \
      __attribute__((section(".dma_buffer")))
#endif
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart9;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */
MCB lars_mcb;
MCB reel_mcb;
EProp eprop;

/* Thruster state numbering is the same as cttdev/VCB and the deployed bench
 * build, so `thruster_state` in the status line keeps its meaning. */
enum ThrusterState
{
  THRUSTER_STATE_FAULT = 0,
  THRUSTER_STATE_IDLE,
  THRUSTER_STATE_STARTUP_LSC,
  THRUSTER_STATE_STARTUP_WAIT,
  THRUSTER_STATE_STARTUP_HSC,
  THRUSTER_STATE_RUN,
  THRUSTER_STATE_COUNT
};

enum FaultCode
{
  FAULT_NONE = 0,
  FAULT_PWM_LOST = 1,                 /* either input invalid while armed */
  FAULT_LSC_ALREADY_CLOSED = 2,       /* interlock faults from here on */
  FAULT_LSC_TIMEOUT = 3,
  FAULT_HSC_ALREADY_CLOSED = 4,
  FAULT_HSC_TIMEOUT = 5,
  FAULT_BUS_LOW_AT_START = 6,
  FAULT_BUS_LOW_IN_RUN = 7,
  FAULT_CONTACTOR_OPENED_IN_RUN = 8,
  FAULT_STOP_COMMAND = 9              /* 'stop' on the console; latched until 'clear' or reset */
};

struct ChannelCmd
{
  PwmInSample s;
  int32_t raw_permille;       /* stick position, -1000..+1000, 0 when invalid */
  int32_t applied_permille;   /* raw clamped to +-THRUST_LIMIT_PERMILLE */
  int32_t sent_permille;      /* what went to the converter last (0 outside RUN); applied after shape_command() */
  bool    cmd_valid;          /* the active command source (PWM or bench) has a usable command */
  bool    neutral;            /* cmd_valid and zero: PWM within ARM_NEUTRAL_WINDOW_US of neutral, or bench 0 */
  int8_t  last_dir;           /* sign of the last non-zero command sent in RUN, 0 = none yet */
  uint32_t last_nonzero_ms;   /* HAL tick of that command */
};

static ChannelCmd chan[PWM_IN_CHANNELS];   /* [0] = ch1 = PA3 = right, [1] = ch2 = PA2 = left */

/* Command source: the PWM inputs (normal) or 'bench R L' lines on the console. */
enum CmdSource { SOURCE_PWM = 0, SOURCE_BENCH = 1 };
static CmdSource cmd_source = SOURCE_PWM;
static int32_t  bench_cmd[PWM_IN_CHANNELS] = {0, 0};   /* [0] right (ch1), [1] left (ch2); permille, clamped */
static uint32_t bench_last_ms = 0;
static bool     bench_have_cmd = false;
static bool     stop_latched = false;                   /* 'stop' received; cleared by 'clear' or reset */

/* Console RX: the ISR fills a ring, the main loop assembles and parses lines. */
static_assert((CONSOLE_RX_RING_SIZE & (CONSOLE_RX_RING_SIZE - 1)) == 0, "ring size must be a power of two");
static uint8_t           console_rx_byte;
static volatile uint8_t  console_ring[CONSOLE_RX_RING_SIZE];
static volatile uint8_t  console_ring_head = 0;   /* ISR writes */
static uint8_t           console_ring_tail = 0;   /* main loop reads */
static volatile uint32_t console_rx_dropped = 0;
static ConsoleLine       console_line;

static bool armed = false;
static ThrusterState thruster_state = THRUSTER_STATE_IDLE;
static FaultCode fault_code = FAULT_NONE;
static uint32_t state_entry_ms = 0;
static uint8_t  state_step = 0;             /* sub-step inside STARTUP_LSC / STARTUP_HSC */
static uint32_t hsc_confirm_ms = 0;
static uint32_t contactor_open_since_ms = 0;
static uint32_t arm_hold_start_ms = 0;
static uint32_t recover_hold_start_ms = 0;
static uint32_t interlock_fault_count = 0;
static bool     fault_latched_notice = false;

static bool estop_present = false;   /* PA9  reads high */
static bool tsms_present = false;    /* PA10 reads high */
static bool lsc_closed = false;      /* LSI (PD4) reads low */
static bool hsc_closed = false;      /* HSI (PD3) reads low */

DMA_BUFFER uint16_t adc_value;                          /* written by ADC DMA; .dma_buffer is NOLOAD, see linker script */
static float bus_voltage = 0.0f;

DMA_BUFFER uint8_t dma_tx_buffer[CONSOLE_TX_BUF_SIZE];  /* persistent buffer for console DMA; memset in main() */

/* schedule */
static uint32_t last_cmd_ms = 0, last_stop_ms = 0, last_mcb_ms = 0;
static uint32_t last_pwm_telem_ms = 0, last_status_telem_ms = 0, last_header_ms = 0, last_idle_note_ms = 0;
static uint8_t  headers_pending = 0;   /* header lines still to print, one per loop iteration */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_UART9_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
static void print_banner(void);
static void print_headers(void);
static void telemetry_update(uint32_t now);
static void print_pwm_line(uint32_t now);
static void print_status_line(uint32_t now);
static void update_leds(uint32_t now);
static void thruster_update(uint32_t now, bool both_valid, bool both_neutral);
static void shape_command(ChannelCmd* c, uint32_t now);
static void reset_command_shaping(void);
static void disarm_to_idle(const char* why);
static void console_start(void);
static void console_update(uint32_t now);
static void handle_console_cmd(const ConsoleCmd* cmd, uint32_t now);
static void open_contactors(void);
static void safe_outputs(void);
static void enter_state(ThrusterState s, const char* msg);
static void fault(FaultCode code, const char* msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* Servo pulse width -> stick position in permille: zero inside the deadband,
 * then a straight line from the deadband edge to full scale (no step). */
static int32_t width_to_permille(uint32_t width_us)
{
  int32_t d = (int32_t)width_us - PWM_NEUTRAL_US;
  int32_t mag = (d < 0) ? -d : d;
  if (mag <= PWM_DEADBAND_US) return 0;
  /* At or past an end point (<= 1000 us, >= 2000 us) the answer is full
   * scale, explicitly: a 1905 or 2010 us pulse means "full", not "out of
   * range". Pulses the capture accepted (PWM_MIN/MAX_WIDTH_US) always land
   * inside +-1000 here, so nothing downstream ever sees more than 100 %. */
  if (mag >= PWM_HALF_SPAN_US) return (d < 0) ? -1000 : 1000;
  int32_t p = (mag - PWM_DEADBAND_US) * 1000 / (PWM_HALF_SPAN_US - PWM_DEADBAND_US);
  return (d < 0) ? -p : p;
}

/* Between `applied` and the converter: a sign change is sent as 0 until
 * DIRECTION_CHANGE_STOP_MS have passed since the last non-zero command of the
 * old direction. Sets c->sent_permille. Called only in RUN. */
static void shape_command(ChannelCmd* c, uint32_t now)
{
  int32_t want = c->applied_permille;
  int8_t dir = (want > 0) - (want < 0);
#if DIRECTION_CHANGE_STOP_MS > 0
  if (dir != 0 && c->last_dir != 0 && dir != c->last_dir &&
      (now - c->last_nonzero_ms) < DIRECTION_CHANGE_STOP_MS)
  {
    want = 0;
    dir = 0;
  }
#endif
  c->sent_permille = want;
  if (dir != 0)
  {
    c->last_dir = dir;
    c->last_nonzero_ms = now;
  }
}

/* Outside RUN the converter gets stop frames; forget the last direction so
 * the first command of the next RUN is not held. */
static void reset_command_shaping(void)
{
  for (uint8_t i = 0; i < PWM_IN_CHANNELS; i++)
  {
    chan[i].sent_permille = 0;
    chan[i].last_dir = 0;
    chan[i].last_nonzero_ms = 0;
  }
}

static inline bool within_neutral_window(uint32_t width_us)
{
  int32_t d = (int32_t)width_us - PWM_NEUTRAL_US;
  return (d <= ARM_NEUTRAL_WINDOW_US) && (d >= -ARM_NEUTRAL_WINDOW_US);
}

static void read_inputs(uint32_t now)
{
  bool bench_fresh = (cmd_source == SOURCE_BENCH) && bench_have_cmd &&
                     ((now - bench_last_ms) <= BENCH_TIMEOUT_MS);
  for (uint8_t i = 0; i < PWM_IN_CHANNELS; i++)
  {
    ChannelCmd* c = &chan[i];
    PwmIn_Snapshot(i, now, &c->s);   /* always measured and reported, whichever source is active */
    if (cmd_source == SOURCE_PWM)
    {
      c->cmd_valid = c->s.valid;
      c->raw_permille = c->s.valid ? width_to_permille(c->s.width_us) : 0;
      c->neutral = c->s.valid && within_neutral_window(c->s.width_us);
    }
    else
    {
      c->cmd_valid = bench_fresh;
      c->raw_permille = bench_fresh ? bench_cmd[i] : 0;
      c->neutral = bench_fresh && (bench_cmd[i] == 0);
    }
    c->applied_permille = clamp_i32(c->raw_permille, -THRUST_LIMIT_PERMILLE, THRUST_LIMIT_PERMILLE);
  }

  estop_present = HAL_GPIO_ReadPin(SDN_ESTOP_OUT_SENSE_GPIO_Port, SDN_ESTOP_OUT_SENSE_Pin) == GPIO_PIN_SET;
  tsms_present  = HAL_GPIO_ReadPin(SDN_TSMS_OUT_SENSE_GPIO_Port, SDN_TSMS_OUT_SENSE_Pin) == GPIO_PIN_SET;
  lsc_closed = HAL_GPIO_ReadPin(LSI_3_3_GPIO_Port, LSI_3_3_Pin) == GPIO_PIN_RESET;
  hsc_closed = HAL_GPIO_ReadPin(HSI_3_3_GPIO_Port, HSI_3_3_Pin) == GPIO_PIN_RESET;

  /* differential 16-bit reading, mid-scale = 0 V (cttdev/VCB) */
  bus_voltage = ((static_cast<float>(adc_value) - ADC_HALF_VALUE) / ADC_HALF_VALUE) * ADC_REF_VOLTAGE * BUS_VOLTAGE_SCALAR;
}

static void open_contactors(void)
{
  HAL_GPIO_WritePin(LSC_CONTROL_GPIO_Port, LSC_CONTROL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HSC_CONTROL_GPIO_Port, HSC_CONTROL_Pin, GPIO_PIN_RESET);
}

/* Contactors open, thrusters told to stop. Called on every fault. */
static void safe_outputs(void)
{
  open_contactors();
  eprop.sendStop();
  last_stop_ms = HAL_GetTick();
}

static void enter_state(ThrusterState s, const char* msg)
{
  thruster_state = s;
  state_entry_ms = HAL_GetTick();
  state_step = 0;
  hsc_confirm_ms = 0;
  contactor_open_since_ms = 0;
  if (msg) printf("%s\r\n", msg);
}

static bool is_interlock_fault(FaultCode code)
{
  return code != FAULT_NONE && code != FAULT_PWM_LOST && code != FAULT_STOP_COMMAND;
}

static void fault(FaultCode code, const char* msg)
{
  fault_code = code;
  if (code == FAULT_STOP_COMMAND) stop_latched = true;
  if (is_interlock_fault(code)) interlock_fault_count++;
  safe_outputs();
  printf("FAULT %u: %s\r\n", (unsigned)code, msg);
  enter_state(THRUSTER_STATE_FAULT, nullptr);
  if (armed)
  {
    armed = false;
    if (code == FAULT_PWM_LOST)
      printf(cmd_source == SOURCE_BENCH ? "DISARMED: bench command lost\r\n" : "DISARMED: PWM lost or invalid\r\n");
    else if (code == FAULT_STOP_COMMAND)
      printf("DISARMED: stop command\r\n");
    else
      printf("DISARMED: thruster startup/run interlock fault\r\n");
  }
  recover_hold_start_ms = 0;
  fault_latched_notice = false;
}

/* Leave any armed state without a fault ('bench off'): contactors open, stop
 * sent, disarmed, and back to IDLE unless the board is in FAULT. */
static void disarm_to_idle(const char* why)
{
  bool running = (thruster_state != THRUSTER_STATE_IDLE && thruster_state != THRUSTER_STATE_FAULT);
  armed = false;
  arm_hold_start_ms = 0;
  if (thruster_state != THRUSTER_STATE_FAULT) safe_outputs();
  printf("DISARMED: %s\r\n", why);
  if (running) enter_state(THRUSTER_STATE_IDLE, "THRUSTER: contactors opened, back to IDLE");
}

/* ---- arming -------------------------------------------------------------- */
static void arming_update(uint32_t now, bool both_valid, bool both_neutral)
{
  if (!armed)
  {
    if (both_valid && both_neutral && thruster_state == THRUSTER_STATE_IDLE)
    {
      if (arm_hold_start_ms == 0)
      {
        arm_hold_start_ms = now ? now : 1;
      }
      else if ((now - arm_hold_start_ms) >= ARM_HOLD_MS)
      {
        armed = true;
        arm_hold_start_ms = 0;
        printf("ARMED: both PWM channels valid and at neutral for %u ms\r\n", (unsigned)ARM_HOLD_MS);
        if (!estop_present || !tsms_present)
        {
          printf("WARNING: E-stop 24V %s, TSMS 24V %s; contactors may not close\r\n",
                 estop_present ? "present" : "ABSENT", tsms_present ? "present" : "ABSENT");
        }
      }
    }
    else
    {
      arm_hold_start_ms = 0;
    }
  }
  else if (!both_valid)
  {
    fault(FAULT_PWM_LOST, cmd_source == SOURCE_BENCH ? "bench command lost: no 'bench R L' within the timeout"
                                                     : "PWM lost or invalid");
  }
  else if (thruster_state != THRUSTER_STATE_RUN && !both_neutral)
  {
    /* Armed but the contactors are not closed yet (IDLE waiting for the bus,
     * or a start-up in progress) and the command is no longer zero: do not
     * carry a non-zero command into RUN. Disarm; arming restarts from neutral. */
    armed = false;
    arm_hold_start_ms = 0;
    printf("DISARMED: throttle left neutral before RUN (ch1 %lu us, ch2 %lu us)\r\n",
           (unsigned long)chan[0].s.width_us, (unsigned long)chan[1].s.width_us);
    if (thruster_state != THRUSTER_STATE_IDLE && thruster_state != THRUSTER_STATE_FAULT)
    {
      open_contactors();
      enter_state(THRUSTER_STATE_IDLE, "THRUSTER: start-up aborted, contactors opened, back to IDLE");
    }
  }
}

/* ---- thruster state machine --------------------------------------------- */
static void thruster_update(uint32_t now, bool both_valid, bool both_neutral)
{
  switch (thruster_state)
  {
    case THRUSTER_STATE_IDLE:
    {
      open_contactors();
      if (!armed || !both_neutral) break;

      const char* blocker = nullptr;
      char note[96];
#if REQUIRE_ESTOP_TSMS_TO_START
      if (!estop_present || !tsms_present)
      {
        snprintf(note, sizeof(note), "THRUSTER IDLE: armed, waiting for E-stop 24V (%s) and TSMS 24V (%s)",
                 estop_present ? "ok" : "absent", tsms_present ? "ok" : "absent");
        blocker = note;
      }
#endif
      if (!blocker && bus_voltage >= IDLE_MAX_BUS_VOLTAGE_TO_START)
      {
        snprintf(note, sizeof(note), "THRUSTER IDLE: armed, but bus is %.2f V (need < %.1f V to start)",
                 bus_voltage, IDLE_MAX_BUS_VOLTAGE_TO_START);
        blocker = note;
      }
      if (blocker)
      {
        if ((now - last_idle_note_ms) >= IDLE_NOTE_PERIOD_MS)
        {
          last_idle_note_ms = now;
          printf("%s\r\n", blocker);
        }
        break;
      }
      enter_state(THRUSTER_STATE_STARTUP_LSC, "THRUSTER IDLE: armed, neutral and bus low, transitioning to STARTUP_LSC");
      break;
    }

    case THRUSTER_STATE_STARTUP_LSC:
    {
      if (state_step == 0)
      {
        if (lsc_closed)
        {
          fault(FAULT_LSC_ALREADY_CLOSED, "THRUSTER STARTUP_LSC: LSI pin is not SET before starting, contactor may already be closed");
          break;
        }
        HAL_GPIO_WritePin(LSC_CONTROL_GPIO_Port, LSC_CONTROL_Pin, GPIO_PIN_SET);
        state_step = 1;
        printf("THRUSTER STARTUP_LSC: LSI check passed, energized LSC_CONTROL, monitoring LSI pin...\r\n");
        break;
      }
      if (lsc_closed)
      {
        enter_state(THRUSTER_STATE_STARTUP_WAIT, "THRUSTER STARTUP_LSC: LSI pin detected LOW, contactor closed");
        break;
      }
      if ((now - state_entry_ms) >= CONTACTOR_CLOSE_TIMEOUT_MS)
      {
        fault(FAULT_LSC_TIMEOUT, "THRUSTER STARTUP_LSC: LSI pin timeout - contactor failed to close");
      }
      break;
    }

    case THRUSTER_STATE_STARTUP_WAIT:
    {
      if ((now - state_entry_ms) >= STARTUP_WAIT_TIME_MS)
      {
        enter_state(THRUSTER_STATE_STARTUP_HSC, "THRUSTER STARTUP_WAIT: done, transitioning to STARTUP_HSC");
      }
      break;
    }

    case THRUSTER_STATE_STARTUP_HSC:
    {
      if (state_step == 0)
      {
        if (hsc_closed)
        {
          fault(FAULT_HSC_ALREADY_CLOSED, "THRUSTER STARTUP_HSC: HSI pin is not SET before starting, contactor may already be closed");
          break;
        }
        HAL_GPIO_WritePin(HSC_CONTROL_GPIO_Port, HSC_CONTROL_Pin, GPIO_PIN_SET);
        state_step = 1;
        printf("THRUSTER STARTUP_HSC: HSI check passed, energized HSC_CONTROL, monitoring HSI pin...\r\n");
        break;
      }
      if (state_step == 1)
      {
        if (hsc_closed)
        {
          state_step = 2;
          hsc_confirm_ms = now;
          printf("THRUSTER STARTUP_HSC: HSI pin detected LOW, contactor closed\r\n");
        }
        else if ((now - state_entry_ms) >= CONTACTOR_CLOSE_TIMEOUT_MS)
        {
          fault(FAULT_HSC_TIMEOUT, "THRUSTER STARTUP_HSC: HSI pin timeout - contactor failed to close");
        }
        break;
      }
      /* state_step == 2: contactor confirmed, check the bus after a short delay */
      if ((now - hsc_confirm_ms) >= STARTUP_HSC_VOLTAGE_CHECK_MS)
      {
        if (bus_voltage >= STARTUP_MIN_BUS_VOLTAGE && both_neutral)
        {
          char msg[96];
          snprintf(msg, sizeof(msg), "THRUSTER STARTUP_HSC: Bus voltage %.2fV >= %.1fV, transitioning to RUN",
                   bus_voltage, STARTUP_MIN_BUS_VOLTAGE);
          enter_state(THRUSTER_STATE_RUN, msg);
          last_cmd_ms = 0;
        }
        else if (bus_voltage >= STARTUP_MIN_BUS_VOLTAGE)
        {
          /* not neutral: arming_update() disarms and aborts on the next loop */
        }
        else
        {
          char msg[96];
          snprintf(msg, sizeof(msg), "THRUSTER STARTUP_HSC: Bus voltage %.2fV < %.1fV after %ums",
                   bus_voltage, STARTUP_MIN_BUS_VOLTAGE, (unsigned)STARTUP_HSC_VOLTAGE_CHECK_MS);
          fault(FAULT_BUS_LOW_AT_START, msg);
        }
      }
      break;
    }

    case THRUSTER_STATE_RUN:
    {
      if (!armed)
      {
        fault(FAULT_PWM_LOST, "THRUSTER RUN: not armed");
        break;
      }
      if (bus_voltage < RUN_MIN_BUS_VOLTAGE)
      {
        char msg[96];
        snprintf(msg, sizeof(msg), "THRUSTER RUN: Bus voltage %.2fV < %.1fV", bus_voltage, RUN_MIN_BUS_VOLTAGE);
        fault(FAULT_BUS_LOW_IN_RUN, msg);
        break;
      }
      if (!lsc_closed || !hsc_closed)
      {
        if (contactor_open_since_ms == 0)
        {
          contactor_open_since_ms = now ? now : 1;
        }
        else if ((now - contactor_open_since_ms) >= RUN_CONTACTOR_OPEN_DEBOUNCE_MS)
        {
          char msg[96];
          snprintf(msg, sizeof(msg), "THRUSTER RUN: contactor opened (LSC %s, HSC %s): E-stop or TSMS?",
                   lsc_closed ? "closed" : "OPEN", hsc_closed ? "closed" : "OPEN");
          fault(FAULT_CONTACTOR_OPENED_IN_RUN, msg);
          break;
        }
      }
      else
      {
        contactor_open_since_ms = 0;
      }

      if ((now - last_cmd_ms) >= THRUSTER_CMD_PERIOD_MS)
      {
        last_cmd_ms = now;
        shape_command(&chan[0], now);
        shape_command(&chan[1], now);
        /* ch1 (PA3) -> right thruster (address 2), ch2 (PA2) -> left thruster (address 1) */
        eprop.sendThrottleCommand(chan[0].sent_permille, chan[1].sent_permille);
      }
      break;
    }

    case THRUSTER_STATE_FAULT:
    default:
    {
      open_contactors();
#if FAULT_RECOVER_HOLD_MS > 0
      bool allowed = (fault_code == FAULT_PWM_LOST) || (fault_code == FAULT_STOP_COMMAND) ||
                     (interlock_fault_count <= INTERLOCK_FAULT_MAX_RETRIES);
#if CONTACTOR_OPEN_FAULT_LATCHED
      if (fault_code == FAULT_CONTACTOR_OPENED_IN_RUN) allowed = false;
#endif
      if (stop_latched) allowed = false;
      if (!allowed)
      {
        if (!fault_latched_notice)
        {
          fault_latched_notice = true;
          if (fault_code == FAULT_CONTACTOR_OPENED_IN_RUN)
            printf("FAULT latched: contactor opened in RUN (E-stop/TSMS); reset the VCB\r\n");
          else if (stop_latched)
            printf("FAULT latched: stop command; send 'clear' or reset the VCB\r\n");
          else
            printf("FAULT latched: %lu interlock faults this boot (max %u retries); reset the VCB\r\n",
                   (unsigned long)interlock_fault_count, (unsigned)INTERLOCK_FAULT_MAX_RETRIES);
        }
        break;
      }
      if (both_valid && both_neutral)
      {
        if (recover_hold_start_ms == 0)
        {
          recover_hold_start_ms = now ? now : 1;
        }
        else if ((now - recover_hold_start_ms) >= FAULT_RECOVER_HOLD_MS)
        {
          recover_hold_start_ms = 0;
          fault_code = FAULT_NONE;
          enter_state(THRUSTER_STATE_IDLE, "FAULT cleared: PWM valid and neutral, back to IDLE (arming will restart)");
        }
      }
      else
      {
        recover_hold_start_ms = 0;
      }
#else
      (void)both_valid; (void)both_neutral;
#endif
      break;
    }
  }
}

/* ---- LEDs (CTL3 = red, CTL2 = orange, CTL1 = green, as in cttdev/VCB) ---- */
static void update_leds(uint32_t now)
{
  bool fast = ((now / 100) & 1) != 0;   /* 100 ms on / 100 ms off */
  bool slow = ((now / 500) & 1) != 0;   /* 500 ms on / 500 ms off */

  if (thruster_state == THRUSTER_STATE_FAULT)
  {
    HAL_GPIO_WritePin(CTL3_GPIO_Port, CTL3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CTL2_GPIO_Port, CTL2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(CTL3_GPIO_Port, CTL3_Pin, GPIO_PIN_RESET);
    GPIO_PinState orange = GPIO_PIN_RESET;
    switch (thruster_state)
    {
      case THRUSTER_STATE_IDLE:         orange = GPIO_PIN_SET; break;
      case THRUSTER_STATE_STARTUP_LSC:
      case THRUSTER_STATE_STARTUP_WAIT:
      case THRUSTER_STATE_STARTUP_HSC:  orange = fast ? GPIO_PIN_SET : GPIO_PIN_RESET; break;
      case THRUSTER_STATE_RUN:          orange = slow ? GPIO_PIN_SET : GPIO_PIN_RESET; break;
      default: break;
    }
    HAL_GPIO_WritePin(CTL2_GPIO_Port, CTL2_Pin, orange);
  }
  /* green: both PWM inputs valid (solid), armed as well (slow blink) */
  bool both_valid = chan[0].cmd_valid && chan[1].cmd_valid;
  HAL_GPIO_WritePin(CTL1_GPIO_Port, CTL1_Pin,
                    (both_valid && (!armed || slow)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ---- telemetry (CSV on UART4, same shape as the deployed bench build) ---- */
static void print_banner(void)
{
  printf("%s %s; limit %d permille both directions; neutral %d us; deadband %d us; "
         "window %d..%d us; dir-change stop %d ms; console commands: help\r\n",
         FW_NAME, FW_VERSION, THRUST_LIMIT_PERMILLE, PWM_NEUTRAL_US, PWM_DEADBAND_US,
         PWM_MIN_WIDTH_US, PWM_MAX_WIDTH_US, DIRECTION_CHANGE_STOP_MS);
}

/* One header line per call (3 in total), so a burst never stalls the loop
 * for more than one line: _write() waits for the previous DMA transfer. */
static void print_header_line(uint8_t which)
{
  switch (which)
  {
    case 0:
      printf("mapping,ch1=CONT_pin2_PA3,ch2=CONT_pin1_PA2,motors=ch1_right:ch2_left,range_us=1000:2000,"
             "neutral_us=%d,deadband_us=%d,limit_permille=%d,window_us=%d:%d,dir_stop_ms=%d,"
             "cmd_full_scale=%d,fw=%s\r\n",
             PWM_NEUTRAL_US, PWM_DEADBAND_US, THRUST_LIMIT_PERMILLE, PWM_MIN_WIDTH_US, PWM_MAX_WIDTH_US,
             DIRECTION_CHANGE_STOP_MS, EPROP_FULL_SCALE_VALUE, FW_VERSION);
      break;
    case 1:
      printf("pwm,ms,ch1_us,ch1_period_us,ch1_valid,ch1_raw_permille,ch1_applied_permille,ch1_ok,ch1_bad,ch1_ovr,"
             "ch2_us,ch2_period_us,ch2_valid,ch2_raw_permille,ch2_applied_permille,ch2_ok,ch2_bad,ch2_ovr,"
             "ch1_cmd,ch2_cmd\r\n");
      break;
    default:
      printf("status,ms,thruster_state,armed,estop_24v_present,tsms_24v_present,lsc_closed,hsc_closed,bus_mv,"
             "fault_code,interlock_faults,cmd_source\r\n");
      break;
  }
}

static void print_headers(void)
{
  headers_pending = 3;
}

/* Telemetry scheduler: at most one line per loop iteration. */
static void telemetry_update(uint32_t now)
{
  if ((now - last_header_ms) >= HEADER_PERIOD_MS)
  {
    last_header_ms = now;
    print_headers();
  }
  if (headers_pending)
  {
    print_header_line(3 - headers_pending);
    headers_pending--;
    return;
  }
  if ((now - last_pwm_telem_ms) >= PWM_TELEM_PERIOD_MS)
  {
    last_pwm_telem_ms = now;
    print_pwm_line(now);
    return;
  }
  if ((now - last_status_telem_ms) >= STATUS_TELEM_PERIOD_MS)
  {
    last_status_telem_ms = now;
    print_status_line(now);
  }
}

static void print_pwm_line(uint32_t now)
{
  const ChannelCmd* a = &chan[0];
  const ChannelCmd* b = &chan[1];
  /* ch1_cmd / ch2_cmd: the signed value byte last sent to the converter for
   * that channel's motor (ch1 = right = address 2, ch2 = left = address 1). */
  printf("pwm,%lu,%lu,%lu,%u,%ld,%ld,%lu,%lu,%lu,%lu,%lu,%u,%ld,%ld,%lu,%lu,%lu,%d,%d\r\n",
         (unsigned long)now,
         (unsigned long)a->s.width_us, (unsigned long)a->s.period_us, (unsigned)a->s.valid,
         (long)a->raw_permille, (long)a->applied_permille,
         (unsigned long)a->s.ok_count, (unsigned long)a->s.bad_count, (unsigned long)a->s.ovr_count,
         (unsigned long)b->s.width_us, (unsigned long)b->s.period_us, (unsigned)b->s.valid,
         (long)b->raw_permille, (long)b->applied_permille,
         (unsigned long)b->s.ok_count, (unsigned long)b->s.bad_count, (unsigned long)b->s.ovr_count,
         (int)eprop.lastCommand(EProp::RIGHT), (int)eprop.lastCommand(EProp::LEFT));
}

static void print_status_line(uint32_t now)
{
  /* cmd_source: 0 = PWM inputs, 1 = bench (console) */
  printf("status,%lu,%u,%u,%u,%u,%u,%u,%ld,%u,%lu,%u\r\n",
         (unsigned long)now, (unsigned)thruster_state, (unsigned)armed,
         (unsigned)estop_present, (unsigned)tsms_present, (unsigned)lsc_closed, (unsigned)hsc_closed,
         (long)(bus_voltage * 1000.0f), (unsigned)fault_code, (unsigned long)interlock_fault_count,
         (unsigned)cmd_source);
}

/* ---- console input (UART4 RX): bench mode, stop, clear ------------------ */
static void console_start(void)
{
  console_line.len = 0;
  console_line.overflow = false;
  HAL_UART_Receive_IT(&huart4, &console_rx_byte, 1);
}

static void console_rx_isr(uint8_t b)
{
  uint8_t next = (uint8_t)((console_ring_head + 1) & (CONSOLE_RX_RING_SIZE - 1));
  if (next == console_ring_tail) { console_rx_dropped++; return; }
  console_ring[console_ring_head] = b;
  console_ring_head = next;
}

static void print_help(void)
{
  printf("CONSOLE: bench on | bench off | bench R L (permille, +-%d max, repeat within %d ms) | stop | clear | help\r\n",
         BENCH_LIMIT_PERMILLE, BENCH_TIMEOUT_MS);
}

static void handle_console_cmd(const ConsoleCmd* cmd, uint32_t now)
{
  switch (cmd->kind)
  {
    case CMD_BENCH_ON:
      if (cmd_source == SOURCE_BENCH) { printf("BENCH: already on\r\n"); break; }
      if (armed || (thruster_state != THRUSTER_STATE_IDLE && thruster_state != THRUSTER_STATE_FAULT))
      {
        printf("BENCH: refused, board is armed or running; 'bench on' only from IDLE/FAULT while disarmed\r\n");
        break;
      }
      cmd_source = SOURCE_BENCH;
      bench_have_cmd = false;
      bench_cmd[0] = bench_cmd[1] = 0;
      arm_hold_start_ms = 0;
      printf("BENCH: on; PWM inputs ignored; send 'bench R L' (permille, +-%d max) at least every %d ms; "
             "'bench 0 0' for %d ms arms; 'bench off' returns to PWM\r\n",
             BENCH_LIMIT_PERMILLE, BENCH_TIMEOUT_MS, ARM_HOLD_MS);
      break;

    case CMD_BENCH_OFF:
      if (cmd_source != SOURCE_BENCH) { printf("BENCH: already off\r\n"); break; }
      if (armed) disarm_to_idle("bench off");
      cmd_source = SOURCE_PWM;
      bench_have_cmd = false;
      bench_cmd[0] = bench_cmd[1] = 0;
      arm_hold_start_ms = 0;
      printf("BENCH: off; PWM inputs are the command source again\r\n");
      break;

    case CMD_BENCH_SET:
    {
      if (cmd_source != SOURCE_BENCH) { printf("BENCH: ignored, send 'bench on' first\r\n"); break; }
      int32_t r = clamp_i32(cmd->right_permille, -BENCH_LIMIT_PERMILLE, BENCH_LIMIT_PERMILLE);
      int32_t l = clamp_i32(cmd->left_permille,  -BENCH_LIMIT_PERMILLE, BENCH_LIMIT_PERMILLE);
      if (r != cmd->right_permille || l != cmd->left_permille)
        printf("BENCH: clamped to %ld %ld (limit +-%d)\r\n", (long)r, (long)l, BENCH_LIMIT_PERMILLE);
      bench_cmd[0] = r;
      bench_cmd[1] = l;
      bench_last_ms = now;
      bench_have_cmd = true;
      break;
    }

    case CMD_STOP:
      if (thruster_state == THRUSTER_STATE_FAULT && stop_latched) { printf("STOP: already stopped and latched\r\n"); break; }
      fault(FAULT_STOP_COMMAND, "stop command from the console");
      break;

    case CMD_CLEAR:
      if (!stop_latched) { printf("CLEAR: nothing latched by 'stop'\r\n"); break; }
      stop_latched = false;
      printf("CLEAR: stop un-latched; recovery after %d ms of valid neutral command\r\n", FAULT_RECOVER_HOLD_MS);
      break;

    case CMD_HELP:
      print_help();
      break;

    case CMD_UNKNOWN:
      printf("CONSOLE: unknown command '%s' (try 'help')\r\n", cmd->line);
      break;

    default:
      break;
  }
}

/* Drain the RX ring (at most a few commands per loop) and, as a safety net,
 * restart reception if a UART error stopped it. */
static void console_update(uint32_t now)
{
  if (huart4.RxState == HAL_UART_STATE_READY) HAL_UART_Receive_IT(&huart4, &console_rx_byte, 1);

  ConsoleCmd cmd;
  uint8_t handled = 0;
  while (console_ring_tail != console_ring_head && handled < 4)
  {
    uint8_t b = console_ring[console_ring_tail];
    console_ring_tail = (uint8_t)((console_ring_tail + 1) & (CONSOLE_RX_RING_SIZE - 1));
    if (Console_Feed(&console_line, (char)b, &cmd))
    {
      handle_console_cmd(&cmd, now);
      handled++;
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART9_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  /* Contactors open before anything else runs. */
  open_contactors();

  /* RGB LED PWM (TIM3), kept from cttdev/VCB; the discrete CTL LEDs carry the status. */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  memset(dma_tx_buffer, 0, sizeof(dma_tx_buffer));

  /* Bus voltage ADC, continuous via DMA */
  adc_value = 0;
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_value, 1);

  lars_mcb.init(&huart7);
  reel_mcb.init(&huart3);
  eprop.init(&huart5);

  /* PWM inputs: ch1 on TIM5 (PA3), ch2 on TIM2 (PA2) */
  PwmIn_Start(&htim5, &htim2);

  /* Console RX (bench mode, stop) */
  console_start();

  print_banner();
  print_header_line(0);
  print_header_line(1);
  print_header_line(2);
  enter_state(THRUSTER_STATE_IDLE, "THRUSTER IDLE: boot");
  eprop.sendStop();

  uint32_t now = HAL_GetTick();
  last_header_ms = now;
  last_pwm_telem_ms = last_status_telem_ms = now;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    now = HAL_GetTick();

    console_update(now);   /* may switch the command source, disarm, or raise a fault */
    read_inputs(now);
    bool both_valid   = chan[0].cmd_valid && chan[1].cmd_valid;
    bool both_neutral = chan[0].neutral && chan[1].neutral;

    arming_update(now, both_valid, both_neutral);
    thruster_update(now, both_valid, both_neutral);

    /* Not in RUN: keep telling the converter to stop, at a low rate. */
    if (thruster_state != THRUSTER_STATE_RUN)
    {
      reset_command_shaping();
      if ((now - last_stop_ms) >= STOP_CMD_PERIOD_MS)
      {
        last_stop_ms = now;
        eprop.sendStop();
      }
    }

    /* The two MCBs (LARS, REEL) are not driven by this build: keep them idle. */
    if ((now - last_mcb_ms) >= MCB_IDLE_PERIOD_MS)
    {
      last_mcb_ms = now;
      lars_mcb.sendIdle();
      reel_mcb.sendIdle();
    }

    update_leds(now);
    telemetry_update(now);

    HAL_Delay(MAIN_LOOP_PERIOD_MS);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 5;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_UART9
                              |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 25;
  PeriphClkInitStruct.PLL2.PLL2P = 10;
  PeriphClkInitStruct.PLL2.PLL2Q = 10;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16910CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 / TIM5 Initialization: servo-PWM input capture at 1 MHz.
  *   TIM2: PA2 (TI3) -> IC3 rising (direct), IC4 falling (indirect)   = PWM ch2, left
  *   TIM5: PA3 (TI4) -> IC4 rising (direct), IC3 falling (indirect)   = PWM ch1, right
  * Both are 32-bit timers, free running (period 0xFFFFFFFF), so the
  * unsigned differences in pwm_in.c are correct across wrap.
  */
static uint32_t PwmTimer_Prescaler(void)
{
  /* APB1 timer kernel clock = PCLK1 when the APB1 prescaler is 1, else 2 x PCLK1 (TIMPRE = 0). */
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t timclk = ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) == RCC_D2CFGR_D2PPRE1_DIV1) ? pclk1 : 2U * pclk1;
  return (timclk / 1000000U) - 1U;   /* 128 MHz -> 127 -> 1 us per tick */
}

static void MX_TIM_IC_Common(TIM_HandleTypeDef* htim, TIM_TypeDef* instance,
                             uint32_t rise_channel, uint32_t fall_channel)
{
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim->Instance = instance;
  htim->Init.Prescaler = PwmTimer_Prescaler();
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = 0xFFFFFFFFU;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(htim) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 3;   /* 8 samples at the timer clock: ~60 ns glitch filter */
  if (HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, rise_channel) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, fall_channel) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  MX_TIM_IC_Common(&htim2, TIM2, TIM_CHANNEL_3, TIM_CHANNEL_4);
}

static void MX_TIM5_Init(void)
{
  MX_TIM_IC_Common(&htim5, TIM5, TIM_CHANNEL_4, TIM_CHANNEL_3);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 38400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart5, UART_DE_POLARITY_HIGH, 1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart7, UART_DE_POLARITY_HIGH, 1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART9_Init(void)
{

  /* USER CODE BEGIN UART9_Init 0 */

  /* USER CODE END UART9_Init 0 */

  /* USER CODE BEGIN UART9_Init 1 */

  /* USER CODE END UART9_Init 1 */
  huart9.Instance = UART9;
  huart9.Init.BaudRate = 115200;
  huart9.Init.WordLength = UART_WORDLENGTH_8B;
  huart9.Init.StopBits = UART_STOPBITS_1;
  huart9.Init.Parity = UART_PARITY_NONE;
  huart9.Init.Mode = UART_MODE_TX_RX;
  huart9.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart9.Init.OverSampling = UART_OVERSAMPLING_16;
  huart9.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart9.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart9.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart9, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart9, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart9, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart9) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART9_Init 2 */

  /* USER CODE END UART9_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENSE_N_GPIO_Port, SENSE_N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CTL4_Pin|CTL3_Pin|CTL2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CTL1_Pin|LSC_CONTROL_Pin|HSC_CONTROL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SENSE_N_Pin */
  GPIO_InitStruct.Pin = SENSE_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSE_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDN_ESTOP_OUT_SENSE_Pin SDN_TSMS_OUT_SENSE_Pin */
  GPIO_InitStruct.Pin = SDN_ESTOP_OUT_SENSE_Pin|SDN_TSMS_OUT_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CTL4_Pin CTL3_Pin CTL2_Pin */
  GPIO_InitStruct.Pin = CTL4_Pin|CTL3_Pin|CTL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CTL1_Pin LSC_CONTROL_Pin HSC_CONTROL_Pin */
  GPIO_InitStruct.Pin = CTL1_Pin|LSC_CONTROL_Pin|HSC_CONTROL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : HSI_3_3_Pin LSI_3_3_Pin */
  GPIO_InitStruct.Pin = HSI_3_3_Pin|LSI_3_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

extern "C" {
  /**
    * @brief printf -> UART4 via DMA (cttdev/VCB). Lines longer than
    *        CONSOLE_TX_BUF_SIZE are truncated; waits up to 100 ms for the
    *        previous transfer.
    */
  int _write(int file, char *ptr, int len) {
    (void)file;
    uint32_t timeout = HAL_GetTick() + 100;
    while (huart4.gState != HAL_UART_STATE_READY) {
      if (HAL_GetTick() > timeout) {
        return 0;
      }
    }
    int copy_len = (len < CONSOLE_TX_BUF_SIZE) ? len : CONSOLE_TX_BUF_SIZE;
    memcpy(dma_tx_buffer, ptr, copy_len);
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart4, (uint8_t*)dma_tx_buffer, copy_len);
    return (status == HAL_OK) ? copy_len : 0;
  }

  /** TIM2 / TIM5 capture interrupts -> PWM measurement. */
  void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    PwmIn_CaptureCallback(htim);
  }

  /** UART4 RX: one byte into the console ring, re-arm. */
  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4) {
      console_rx_isr(console_rx_byte);
      HAL_UART_Receive_IT(&huart4, &console_rx_byte, 1);
    }
  }

  /** UART errors (overrun, framing, noise). For a blocking error the HAL
   *  ends the reception; console_update() restarts it on the next loop. */
  void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    (void)huart;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Contactors open, then stop. */
  HAL_GPIO_WritePin(LSC_CONTROL_GPIO_Port, LSC_CONTROL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HSC_CONTROL_GPIO_Port, HSC_CONTROL_Pin, GPIO_PIN_RESET);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
