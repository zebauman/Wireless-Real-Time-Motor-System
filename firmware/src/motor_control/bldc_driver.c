#include "bldc_driver.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <soc.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bldc_driver, LOG_LEVEL_INF);

/* ========================================================================= *
 * HARDWARE CONSTANTS                                                        *
 * ========================================================================= */

// STM32WB55 @ 64MHz, APB2 prescaler=1 → TIM1 = 64MHz
#define TIM1_ARR            3200        // 64MHz / 3200 = 20kHz PWM
#define DEADTIME_TICKS      50          // 50/64MHz = 781ns

/* ── Pole pairs ────────────────────────────────────────────────────────────
 * Motor spec says "8 poles" = 4 pole pairs (N+S = 1 pair).
 * edges_per_rev = POLE_PAIRS * 6 = 24
 * Confirmed: partner's RPM_CONSTANT_FILTERED=15,000,000 at 1MHz TIM2
 * corresponds to 24 edges/rev (4 pole pairs).                             */
#define POLE_PAIRS          4
#define EDGES_PER_REV       (POLE_PAIRS * 6)    // = 24

/* ── TIM2 RPM timer ────────────────────────────────────────────────────────
 * TIM2 free-running at 1MHz (prescaler = 64-1 = 63 for 64MHz APB1).
 * Same approach as partner's working code — cleaner than CPU cycle counter.
 * dt measured in microseconds directly.
 *
 * RPM calculation with 6-sample circular buffer (smoothing):
 *   sum_6 = sum of 6 consecutive inter-edge times in µs
 *   RPM = (60 * 1,000,000 * 6) / (sum_6 * EDGES_PER_REV)
 *       = 360,000,000 / (sum_6 * 24)
 *       = 15,000,000 / sum_6
 *
 * Single-edge (instantaneous) formula:
 *   RPM = (60 * 1,000,000) / (dt_us * EDGES_PER_REV)
 *       = 60,000,000 / (dt_us * 24)
 *       = 2,500,000 / dt_us                                              */
#define TIM2_PRESCALER      63          // 64MHz / (63+1) = 1MHz
#define RPM_CONSTANT        2500000UL   // single-edge: 60e6 / EDGES_PER_REV
#define RPM_CONSTANT_FILT   15000000UL  // 6-sample buffer: RPM_CONSTANT * 6
#define RPM_HISTORY_SIZE    6
#define RPM_TIMEOUT_US      2000000UL   // 2 seconds → rpm = 0 (stopped)

/* ── Debounce ──────────────────────────────────────────────────────────────
 * At 3000 RPM with 24 edges/rev: edge every 833µs → use 50µs debounce.
 * 5000µs for bench/hand testing.                                          */
#define HALL_DEBOUNCE_US    50

/* ── Duty cycle constants ───────────────────────────────────────────────── */
#define BOOTSTRAP_DUTY      ((TIM1_ARR * 95) / 100)   // 3040 counts
#define SOFTSTART_DUTY      ((TIM1_ARR * 10) / 100)   //  320 counts — 10%
#define SOFTSTART_STEP      ((TIM1_ARR *  1) / 100)   //   32 counts/edge — 1%
#define SOFTSTART_END_PCT   15.0f                       // PID takes over above 15%

/* ========================================================================= *
 * COMMUTATION LOOKUP TABLES                                                 *
 * ========================================================================= *
 * Confirmed by observation: direct mapping (case==state) = CCW.
 * CCW sequence: 6→4→5→1→3→2→(repeat)
 *
 * Pin → TIM1 channel:
 *   PA8 /CH1  = U+   PB13/CH1N = U-
 *   PA9 /CH2  = V+   PB14/CH2N = V-
 *   PA10/CH3  = W+   PB15/CH3N = W-
 *
 * CW = swap + and - on every pair (reverse current direction):
 *   0x1→case6  0x2→case5  0x3→case4
 *   0x4→case3  0x5→case2  0x6→case1                                      */
static const uint8_t cw_commutation[8]  = { 0, 6, 5, 4, 3, 2, 1, 0 };
static const uint8_t ccw_commutation[8] = { 0, 1, 2, 3, 4, 5, 6, 0 };

/* ========================================================================= *
 * GPIO DEFINITIONS                                                          *
 * ========================================================================= */
static const struct gpio_dt_spec hall_u = GPIO_DT_SPEC_GET(DT_ALIAS(hall_u), gpios);
static const struct gpio_dt_spec hall_v = GPIO_DT_SPEC_GET(DT_ALIAS(hall_v), gpios);
static const struct gpio_dt_spec hall_w = GPIO_DT_SPEC_GET(DT_ALIAS(hall_w), gpios);

/* ========================================================================= *
 * ISR / RUNTIME STATE                                                       *
 * ========================================================================= */
static struct gpio_callback hall_u_cb;
static struct gpio_callback hall_v_cb;
static struct gpio_callback hall_w_cb;

atomic_t g_motor_speed_atomic = ATOMIC_INIT(0);

/* ── TIM2-based RPM measurement ─────────────────────────────────────────── */
static volatile uint32_t rpm_history[RPM_HISTORY_SIZE];
static volatile uint8_t  rpm_idx         = 0;
static volatile uint32_t rpm_prev_ticks  = 0;
static volatile uint32_t rpm_last_edge   = 0;  // TIM2 tick of last valid edge

/* ── Motor control state ─────────────────────────────────────────────────── */
static volatile int  current_direction_ccw = 0;
static volatile bool motor_running         = false;
static volatile int  softstart_pulse       = SOFTSTART_DUTY;

static void hall_isr_callback(const struct device *dev,
                               struct gpio_callback *cb, uint32_t pins);

/* ========================================================================= *
 * TIM2 INIT — free-running 1MHz counter                                   *
 * ========================================================================= */
static void tim2_init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    LL_TIM_SetPrescaler(TIM2, TIM2_PRESCALER);  // 64MHz / 64 = 1MHz
    LL_TIM_SetAutoReload(TIM2, 0xFFFFFFFF);     // 32-bit free-run
    LL_TIM_GenerateEvent_UPDATE(TIM2);          // latch prescaler
    LL_TIM_EnableCounter(TIM2);

    LOG_INF("TIM2 running at 1MHz for RPM measurement");
}

/* ========================================================================= *
 * INITIALIZATION                                                            *
 * ========================================================================= */
int bldc_driver_init(void)
{
    LOG_INF("Initializing BLDC driver...");

    if (!gpio_is_ready_dt(&hall_u) ||
        !gpio_is_ready_dt(&hall_v) ||
        !gpio_is_ready_dt(&hall_w)) {
        LOG_ERR("Hall GPIOs not ready");
        return -ENODEV;
    }

    gpio_pin_configure_dt(&hall_u, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&hall_v, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&hall_w, GPIO_INPUT | GPIO_PULL_UP);

    int boot_state = bldc_read_hall_state();
    LOG_INF("Boot hall state: 0x%X  %s", boot_state,
            (boot_state == 0 || boot_state == 7)
            ? "*** INVALID — rotate shaft slightly ***" : "OK");

    /* ── TIM2 for RPM ───────────────────────────────────────────────────── */
    tim2_init();
    rpm_prev_ticks = TIM2->CNT;
    rpm_last_edge  = TIM2->CNT;
    for (int i = 0; i < RPM_HISTORY_SIZE; i++) {
        rpm_history[i] = 0;
    }

    /* ── TIM1 PWM ───────────────────────────────────────────────────────── */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_TIM_SetPrescaler(TIM1, 0);
    LL_TIM_SetAutoReload(TIM1, TIM1_ARR - 1);
    LL_TIM_EnableARRPreload(TIM1);

    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);

    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

    LL_TIM_SetOffStates(TIM1, LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);
    LL_TIM_OC_SetDeadTime(TIM1, DEADTIME_TICKS);

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_GenerateEvent_UPDATE(TIM1);

    bldc_set_bootstrap();

    /* ── Hall interrupts ────────────────────────────────────────────────── */
    gpio_pin_interrupt_configure_dt(&hall_u, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_v, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_w, GPIO_INT_EDGE_BOTH);

    gpio_init_callback(&hall_u_cb, hall_isr_callback, BIT(hall_u.pin));
    gpio_init_callback(&hall_v_cb, hall_isr_callback, BIT(hall_v.pin));
    gpio_init_callback(&hall_w_cb, hall_isr_callback, BIT(hall_w.pin));

    gpio_add_callback_dt(&hall_u, &hall_u_cb);
    gpio_add_callback_dt(&hall_v, &hall_v_cb);
    gpio_add_callback_dt(&hall_w, &hall_w_cb);

    LOG_INF("BLDC ready — 20kHz PWM  781ns dead-time  4PP  TIM2@1MHz");
    return 0;
}

/* ========================================================================= *
 * BOOTSTRAP STATE                                                           *
 * ========================================================================= */
void bldc_set_bootstrap(void)
{
    motor_running   = false;
    softstart_pulse = SOFTSTART_DUTY;

    unsigned int key = irq_lock();
    TIM1->CCER &= ~(TIM_CCER_CC1E  | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E  | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E  | TIM_CCER_CC3NE);
    TIM1->CCR1 = BOOTSTRAP_DUTY;
    TIM1->CCR2 = BOOTSTRAP_DUTY;
    TIM1->CCR3 = BOOTSTRAP_DUTY;
    TIM1->CCER |= TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    irq_unlock(key);

    atomic_set(&g_motor_speed_atomic, 0);
    LOG_INF("Bootstrap: low-sides active, high-sides off");
}

/* ========================================================================= *
 * MOTOR START                                                               *
 * ========================================================================= */
void bldc_set_running(void)
{
    softstart_pulse = SOFTSTART_DUTY;
    motor_running   = true;

    // Seed TIM2 timestamp so hall_age doesn't false-timeout immediately
    rpm_last_edge = TIM2->CNT;

    uint8_t state = (uint8_t)bldc_read_hall_state();
    if (state != 0 && state != 7) {
        bldc_set_commutation_with_duty(state, softstart_pulse);
        LOG_INF("Motor start: hall=0x%X duty=%d", state, softstart_pulse);
    }
}

/* ========================================================================= *
 * HALL SENSOR ISR                                                           *
 * ========================================================================= */
static void hall_isr_callback(const struct device *dev,
                               struct gpio_callback *cb, uint32_t pins)
{
    uint32_t now_us = TIM2->CNT;
    uint32_t dt_us  = now_us - rpm_prev_ticks;  // wraps correctly (uint32)

    if (dt_us < HALL_DEBOUNCE_US) return;

    rpm_prev_ticks = now_us;
    rpm_last_edge  = now_us;

    uint8_t raw_step = (uint8_t)bldc_read_hall_state();
    if (raw_step == 0 || raw_step == 7) return;

    if (!motor_running) {
        atomic_set(&g_motor_speed_atomic, 0);
        return;
    }

    /* ── Softstart ramp ─────────────────────────────────────────────────── */
    if (softstart_pulse < bldc_percent_to_pulse(SOFTSTART_END_PCT)) {
        softstart_pulse += SOFTSTART_STEP;
    }

    /* ── Commutation ────────────────────────────────────────────────────── */
    bldc_set_commutation_with_duty(raw_step, softstart_pulse);

    /* ── RPM via TIM2 circular buffer ───────────────────────────────────── *
     * Store this inter-edge time in the buffer. Average over 6 edges
     * gives stable reading without lag. Matches partner's proven approach.*/
    rpm_history[rpm_idx] = dt_us;
    rpm_idx = (rpm_idx + 1) % RPM_HISTORY_SIZE;

    uint32_t sum = 0;
    for (int i = 0; i < RPM_HISTORY_SIZE; i++) {
        sum += rpm_history[i];
    }

    int32_t mech_rpm = 0;
    if (sum > 0) {
        // RPM = 15,000,000 / sum_6  (derived: 60e6 * 6 / (24 * sum_6))
        mech_rpm = (int32_t)(RPM_CONSTANT_FILT / sum);
    }

    atomic_set(&g_motor_speed_atomic,
               (atomic_val_t)(current_direction_ccw ? -mech_rpm : mech_rpm));
}

/* ========================================================================= *
 * RPM TIMEOUT CHECK — call from motor_control.c instead of cycle count    *
 * ========================================================================= */
bool bldc_is_rpm_timed_out(void)
{
    uint32_t now = TIM2->CNT;
    uint32_t age = now - rpm_last_edge;
    return (age > RPM_TIMEOUT_US);
}

/* ========================================================================= *
 * SENSOR READ                                                               *
 * ========================================================================= */
int bldc_read_hall_state(void)
{
    int u = !gpio_pin_get_dt(&hall_u);
    int v = !gpio_pin_get_dt(&hall_v);
    int w = !gpio_pin_get_dt(&hall_w);
    return (u << 2) | (v << 1) | w;
}

/* ========================================================================= *
 * COMMUTATION WITH PER-STEP DUTY                                           *
 * ========================================================================= *
 * High-side CCR = pulse  → PWMs at requested duty
 * Low-side  CCR = 0      → complementary ON full cycle (solid return path)
 * IRQ lock protects the CCER read-modify-write from ISR preemption.       */
void bldc_set_commutation_with_duty(uint8_t hall_state, int pulse)
{
    if (pulse > TIM1_ARR) pulse = TIM1_ARR;
    if (pulse < 0)        pulse = 0;

    uint8_t comm = current_direction_ccw
                   ? ccw_commutation[hall_state]
                   : cw_commutation[hall_state];

    unsigned int key = irq_lock();

    TIM1->CCER &= ~(TIM_CCER_CC1E  | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E  | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E  | TIM_CCER_CC3NE);

    switch (comm) {
        case 1: // W+ V-
            TIM1->CCR3 = (uint32_t)pulse;  TIM1->CCR2 = 0;
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC2NE; break;
        case 2: // V+ U-
            TIM1->CCR2 = (uint32_t)pulse;  TIM1->CCR1 = 0;
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1NE; break;
        case 3: // W+ U-
            TIM1->CCR3 = (uint32_t)pulse;  TIM1->CCR1 = 0;
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC1NE; break;
        case 4: // U+ W-
            TIM1->CCR1 = (uint32_t)pulse;  TIM1->CCR3 = 0;
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3NE; break;
        case 5: // U+ V-
            TIM1->CCR1 = (uint32_t)pulse;  TIM1->CCR2 = 0;
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2NE; break;
        case 6: // V+ W-
            TIM1->CCR2 = (uint32_t)pulse;  TIM1->CCR3 = 0;
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3NE; break;
        default:
            irq_unlock(key);
            LOG_ERR("bldc: invalid comm %u for hall 0x%X", comm, hall_state);
            return;
    }

    LL_TIM_GenerateEvent_UPDATE(TIM1);
    irq_unlock(key);
}

/* ========================================================================= *
 * PWM OUTPUT — called by PID thread                                        *
 * ========================================================================= */
void bldc_set_pwm(int pulse)
{
    if (!motor_running) return;
    if (softstart_pulse < bldc_percent_to_pulse(SOFTSTART_END_PCT)) return;

    uint8_t state = (uint8_t)bldc_read_hall_state();
    if (state != 0 && state != 7) {
        bldc_set_commutation_with_duty(state, pulse);
    }

    if (pulse > softstart_pulse) {
        softstart_pulse = pulse;
    }
}

/* ========================================================================= *
 * LEGACY bldc_set_commutation                                              *
 * ========================================================================= */
void bldc_set_commutation(uint8_t step)
{
    bldc_set_commutation_with_duty(bldc_read_hall_state(), softstart_pulse);
    (void)step;
}

/* ========================================================================= *
 * CONVERSION / GETTERS / SETTERS                                            *
 * ========================================================================= */
int bldc_percent_to_pulse(float pct)
{
    int pulse = (int)(pct * 32.0f);
    if (pulse > TIM1_ARR) pulse = TIM1_ARR;
    if (pulse < 0)        pulse = 0;
    return pulse;
}

uint32_t bldc_get_last_cycle_count(void)
{
    // Legacy shim — returns TIM2 tick. Use bldc_get_rpm_age_ms() instead.
    return rpm_last_edge;
}

/** @brief Return milliseconds since last valid hall edge using TIM2.
 *  Correct unit — do NOT use k_cyc_to_ms on the return value of
 *  bldc_get_last_cycle_count() which returns µs ticks not CPU cycles.
 */
uint32_t bldc_get_rpm_age_ms(void)
{
    uint32_t age_us = TIM2->CNT - rpm_last_edge;  // wraps correctly uint32
    return age_us / 1000U;
}

void bldc_set_direction(int ccw)
{
    current_direction_ccw = ccw;
}