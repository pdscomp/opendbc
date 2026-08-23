#pragma once

#include "opendbc/safety/declarations.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_INFO      0x21bU
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_RADAR_STATIC  0x499U
#define MAZDA_RADAR_TRACK_1 0x361U
#define MAZDA_RADAR_TRACK_2 0x362U
#define MAZDA_RADAR_TRACK_3 0x363U
#define MAZDA_RADAR_TRACK_4 0x364U
#define MAZDA_RADAR_TRACK_5 0x365U
#define MAZDA_RADAR_TRACK_6 0x366U
#define MAZDA_RADAR_UDS     0x764U
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U
#define MAZDA_TI_LKAS       0x249U
#define MAZDA_TI_FEEDBACK   0x24aU

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_AUX  1
#define MAZDA_CAM  2

#define MAZDA_PARAM_LONGITUDINAL 1U
#define MAZDA_PARAM_TI           2U
#define MAZDA_TI_FEEDBACK_TIMEOUT_US 40000U  // fail closed after two missed 50 Hz frames

static bool mazda_longitudinal = false;
static bool mazda_ti = false;
static bool mazda_ti_feedback_healthy = false;
static uint32_t mazda_ti_feedback_ts = 0U;
static int mazda_ti_desired_torque_last = 0;
static int mazda_ti_rt_torque_last = 0;
static uint32_t mazda_ti_ts_torque_check_last = 0U;

// With longitudinal control the stock radar is silenced and openpilot replays its frames,
// so allowed tx patterns are pinned to byte-exact stock captures wherever possible.

static bool mazda_radar_static_msg_valid(const CANPacket_t *msg) {
  return (msg->data[0] == 0x00U) && (msg->data[1] == 0x08U) &&
         (msg->data[2] == 0xc0U) && (msg->data[3] == 0x00U) &&
         (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
         (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
}

static bool mazda_empty_radar_track_msg_valid(const CANPacket_t *msg) {
  bool valid = false;

  if ((msg->addr == MAZDA_RADAR_TRACK_1) || (msg->addr == MAZDA_RADAR_TRACK_2) ||
      (msg->addr == MAZDA_RADAR_TRACK_3) || (msg->addr == MAZDA_RADAR_TRACK_4)) {
    valid = (msg->data[0] == 0xffU) && (msg->data[1] == 0xf7U) &&
            (msg->data[2] == 0xfeU) && (msg->data[3] == 0xfeU) &&
            (msg->data[4] == 0x1fU);

    if (msg->addr == MAZDA_RADAR_TRACK_2) {
      valid = valid && (msg->data[5] == 0xc7U) && (msg->data[6] == 0x8cU) &&
              ((msg->data[7] & 0xf0U) == 0x80U);
    } else if ((msg->addr == MAZDA_RADAR_TRACK_3) || (msg->addr == MAZDA_RADAR_TRACK_4)) {
      valid = valid && (msg->data[5] == 0xc0U) && (msg->data[6] == 0x00U) &&
              ((msg->data[7] & 0xf0U) == 0x00U);
    } else {
      valid = valid && (msg->data[5] == 0xc0U) && (msg->data[6] == 0x00U) &&
              ((msg->data[7] & 0xf0U) == 0x80U);
    }
  } else if ((msg->addr == MAZDA_RADAR_TRACK_5) || (msg->addr == MAZDA_RADAR_TRACK_6)) {
    valid = (msg->data[0] == 0xffU) && (msg->data[1] == 0xf7U) &&
            (msg->data[2] == 0xfeU) && (msg->data[3] == 0x7fU) &&
            (msg->data[4] == 0xfbU) && (msg->data[5] == 0xffU) &&
            (msg->data[6] == 0x3fU) && ((msg->data[7] & 0xf0U) == 0xc0U);
  } else {
    valid = false;
  }

  return valid;
}

static bool mazda_synthetic_lead_radar_track_msg_valid(const CANPacket_t *msg) {
  // The controller writes the lead it is following into the occupied-slot capture:
  // DIST_OBJ fills data[0] and the high nibble of data[1], RELV_OBJ fills data[3] and the
  // high 3 bits of data[4]. Those fields are free; every bit the template owns must still
  // match it exactly. A byte-exact check here silently dropped every real-lead frame and
  // starved the camera of the track (route 6bb2dc61c4: 982 asked, 0 transmitted).
  return (msg->addr == MAZDA_RADAR_TRACK_4) &&
         ((msg->data[1] & 0x0fU) == 0x00U) && (msg->data[2] == 0x00U) &&
         ((msg->data[4] & 0x1fU) == 0x1dU) && (msg->data[5] == 0xc0U) &&
         (msg->data[6] == 0x00U) && ((msg->data[7] & 0xf0U) == 0x00U);
}

static bool mazda_radar_track_msg_valid(const CANPacket_t *msg) {
  return mazda_empty_radar_track_msg_valid(msg) ||
         (controls_allowed && mazda_synthetic_lead_radar_track_msg_valid(msg));
}

static bool mazda_ti_feedback_is_fresh(void) {
  return mazda_ti_feedback_healthy &&
         (safety_get_ts_elapsed(microsecond_timer_get(), mazda_ti_feedback_ts) <= MAZDA_TI_FEEDBACK_TIMEOUT_US);
}

static void mazda_ti_reset_torque_state(void) {
  mazda_ti_desired_torque_last = 0;
  mazda_ti_rt_torque_last = 0;
  mazda_ti_ts_torque_check_last = microsecond_timer_get();
}

static bool mazda_ti_torque_cmd_checks(int desired_torque) {
  // Rates mirror the stock STEER_TO_ZERO tune (EPS accepts 12/frame); magnitude is capped at the
  // TI hardware's real ceiling. Non-STZ controllers self-cap lower (6/15/192) under this backstop.
  const TorqueSteeringLimits MAZDA_TI_STEERING_LIMITS = {
    // TI DAC output stage self-protects above ~600 (VIOL=0x11, ~30 s latch) — 2026-08-22 drive data
    .max_torque = 600,
    .max_rate_up = 12,
    .max_rate_down = 25,
    .max_rt_delta = 384,
    .driver_torque_multiplier = 40,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  bool violation = false;
  bool controls = controls_allowed || controls_allowed_lateral;
  uint32_t ts = microsecond_timer_get();
  bool feedback_fresh = mazda_ti_feedback_is_fresh();

  if (controls && feedback_fresh) {
    violation |= safety_max_limit_check(desired_torque, MAZDA_TI_STEERING_LIMITS.max_torque,
                                        -MAZDA_TI_STEERING_LIMITS.max_torque);
    violation |= driver_limit_check(desired_torque, mazda_ti_desired_torque_last, &torque_driver,
                                    MAZDA_TI_STEERING_LIMITS.max_torque,
                                    MAZDA_TI_STEERING_LIMITS.max_rate_up,
                                    MAZDA_TI_STEERING_LIMITS.max_rate_down,
                                    MAZDA_TI_STEERING_LIMITS.driver_torque_allowance,
                                    MAZDA_TI_STEERING_LIMITS.driver_torque_multiplier);
    mazda_ti_desired_torque_last = desired_torque;
    violation |= rt_torque_rate_limit_check(desired_torque, mazda_ti_rt_torque_last,
                                            MAZDA_TI_STEERING_LIMITS.max_rt_delta);
    if (safety_get_ts_elapsed(ts, mazda_ti_ts_torque_check_last) > MAX_RT_INTERVAL) {
      mazda_ti_rt_torque_last = desired_torque;
      mazda_ti_ts_torque_check_last = ts;
    }
  } else {
    violation = desired_torque != 0;
  }

  if (violation || !controls || !feedback_fresh) {
    mazda_ti_reset_torque_state();
  }
  return violation;
}

static bool mazda_get_quality_flag_valid(const CANPacket_t *msg) {
  // Integrity only: right addr/bus, a known version byte, plausible state. The TI
  // routinely drops to OFF with VIOL bits at standstill (steering-current
  // self-protection) and during boot — those are NOT rx faults, and failing the
  // quality flag on them spams safetyRxChecksInvalid ("controls mismatch") at
  // every stop. Health (RUN + zero fault bytes) gates torque separately below.
  bool integrity = (msg->addr == MAZDA_TI_FEEDBACK) && ((int)msg->bus == MAZDA_AUX) &&
                   ((msg->data[2] == 1U) || (msg->data[2] == 0x10U)) && (msg->data[3] <= 3U);
  if (msg->addr == MAZDA_TI_FEEDBACK) {
    mazda_ti_feedback_healthy = integrity && (msg->data[3] == 3U) &&
                                (msg->data[4] == 0U) && (msg->data[5] == 0U) && (msg->data[6] == 0U);
    if (integrity) {
      mazda_ti_feedback_ts = microsecond_timer_get();
    }
  }
  return integrity;
}

static bool mazda_fwd_hook(int bus_num, int addr) {
  return ((bus_num == MAZDA_CAM) && ((addr == (int)MAZDA_LKAS) || (addr == (int)MAZDA_LKAS_HUD) || (addr == (int)MAZDA_TI_LKAS))) ||
         ((bus_num == MAZDA_MAIN) && (addr == (int)MAZDA_TI_LKAS));
}

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if ((int)msg->bus == MAZDA_MAIN) {
    if (msg->addr == MAZDA_ENGINE_DATA) {
      // sample speed: scale by 0.01 to get kph
      int speed = (msg->data[2] << 8) | msg->data[3];
      vehicle_moving = speed > 10; // moving when speed > 0.1 kph
    }

    if ((msg->addr == MAZDA_STEER_TORQUE) && !mazda_ti) {
      int torque_driver_new = msg->data[0] - 127U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if ((msg->addr == MAZDA_CRZ_CTRL) && !mazda_longitudinal) {
      bool cruise_engaged = msg->data[0] & 0x8U;
      pcm_cruise_check(cruise_engaged);
      acc_main_on = GET_BIT(msg, 17U);
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_longitudinal) {
      // ensure the driver's cancel press always exits controls
      bool cancel = GET_BIT(msg, 0U);
      if (cancel) {
        controls_allowed = false;
      }
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
    }

    if (msg->addr == MAZDA_PEDALS) {
      bool brake = (msg->data[0] & 0x10U);
      if (mazda_longitudinal) {
        // The radar teardown removes the stock CRZ_CTRL frame, so derive cruise state from
        // PEDALS: ACC_OFF (bit 2) means MRCC is armed but idle, ACC_ACTIVE (bit 3) means
        // engaged. Brake-only samples can arrive with both bits low mid-press; skip those
        // so they are not mistaken for an ACC-off edge.
        bool cruise_engaged = GET_BIT(msg, 3U);
        bool acc_armed = GET_BIT(msg, 2U) || cruise_engaged;

        if (acc_armed || cruise_engaged_prev || (!brake && !brake_pressed_prev)) {
          acc_main_on = acc_armed;
          pcm_cruise_check(cruise_engaged);
        }
      }
      brake_pressed = brake;
    }
  }

  if (mazda_ti && ((int)msg->bus == MAZDA_AUX) && (msg->addr == MAZDA_TI_FEEDBACK)) {
    update_sample(&torque_driver, (int)msg->data[0] - 127);
  }
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  // Envelope sized for the CX-5 2022+ EPS, which the controller commands up to (max_torque 1200,
  // driver_torque_multiplier 15 vs upstream stock 800/1). SafetyModel.mazda is per-brand and can't
  // see the fingerprint/EPS, so these limits apply to every Mazda. Non-CX-5-EPS Mazdas self-cap
  // lower in the controller (values.py gates the tune on minSteerSpeed == 0), so this is only a
  // looser backstop for them — not a behavior change. Per-car gating would need a safety param.
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 1200,
    .max_rate_up = 12,
    .max_rate_down = 25,
    .max_rt_delta = 384,
    .driver_torque_multiplier = 15,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // CRZ_INFO.ACCEL_CMD is raw units of 0.001 m/s2 (offset removed below), so this is the
  // ISO window: 2.0 / -3.5 m/s2. Stock MRCC itself commands down to raw -3891 in lead stops.
  const LongitudinalLimits MAZDA_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 0,
  };

  bool tx = true;
  bool main_bus = msg->bus == (unsigned char)MAZDA_MAIN;
  bool long_replacement_bus = main_bus || (msg->bus == (unsigned char)MAZDA_CAM);

  // steer cmd checks
  if (main_bus && (msg->addr == MAZDA_LKAS)) {
    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

    if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_STEERING_LIMITS)) {
      tx = false;
    }
  }

  if (((int)msg->bus == MAZDA_AUX) && (msg->addr == MAZDA_TI_LKAS)) {
    int desired_torque_raw = ((msg->data[0] & 0x0FU) << 8) | msg->data[1];
    int duplicate_torque_raw = ((msg->data[2] & 0x0FU) << 8) | msg->data[3];
    int desired_torque = desired_torque_raw - 2048;
    bool valid_key = (msg->data[4] == 0xc4U) && (msg->data[5] == 0x61U) &&
                     (msg->data[6] == 0xceU) && (msg->data[7] == 0x60U);
    bool reserved_bits = ((msg->data[0] & 0xF0U) != 0U) || ((msg->data[2] & 0xF0U) != 0U);
    bool violation = !mazda_ti || reserved_bits || (desired_torque_raw != duplicate_torque_raw) || !valid_key;
    violation |= mazda_ti_torque_cmd_checks(desired_torque);
    if (violation) {
      mazda_ti_reset_torque_state();
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_CRZ_INFO)) {
    // the stock standby pattern pegs the command field high; allow it byte-exactly
    // (checksum included) instead of decoding it as a huge accel command
    bool stock_standby = (msg->data[0] == 0x01U) && (msg->data[1] == 0xffU) &&
                         (msg->data[2] == 0xe3U) && (msg->data[3] == 0xffU) &&
                         (msg->data[4] == 0xc0U) && (msg->data[5] == 0x00U) &&
                         ((msg->data[6] & 0xf0U) == 0x00U) &&
                         (msg->data[7] == ((0x5dU - msg->data[6]) & 0xffU));

    // 13-bit ACCEL_CMD: data[2] low bits, data[3], data[4] high bits, offset 4096
    uint32_t desired_accel_raw = (((uint32_t)msg->data[2] & 0x3U) << 11) |
                                 ((uint32_t)msg->data[3] << 3) |
                                 ((uint32_t)msg->data[4] >> 5);
    int desired_accel = (int)desired_accel_raw - 4096;
    if (!stock_standby && longitudinal_accel_checks(desired_accel, MAZDA_LONG_LIMITS)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_CRZ_CTRL)) {
    bool cruise_active = GET_BIT(msg, 3U);
    if (!controls_allowed && cruise_active) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_RADAR_STATIC)) {
    if (!mazda_radar_static_msg_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr >= MAZDA_RADAR_TRACK_1) && (msg->addr <= MAZDA_RADAR_TRACK_6)) {
    if (!mazda_radar_track_msg_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && main_bus && (msg->addr == MAZDA_RADAR_UDS)) {
    // only tester present and default/programming session control; flashing services stay blocked
    bool tester_present = (msg->data[0] == 0x02U) && (msg->data[1] == 0x3eU) && (msg->data[2] == 0x80U);
    bool session_control = (msg->data[0] == 0x02U) && (msg->data[1] == 0x10U) &&
                           ((msg->data[2] == 0x01U) || (msg->data[2] == 0x02U));
    if (!tester_present && !session_control) {
      tx = false;
    }
  }

  // cruise buttons check
  if (main_bus && (msg->addr == MAZDA_CRZ_BTNS)) {
    // allow resume spamming while controls allowed, but
    // only allow cancel while controls not allowed
    bool cancel_cmd = (msg->data[0] == 0x1U);
    if (!controls_allowed && !cancel_cmd) {
      tx = false;
    }
  }

  return tx;
}

static safety_config mazda_init(uint16_t param) {
  static const CanMsg MAZDA_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
  };

  static const CanMsg MAZDA_TI_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
    {MAZDA_TI_LKAS, MAZDA_AUX, 8, .check_relay = false},
  };

  static const CanMsg MAZDA_LONG_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
    {MAZDA_CRZ_INFO, 0, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, 0, 8, .check_relay = false},
    {MAZDA_RADAR_UDS, 0, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, MAZDA_CAM, 8, .check_relay = false},
  };

  static const CanMsg MAZDA_LONG_TI_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
    {MAZDA_TI_LKAS, MAZDA_AUX, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, 0, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, 0, 8, .check_relay = false},
    {MAZDA_RADAR_UDS, 0, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, MAZDA_CAM, 8, .check_relay = false},
  };

  mazda_longitudinal = GET_FLAG(param, MAZDA_PARAM_LONGITUDINAL);
  mazda_ti = GET_FLAG(param, MAZDA_PARAM_TI);
  mazda_ti_feedback_healthy = false;
  mazda_ti_feedback_ts = 0U;
  mazda_ti_reset_torque_state();
  acc_main_on = false;

  safety_config ret;
  if (mazda_longitudinal) {
    // no CRZ_CTRL check: the stock radar frame disappears after the teardown
    static RxCheck mazda_long_rx_checks[] = {
      {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    };
    static RxCheck mazda_long_ti_rx_checks[] = {
      {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_TI_FEEDBACK,  MAZDA_AUX, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = false}, { 0 }, { 0 }}},
    };
    ret = mazda_ti ? BUILD_SAFETY_CFG(mazda_long_ti_rx_checks, MAZDA_LONG_TI_TX_MSGS) :
                     BUILD_SAFETY_CFG(mazda_long_rx_checks, MAZDA_LONG_TX_MSGS);
  } else {
    static RxCheck mazda_rx_checks[] = {
      {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    };
    static RxCheck mazda_ti_rx_checks[] = {
      {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_TI_FEEDBACK,  MAZDA_AUX, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = false}, { 0 }, { 0 }}},
    };
    ret = mazda_ti ? BUILD_SAFETY_CFG(mazda_ti_rx_checks, MAZDA_TI_TX_MSGS) :
                     BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
  }
  return ret;
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
  .fwd = mazda_fwd_hook,
  .get_quality_flag_valid = mazda_get_quality_flag_valid,
};
