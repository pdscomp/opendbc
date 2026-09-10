#pragma once

#include "opendbc/safety/declarations.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_INFO      0x21bU
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
// Physical TJA button, DBC start bit 11 (byte 1, bit 3). Observed on a CTS-equipped gen1
// Mazda; trims without the button hold it low for the life of a drive.
#define MAZDA_TJA_BUTTON_BIT 11U
// sunnypilot safety param: the TJA button is the MADS lateral switch
#define MAZDA_PARAM_SP_TJA_BUTTON 1U
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
// Select the steer-to-zero EPS envelope from the firmware-derived interface flag.
#define MAZDA_PARAM_STEER_TO_ZERO_EPS 2U
// The same EPS hardware on firmware that keeps the 45 kph floor: the same envelope.
#define MAZDA_PARAM_LEGACY_FW_EPS 4U
// Fork: comma torque interceptor. Bit 8 because the old fork value 2 collided with
// MAZDA_PARAM_STEER_TO_ZERO_EPS. panda's board routing reads this same define.
#define MAZDA_PARAM_TI           8U
#define MAZDA_TI_FEEDBACK_TIMEOUT_US 40000U  // fail closed after two missed 50 Hz frames

// Keep SET/RES intent fresh until PEDALS reports engagement.
#define MAZDA_ENGAGE_BTN_WINDOW 10U
// A wheel cancel turns MRCC main off for real, and PEDALS trails the press: keep the context
// on the 50 Hz PEDALS clock for carstate's CANCEL_CONTEXT_T so the main-off edge lands under
// braking, where a brake-only bit dropout would otherwise be held.
#define MAZDA_CANCEL_CONTEXT_FRAMES 25U

static bool mazda_longitudinal = false;
// Declared by the driver: the TJA button owns lateral and MRCC no longer drives the main edge.
static bool mazda_tja_button = false;
static bool mazda_steer_to_zero_eps = false;
static bool mazda_legacy_fw_eps = false;
static uint32_t mazda_engage_btn_frames = 0U;
static uint32_t mazda_cancel_context_frames = 0U;
static bool mazda_ti = false;
static bool mazda_ti_feedback_healthy = false;
static uint32_t mazda_ti_feedback_ts = 0U;
static int mazda_ti_desired_torque_last = 0;
static int mazda_ti_rt_torque_last = 0;
static uint32_t mazda_ti_ts_torque_check_last = 0U;

// Mirror carstate's radar-ownership guard so panda and MADS arm on the same edge. Start the
// 50 Hz clock from the first synthetic CRZ_INFO because rx never sees the stock copy.
#define MAZDA_RADAR_SILENT_FRAMES 50U
static bool mazda_radar_mastered = false;
static uint32_t mazda_mastered_pedals_frames = 0U;
static bool mazda_radar_was_silenced = false;

// Pin replaced-radar traffic to captured stock patterns where possible.

// Each radar generation sends its own static capture; the controller picks the dialect (mazdacan.py)
static bool mazda_radar_static_msg_valid(const CANPacket_t *msg) {
  bool capture_2022 = (msg->data[0] == 0x00U) && (msg->data[1] == 0x08U) &&
                      (msg->data[2] == 0xc0U) && (msg->data[3] == 0x00U) &&
                      (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
                      (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
  bool capture_g46l = (msg->data[0] == 0x00U) && (msg->data[1] == 0x98U) &&
                      (msg->data[2] == 0x40U) && (msg->data[3] == 0x00U) &&
                      (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
                      (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
  return capture_2022 || capture_g46l;
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
  }

  return valid;
}

static bool mazda_synthetic_lead_radar_track_msg_valid(const CANPacket_t *msg) {
  // Permit only the distance and relative-velocity fields in the occupied-track template.
  return (msg->addr == MAZDA_RADAR_TRACK_4) &&
         ((msg->data[1] & 0x0fU) == 0x0eU) && (msg->data[2] == 0x00U) &&
         ((msg->data[4] & 0x1fU) == 0x1cU) && (msg->data[5] == 0x00U) &&
         (msg->data[6] == 0x00U) && ((msg->data[7] & 0xf0U) == 0x00U);
}

static bool mazda_radar_track_msg_valid(const CANPacket_t *msg) {
  // Occupied tracks represent perception and remain valid while controls are disengaged.
  return mazda_empty_radar_track_msg_valid(msg) ||
         mazda_synthetic_lead_radar_track_msg_valid(msg);
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

// fwd-declared: defined below with the camera-LKAS ownership rules
static bool mazda_openpilot_controlling(void);

static bool mazda_ti_torque_cmd_checks(int desired_torque) {
  // Rates mirror the STZ-EPS tune (EPS accepts 12/frame); magnitude is capped at the TI
  // hardware's real ceiling. Non-STZ controllers self-cap lower (6/15/192) under this backstop.
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
  // one torque master: the TI may only drive when openpilot owns the LKAS addresses, so a
  // MADS-pause or TJA-press transient can never overlap a forwarded camera 0x243 with live
  // TI torque on the AUX bus (upstream's conditional camera forwarding would allow it)
  bool controls = mazda_openpilot_controlling();
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
      // With the TJA button owning lateral, MRCC no longer drives the MADS main edge: its
      // falling edge would exit the panda's lateral while the software's MADS stays on.
      if (!mazda_tja_button) {
        acc_main_on = GET_BIT(msg, 17U);
      }
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_tja_button) {
      // The physical TJA button is the MADS lateral switch, so lateral no longer follows MRCC.
      mads_button_press = GET_BIT(msg, MAZDA_TJA_BUTTON_BIT) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_longitudinal) {
      // A physical cancel press always exits controls, and explains the main-off that follows.
      bool cancel = GET_BIT(msg, 0U);
      if (cancel) {
        controls_allowed = false;
        mazda_cancel_context_frames = MAZDA_CANCEL_CONTEXT_FRAMES;
      }
      // Record SET/RES intent for the engagement qualifier below.
      if (GET_BIT(msg, 2U) || GET_BIT(msg, 4U) || GET_BIT(msg, 5U)) {
        mazda_engage_btn_frames = MAZDA_ENGAGE_BTN_WINDOW;
      } else if (mazda_engage_btn_frames > 0U) {
        mazda_engage_btn_frames -= 1U;
      } else {
      }
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
    }

    if (msg->addr == MAZDA_PEDALS) {
      bool brake = (msg->data[0] & 0x10U);
      if (mazda_longitudinal) {
        // Keep radar ownership latched; returning stock traffic is handled as a fault.
        if (mazda_radar_mastered && (mazda_mastered_pedals_frames < MAZDA_RADAR_SILENT_FRAMES)) {
          mazda_mastered_pedals_frames += 1U;
        }
        mazda_radar_was_silenced = mazda_radar_was_silenced ||
                                   (mazda_mastered_pedals_frames >= MAZDA_RADAR_SILENT_FRAMES);

        // Derive cruise state from PEDALS after radar teardown. Ignore transient brake-only
        // samples where both cruise bits are low.
        bool cruise_engaged = GET_BIT(msg, 3U);
        bool acc_armed = GET_BIT(msg, 2U) || cruise_engaged;
        bool brake_free = !brake && !brake_pressed_prev;

        // Main mirrors carstate's cruise_available: it follows arming, and a both-low sample is
        // held under braking unless a wheel cancel explains it. Without the cancel path, main
        // toggled at a stop with the brake held never falls, the next press has no rising edge,
        // and MADS runs into 200 rejected frames (route 000001c9--0b2a64a214 seg 0).
        if (mazda_tja_button) {
          // the button is the lateral switch; MRCC is cruise only
        } else if (acc_armed) {
          // Gate the main edge on radar ownership to align with software availability.
          acc_main_on = mazda_radar_was_silenced;
        } else if (brake_free || (mazda_cancel_context_frames > 0U)) {
          acc_main_on = false;
        } else {
        }
        if (mazda_cancel_context_frames > 0U) {
          mazda_cancel_context_frames -= 1U;
        }

        if (acc_armed || cruise_engaged_prev || brake_free) {
          // Require recent SET/RES intent on the engaged edge; ACC_ACTIVE alone may acknowledge
          // synthetic traffic rather than a driver request.
          if (cruise_engaged && !cruise_engaged_prev && (mazda_engage_btn_frames > 0U)) {
            controls_allowed = true;
          }
          if (!cruise_engaged) {
            controls_allowed = false;
          }
          cruise_engaged_prev = cruise_engaged;
        }
      }
      brake_pressed = brake;
    }
  }

  if (mazda_ti && ((int)msg->bus == MAZDA_AUX) && (msg->addr == MAZDA_TI_FEEDBACK)) {
    update_sample(&torque_driver, (int)msg->data[0] - 127);
  }
}

static bool mazda_is_lka_addr(int addr) {
  return (((unsigned int)addr == MAZDA_LKAS) || ((unsigned int)addr == MAZDA_LKAS_HUD));
}

// The camera owns the LKAS addresses whenever openpilot is not steering. Lateral is its own
// axis under MADS (controls_allowed_lateral); with MADS off it follows cruise. Cruise alone must
// not claim them: under stock long that silenced the camera's own TJA/CTS with MADS off while the
// dash showed nothing (route 00000018--5655da2c1c seg 15).
static bool mazda_openpilot_controlling(void) {
  return controls_allowed_lateral || (controls_allowed && !m_mads_state.system_enabled);
}

// The one CRZ_BTNS frame openpilot may put on the camera bus: the TJA button pressed over the
// wheel's idle pattern (00 09 ff Cx 00 00 00 00, Cx = MODE_X_INV, MODE_Y_INV and the counter),
// no other button. It presses the camera's own TJA/CTS off whenever the camera is armed, so the
// two lane-centering systems never run at once and the camera never takes the wheel behind a
// MADS-off press. Accepted in every state: the frame only reaches the camera and can only
// toggle its lane centering, which the wheel button does anyway; the camera's torque is still
// vetoed whenever openpilot steers (docs/zoompilot/mazda-lateral.md, "The camera's own TJA/CTS
// state").
static bool mazda_cam_tja_press_msg_valid(const CANPacket_t *msg) {
  return (msg->data[0] == 0x00U) && (msg->data[1] == 0x09U) && (msg->data[2] == 0xffU) &&
         ((msg->data[3] & 0xc3U) == 0xc0U) && (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
         (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  // Stock pre-2022 EPS envelope.
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 800,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // The measured EPS envelope, selected by either firmware bit: the EPS's 12-count hardware
  // slew, with max_rate_down equal to the controller retreat rate so driver-limit winddown
  // frames remain valid.
  const TorqueSteeringLimits MAZDA_STEER_TO_ZERO_EPS_STEERING_LIMITS = {
    .max_torque = 1200,
    .max_rate_up = 12,
    .max_rate_down = 12,
    .max_rt_delta = 384,
    .driver_torque_multiplier = 15,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // CRZ_INFO.ACCEL_CMD uses 0.001 m/s2 raw units after removing the offset.
  const LongitudinalLimits MAZDA_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 0,
  };

  bool tx = true;
  bool main_bus = msg->bus == (unsigned char)MAZDA_MAIN;
  bool long_replacement_bus = main_bus || (msg->bus == (unsigned char)MAZDA_CAM);

  if (main_bus && (msg->addr == MAZDA_LKAS)) {
    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

    const TorqueSteeringLimits *limits = &MAZDA_STEERING_LIMITS;
    if (mazda_steer_to_zero_eps || mazda_legacy_fw_eps) {
      limits = &MAZDA_STEER_TO_ZERO_EPS_STEERING_LIMITS;
    } else {
      // upstream's pre-2022 envelope, no longer selected by the interface
    }
    if (steer_torque_cmd_checks(desired_torque, -1, *limits)) {
      tx = false;
    }
  }

  // Fork: TI command on the AUX bus. 12-bit torque duplicated in both halves, fixed key.
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

  // Run after steering checks, which reset rate-limit state while disengaged.
  if (main_bus && mazda_is_lka_addr(msg->addr) && !mazda_openpilot_controlling()) {
    tx = false;
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_CRZ_INFO)) {
    // Allow byte-exact stock standby patterns with the raw 8190 command sentinel.
    bool stock_standby = (msg->data[0] == 0x01U) && (msg->data[1] == 0xffU) &&
                         (msg->data[2] == 0xe3U) && (msg->data[3] == 0xffU) &&
                         ((msg->data[4] & 0xfbU) == 0xc0U) &&
                         ((msg->data[5] & 0x7fU) == 0x00U) &&
                         ((msg->data[6] & 0xf0U) == 0x00U) &&
                         (msg->data[7] == ((0xffU - ((msg->data[0] + msg->data[1] + msg->data[2] + msg->data[3] +
                                                     msg->data[4] + msg->data[5] + msg->data[6]) & 0xffU)) & 0xffU));

    // Assemble the 13-bit ACCEL_CMD unsigned for MISRA 10.1, then remove its 4096 offset.
    uint32_t accel_raw = (((uint32_t)msg->data[2] & 0x3U) << 11) | ((uint32_t)msg->data[3] << 3) | ((uint32_t)msg->data[4] >> 5);
    int desired_accel = (int)accel_raw - 4096;
    if (!stock_standby && longitudinal_accel_checks(desired_accel, MAZDA_LONG_LIMITS)) {
      tx = false;
    }

    // ACC_ACTIVE requires prior controls permission established from the physical SET edge.
    bool acc_active = GET_BIT(msg, 33U);
    if (!controls_allowed && acc_active) {
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
    // Allow tester-present and default/programming session control only.
    bool tester_present = (msg->data[0] == 0x02U) && (msg->data[1] == 0x3eU) && (msg->data[2] == 0x80U);
    bool session_control = (msg->data[0] == 0x02U) && (msg->data[1] == 0x10U) &&
                           ((msg->data[2] == 0x01U) || (msg->data[2] == 0x02U));
    if (!tester_present && !session_control) {
      tx = false;
    }
  }

  if (main_bus && (msg->addr == MAZDA_CRZ_BTNS)) {
    // Permit resume only while controlling and cancel only while not controlling.
    bool cancel_cmd = (msg->data[0] == 0x1U);
    if (!controls_allowed && !cancel_cmd) {
      tx = false;
    }
    // The TJA button is never pressed on the car's side: it would toggle MADS through the
    // rx hook and arm MRCC in the body.
    if (GET_BIT(msg, MAZDA_TJA_BUTTON_BIT)) {
      tx = false;
    }
  }

  if ((msg->bus == (unsigned char)MAZDA_CAM) && (msg->addr == MAZDA_CRZ_BTNS)) {
    // The camera-side press exists only to switch the camera's TJA/CTS off: byte-exact, any state.
    if (!mazda_cam_tja_press_msg_valid(msg)) {
      tx = false;
    }
  }

  // The first synthetic CRZ_INFO marks the radar ownership transition.
  if (tx && main_bus && (msg->addr == MAZDA_CRZ_INFO) && mazda_longitudinal) {
    mazda_radar_mastered = true;
  }

  return tx;
}

static bool mazda_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == MAZDA_CAM) {
    if (mazda_is_lka_addr(addr)) {
      block_msg = mazda_openpilot_controlling();
    }
  }

  return block_msg;
}

static safety_config mazda_init(uint16_t param) {
  mazda_engage_btn_frames = 0U;
  mazda_cancel_context_frames = 0U;
  mazda_radar_mastered = false;
  mazda_mastered_pedals_frames = 0U;
  mazda_radar_was_silenced = false;

  static const CanMsg MAZDA_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
    // The camera press: no relay check, so the wheel's own 0x09d keeps forwarding to the camera.
    {MAZDA_CRZ_BTNS, MAZDA_CAM, 8, .check_relay = false},
  };

  static const CanMsg MAZDA_TI_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_TI_LKAS, MAZDA_AUX, 8, .check_relay = false},
  };

// Replaced-radar addresses omit relay checks because the radar remains live during boot and
// hand-back. carstate enforces single ownership instead.
  static const CanMsg MAZDA_LONG_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, MAZDA_CAM, 8, .check_relay = false},
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
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, MAZDA_CAM, 8, .check_relay = false},
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
  mazda_steer_to_zero_eps = GET_FLAG(param, MAZDA_PARAM_STEER_TO_ZERO_EPS);
  mazda_legacy_fw_eps = GET_FLAG(param, MAZDA_PARAM_LEGACY_FW_EPS);
  mazda_tja_button = GET_FLAG(current_safety_param_sp, MAZDA_PARAM_SP_TJA_BUTTON);
  mazda_ti = GET_FLAG(param, MAZDA_PARAM_TI);
  mazda_ti_feedback_healthy = false;
  mazda_ti_feedback_ts = 0U;
  mazda_ti_reset_torque_state();
  acc_main_on = false;

  safety_config ret;
  if (mazda_longitudinal) {
    // CRZ_CTRL intentionally disappears after radar teardown.
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
