import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg, rate_limit, structs, uds
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.longitudinal import (RADAR_ADDR, AdvertisedLead, RadarSessionManager, RadarSessionState,
                                            StandstillHold, create_radar_session_msg)
from opendbc.car.mazda.values import CarControllerParams, Buttons, MazdaFlags, TorqueInterceptorControllerParams

from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# Synthetic radar frames go to the car and to the camera; the panda only forwards
# received frames between those buses, not our own transmissions.
LONG_BUSES = (0, 2)


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    if not CP.flags & MazdaFlags.GEN1:
      # every message builder in mazdacan assumes the GEN1 frame layouts; a new platform
      # needs its own before it can be admitted
      raise NotImplementedError(f"unsupported platform: {CP.carFingerprint}")
    self.params = CarControllerParams(CP)
    self.apply_torque_last = 0
    # ponytail: keep TI RT state local until another host torque path needs the same limiter.
    self.ti_apply_torque_last = 0
    self.ti_rt_torque_last = 0
    self.ti_rt_torque_last_ts = None
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.stop_and_go = StandstillHold()
    self.lead_adv = AdvertisedLead()
    self.long_counter = 0
    self.radar_counter = 0
    self.radar_session = RadarSessionManager()
    self.accel_last = 0.
    self.release_ramp = None

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    apply_torque = 0
    ti_apply_torque = 0

    # Speed-dependent STEER_MAX (CX-5 2022: 1200 below 32 mph, 800 above)
    if hasattr(self.params, 'STEER_MAX_LOOKUP'):
      steer_max = round(float(np.interp(CS.out.vEgoRaw, self.params.STEER_MAX_LOOKUP[0],
                                         self.params.STEER_MAX_LOOKUP[1])))
    else:
      steer_max = self.params.STEER_MAX

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * steer_max))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, self.params, steer_max)

    if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      if CC.latActive and CS.ti_lkas_allowed:
        ti_new_torque = int(round(CC.actuators.torque * TorqueInterceptorControllerParams.STEER_MAX))
        ti_apply_torque = apply_driver_steer_torque_limits(ti_new_torque, self.ti_apply_torque_last,
                                                           CS.out.steeringTorque, TorqueInterceptorControllerParams)
        if self.ti_rt_torque_last_ts is None:
          self.ti_rt_torque_last_ts = now_nanos
        highest_torque = max(self.ti_rt_torque_last, 0) + TorqueInterceptorControllerParams.STEER_MAX_RT_DELTA
        lowest_torque = min(self.ti_rt_torque_last, 0) - TorqueInterceptorControllerParams.STEER_MAX_RT_DELTA
        ti_apply_torque = max(min(ti_apply_torque, highest_torque), lowest_torque)
        if now_nanos - self.ti_rt_torque_last_ts > TorqueInterceptorControllerParams.STEER_RT_INTERVAL_NS:
          self.ti_rt_torque_last = ti_apply_torque
          self.ti_rt_torque_last_ts = now_nanos
      else:
        self.ti_rt_torque_last = 0
        self.ti_rt_torque_last_ts = now_nanos
      self.ti_apply_torque_last = ti_apply_torque

    # Under op-long, controlsd raises cancel whenever cruiseState.enabled has no matching
    # CC.enabled (pcmCruise). While the stock radar still owns the bus -- the pre-teardown
    # settle window and the silencing-failed stay-stock fallback -- that engagement is the
    # driver's own stock MRCC (openpilot cannot engage there: availability is held low), and
    # the 10 Hz CANCEL would turn its main off within ~100 ms. Leave it alone; the teardown
    # gate already waits out a stock engagement. Once the radar has been silenced a stock
    # engagement is impossible and cancel keeps handling state desync. (The deeper home is
    # carstate not reporting a stock engagement as cruiseState.enabled under op-long at all;
    # that needs an audit of every enabled consumer first, so the send is filtered here.)
    stock_mrcc_owns_cruise = self.CP.openpilotLongitudinalControl and not CS.radar_was_silenced
    if CC.cruiseControl.cancel and not stock_mrcc_owns_cruise:
      # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
      # a race condition with the stock system, where the second cancel from openpilot
      # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
      # read 3 messages and most likely sync state before we attempt cancel.
      self.brake_counter = self.brake_counter + 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        # Cancel Stock ACC if it's enabled while OP is disengaged
        # Send at a rate of 10hz until we sync with stock ACC state
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0
      if self.resume_requested(CC) and self.frame % 5 == 0:
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

    self.apply_torque_last = apply_torque

    if self.CP.openpilotLongitudinalControl:
      can_sends.extend(self.update_longitudinal(CC, CC_SP, CS))

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      steer_required = steer_required and CS.lkas_allowed_speed
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))
    if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      can_sends.append(mazdacan.create_ti_steering_control(self.packer, ti_apply_torque))

    # Intelligent Cruise Button Management
    # Suppress ICBM CRZ_BTNS spam while cancel/resume are in flight or while the driver is
    # holding the wheel cancel button. Without this guard ICBM's interleaved cancel=0 frames
    # race the driver's cancel=1 frames on the bus and the body ECU drops the cancel intent.
    icbm_suppress = CC.cruiseControl.cancel or CC.cruiseControl.resume or CS.cancel_button == 1
    if not icbm_suppress:
      can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame, self.last_button_frame))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / steer_max
    new_actuators.torqueOutputCan = apply_torque
    # report what actually went on the wire, not the plan: the clip, the standstill hold values,
    # the slew limit, and the zero we send through a gas override all live in accel_last
    new_actuators.accel = self.accel_last

    self.frame += 1
    return new_actuators, can_sends

  def resume_requested(self, CC) -> bool:
    """The resume button is the stock ACC's only lever on a standstill hold, so it belongs to the
    stock-longitudinal path alone.

    Under openpilot longitudinal we are the ACC, and the hold is released in-protocol: CRZ_INFO's
    stop bits drop, RESUME_UNLATCHING pulses and the command ramps positive off the plan. That is
    what the car's own MRCC does -- across 23 stock body-latched-hold releases with cruise
    engaged, 0 put a RES press on the bus and all 23 pulsed RESUME_UNLATCHING
    (tools/mazda_long/scan_stock_release.py). Toyota, Honda and Hyundai all gate their resume
    button off openpilotLongitudinalControl the same way and release through their own ACC frame.

    Pressing it here would also put a second writer on CRZ_BTNS at the release: ICBM owns that
    address, and both of its interlocks (icbm_suppress above and the controller's own readiness
    gate) key off CC.cruiseControl.resume, which carstate makes False under openpilot
    longitudinal by construction.
    """
    return not self.CP.openpilotLongitudinalControl and CC.cruiseControl.resume

  def update_longitudinal(self, CC, CC_SP, CS):
    can_sends = []

    # Radar session sequencing (the why lives on RadarSessionManager): hold off the takeover
    # until the FSC's cold-boot radar-presence check has cleared, and never yank the radar
    # out from under an active stock MRCC engagement (driver SET before the gate passed on a
    # warm boot) -- wait for the driver to disengage first.
    stock_radar_alive = CS.stock_radar_alive
    setup_ok = CS.fsc_settled and not (stock_radar_alive and CS.out.cruiseState.enabled)
    session_state = self.radar_session.update(setup_ok, stock_radar_alive, CC_SP.stockEcuHandBack,
                                              standstill=CS.out.standstill,
                                              session_refused=CS.radar_session_refused)
    # synthetic radar frames flow while we own the bus, and keep flowing through the
    # hand-back so the camera never sees a radar gap
    radar_master = session_state in (RadarSessionState.SILENCED, RadarSessionState.HANDBACK)

    if self.frame % CarControllerParams.RADAR_UDS_STEP == 0:
      if session_state == RadarSessionState.SILENCING:
        can_sends.append(create_radar_session_msg(uds.SESSION_TYPE.PROGRAMMING))
      elif session_state == RadarSessionState.HANDBACK:
        can_sends.append(create_radar_session_msg(uds.SESSION_TYPE.DEFAULT))
      elif session_state == RadarSessionState.SILENCED:
        # keeps the radar in its diagnostic session, and with it the stock frames silenced
        can_sends.append(make_tester_present_msg(RADAR_ADDR, 0, suppress_response=True))

    stopping = CC.actuators.longControlState == LongCtrlState.stopping
    # The engaged bits follow CC.enabled the way Honda drives ACC_CONTROL's CONTROL_ON: a gas
    # press is an override, not a disengagement, so enabled holds while controlsd drops
    # longActive and the command goes to zero. Clearing the bits mid-decel takes the PCM out
    # of ACC mode as the driver adds throttle, so a light pedal input lands as a lurch and a
    # rev flare; stock MRCC holds them through 9 of 11 decel overrides (analyze_gas_override.py,
    # 576 stock segments). (MADS lateral-only sits outside CC.enabled, so this stays False
    # with cruise off.)
    long_engaged = CC.enabled
    sm = self.stop_and_go
    sm.update(long_engaged, stopping, CS.out.standstill, CC.actuators.accel, CS.brake_hold,
              gas_pressed=CS.out.gasPressed)
    # runs engaged or not: the advertisement is perception (see AdvertisedLead)
    self.lead_adv.update(CC.hudControl.leadVisible, CC_SP.leadOne.dRel,
                         CC_SP.leadOne.vRel, sm.holding)

    if sm.just_released:
      # the release command follows stock's shape, not a slew off the hold value: a
      # never-latched stop relax-jumps into the release band in one frame, a latched hold
      # ramps off the relaxed -0.001 (values.py census). Slewing up from -1.024 instead kept
      # hold-grade braking under the release pulse, and the camera latched it as an SCBS
      # fault 90 ms in (route 00000053 t+714.8, real departing lead advertised)
      self.release_ramp = CarControllerParams.ACCEL_HOLD_LATCHED if sm.latched_release else \
                          CarControllerParams.ACCEL_RELEASE_BAND
    elif sm.holding or not CC.longActive:
      # a re-hold or a driver override takes the command back; the ramp is release-only
      self.release_ramp = None

    accel = 0.
    if CC.longActive:
      accel = float(np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      if self.release_ramp is not None and self.release_ramp < accel:
        # the release owns the command until its ramp catches the plan: stock climbs
        # ~+1.25 m/s3 straight through the blip or pulse and on into the drive-off
        accel = self.release_ramp
        self.release_ramp += CarControllerParams.ACCEL_RELEASE_RAMP * DT_CTRL
      else:
        self.release_ramp = None
        # Slew limit the plan-following command. accel_last is tracked through overrides too,
        # so taking control back when the driver lifts off ramps in instead of stepping.
        accel = rate_limit(accel, self.accel_last, CarControllerParams.ACCEL_WINDDOWN_LIMIT,
                           CarControllerParams.ACCEL_WINDUP_LIMIT)
      if sm.car_has_hold:
        # the body ECU is holding the brakes itself, so stop asking for them like stock does
        accel = CarControllerParams.ACCEL_HOLD_LATCHED
      elif sm.holding:
        # while the plan is braking the hold command is the plan's own, but the moment it
        # turns positive (release debounce) the hold freezes where it is:
        # stock never lets ACCEL_CMD climb while STOPPING is asserted, and pre-ramping toward
        # the plan here put the release's zero-cross inside the unlatch pulse, which the
        # camera latched as an SCBS fault (route 00000100 t+353)
        accel = min(accel, 0.) if CC.actuators.accel <= 0. else min(self.accel_last, 0.)
      if sm.resume_unlatching:
        # ceiling by pulse family: stock's command is negative in every never-latched blip
        # frame of the corpus and peaks at +0.25 m/s2 inside latched pulses. The ramp already
        # stays inside both; this is the invariant, kept as a guard.
        accel = min(accel, CarControllerParams.ACCEL_RESUME_PULSE_MAX if sm.latched_release else 0.)
    self.accel_last = accel

    if radar_master and self.frame % CarControllerParams.RADAR_STEP == 0:
      for bus in LONG_BUSES:
        can_sends.extend(mazdacan.create_radar_frames(bus, self.radar_counter, self.lead_adv.lead))
      self.radar_counter += 1

    if radar_master and self.frame % CarControllerParams.LONG_STEP == 0:
      acc_available = CS.out.cruiseState.available
      # mirror the driver's distance setting on the dash; stock shows gap 2 by default
      gap = (int(CC.hudControl.leadDistanceBars) or 2) if (long_engaged or acc_available) else 0
      acc_active_2 = sm.acc_active_2 if long_engaged else False
      for bus in LONG_BUSES:
        can_sends.append(mazdacan.create_acc_command(self.packer, bus, self.long_counter, accel,
                                                     long_active=long_engaged, acc_available=acc_available,
                                                     brake_pressed=CS.out.brakePressed,
                                                     stopping=sm.stop_bits, resume_unlatching=sm.resume_unlatching))
        can_sends.append(mazdacan.create_crz_ctrl(self.packer, bus, long_engaged, acc_available, gap,
                                                  self.lead_adv.has_lead, self.lead_adv.ctrl_phase,
                                                  acc_active_2))
      self.long_counter += 1

    return can_sends
