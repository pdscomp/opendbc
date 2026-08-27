from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.values import DBC, LKAS_LIMITS, CarControllerParams, MazdaFlags, TorqueInterceptorState
from opendbc.sunnypilot.car.mazda.carstate_ext import CarStateExt

ButtonType = structs.CarState.ButtonEvent.Type

FSC_SETTLE_FRAMES = int(CarControllerParams.FSC_SETTLE_T / DT_CTRL)
STOCK_RADAR_ALIVE_FRAMES = int(CarControllerParams.STOCK_RADAR_ALIVE_T / DT_CTRL)
STOCK_RADAR_GUARD_FRAMES = int(CarControllerParams.STOCK_RADAR_GUARD_T / DT_CTRL)
CANCEL_CONTEXT_FRAMES = int(CarControllerParams.CANCEL_CONTEXT_T / DT_CTRL)
CAM_LANEINFO_FRESH_FRAMES = int(CarControllerParams.CAM_LANEINFO_FRESH_T / DT_CTRL)


class CarState(CarStateBase, CarStateExt):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    CarStateExt.__init__(self, CP, CP_SP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.lkas_allowed_speed = False
    self.ti_lkas_allowed = False

    self.distance_button = 0
    self.accel_button = 0
    self.decel_button = 0
    self.cancel_button = 0
    self.resume_button = 0
    self.main_button = 0

    self.cruise_available = False
    self.cruise_enabled = False
    self.brake_pressed_prev = False
    self.stock_radar_silent_frames = 0
    self.radar_was_silenced = False
    self.cancel_context_frames = 0
    self.cam_laneinfo_seen = False
    self.cam_laneinfo_silent_frames = 0
    self.cam_empty_seen = False
    self.radar_session_refused = False
    self.fsc_settled_frames = 0
    # the body ECU has taken the standstill hold over and is holding the brakes itself
    self.brake_hold = False

  @property
  def fsc_settled(self) -> bool:
    return self.fsc_settled_frames >= FSC_SETTLE_FRAMES

  @property
  def stock_radar_alive(self) -> bool:
    return self.stock_radar_silent_frames < STOCK_RADAR_ALIVE_FRAMES

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ti_enabled = bool(self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR)

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )

    # Match panda speed reading. standstill deliberately comes off ENGINE_DATA while vEgo
    # comes off WHEEL_SPEEDS: panda's vehicle_moving reads the same ENGINE_DATA field, so the
    # two agree on when the car counts as stopped -- and standstill is load-bearing for the
    # longitudinal hold, so the ~0.03 m/s disagreement with vEgo at the stop is the price of
    # that parity
    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph <= .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    self.brake_hold = cp.vl["GEAR"]["BRAKE_HOLD"] == 1

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    if ti_enabled:
      cp_body = can_parsers[Bus.body]
      ti_feedback = cp_body.vl["TI_FEEDBACK"]
      ret.steeringTorque = ti_feedback["TI_TORQUE_SENSOR"]
      ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 6, 5)
      # known TI firmware reports VERSION_NUMBER 1 or 16 (0x10); both are healthy
      self.ti_lkas_allowed = cp_body.can_valid and \
        ti_feedback["VERSION_NUMBER"] in (1, 16) and \
        ti_feedback["STATE"] == TorqueInterceptorState.RUN and \
        not any(ti_feedback[s] for s in ("VIOL", "ERROR", "RAMP_DOWN"))
      ret_sp.torqueInterceptorReady = self.ti_lkas_allowed
    else:
      ret.steeringTorque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]
      ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD, 5)

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]

    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"],
                        cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])

    # TODO: this should be from 0 - 1.
    ret.gasPressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0

    # Either due to low speed or hands off
    lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1

    if self.CP.minSteerSpeed > 0:
      # LKAS is enabled at 52kph going up and disabled at 45kph going down
      # wait for LKAS_BLOCK signal to clear when going up since it lags behind the speed sometimes
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not lkas_blocked:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    # CAM_LANEINFO freshness, same shape as stock_radar_silent_frames below: before the
    # first frame the parser reads all-zero, and through a camera dropout it repeats stale
    # values; neither may drive the FSC settle gate or invalidLkasSetting
    if len(cp_cam.vl_all["CAM_LANEINFO"]["LANE_LINES"]) > 0:
      self.cam_laneinfo_seen = True
      self.cam_laneinfo_silent_frames = 0
    else:
      self.cam_laneinfo_silent_frames += 1
    cam_laneinfo_fresh = self.cam_laneinfo_seen and self.cam_laneinfo_silent_frames < CAM_LANEINFO_FRESH_FRAMES

    # Camera SCBS display -> stockFcw (telemetry: upstream maps no alert to it). 0x21d leaves
    # its idle 0x7f status only while actively showing the collision display (route 0000004d
    # t+213; zero episodes in 50 h of stock cruising); the DBC-named warning bits ride along.
    # A seen latch is guard enough here: unlike the laneinfo gate above, a stale value only
    # strands a log flag. stockAeb stays unmapped: no candidate signal has ever activated.
    if not self.cam_empty_seen:
      self.cam_empty_seen = len(cp_cam.vl_all["CAM_EMPTY"]["STATUS"]) > 0
    cam_empty = cp_cam.vl["CAM_EMPTY"]
    ped = cp_cam.vl["CAM_PEDESTRIAN"]
    ret.stockFcw = (self.cam_empty_seen and cam_empty["STATUS"] != 0x7F) or \
                   ped["PED_WARNING"] == 1 or ped["BRAKE_WARNING"] == 1

    if self.CP.openpilotLongitudinalControl:
      # The radar teardown silences the radar-owned CRZ_CTRL frame, so cruise state comes
      # from PEDALS: ACC_OFF means MRCC is armed but idle, ACC_ACTIVE means it is engaged.
      # Brake-only samples can arrive with both bits low mid-press; mirror the panda rx
      # guard and hold the previous state through them, else MADS sees a false
      # availability drop and force-disengages lateral.
      acc_armed = cp.vl["PEDALS"]["ACC_OFF"] == 1
      acc_active = cp.vl["PEDALS"]["ACC_ACTIVE"] == 1
      brake_free = not ret.brakePressed and not self.brake_pressed_prev
      # The brake hold below exists for brake-only PEDALS samples that arrive with both bits
      # low mid-press. A wheel CANCEL is different: it turns the MRCC main state off for real,
      # and it has to land even with the brake down -- holding through it kept lateral engaged
      # against a cancel mashed under braking until the brake was released 4 s later (route
      # 7f9e3ff336 t+484-488). The PEDALS reaction runs a few frames behind the button, so
      # cancel context outlives the press by a moment.
      if cp.vl["CRZ_BTNS"]["CAN_OFF"] == 1:
        self.cancel_context_frames = CANCEL_CONTEXT_FRAMES
      elif self.cancel_context_frames > 0:
        self.cancel_context_frames -= 1
      if acc_armed or acc_active:
        self.cruise_available = True
      elif brake_free or self.cancel_context_frames > 0:
        self.cruise_available = False
      if acc_armed or acc_active or self.cruise_enabled or brake_free:
        self.cruise_enabled = acc_active

      # Two-master guard: while the stock radar still broadcasts CRZ_INFO, our synthetic
      # frames would fight it on the bus, so engagement stays blocked until it has been
      # silent for 1 second. The block wears two different hats:
      #  - Before the first teardown of the drive this is the expected boot phase (FSC
      #    settle + UDS handover, ~10-15 s), not a fault. Holding availability low keeps
      #    engagement out with at most a wrongCarMode no-entry toast; raising accFaulted
      #    here showed a permanent "Cruise Fault: Restart the Car" on every start for a
      #    condition that clears by itself.
      #  - After the radar has been silenced once, hearing it again is a genuine
      #    two-master conflict (dropped tester present, S3 recovery, or the ordered
      #    hand-back) and is a real accFaulted. The alpha-long toggle monitor relies on
      #    exactly this edge as its "stock radar heard" acknowledgment.
      if len(cp.vl_all["CRZ_INFO"]["CTR1"]) > 0:
        self.stock_radar_silent_frames = 0
      else:
        self.stock_radar_silent_frames += 1

      # The radar answers every session request within ~10 ms (route 000000fe t+15.0: request
      # 02 10 02 -> 06 50 02 with P2* = 5.0 s, the S3 timeout), and the session manager
      # consumes this on the same control frame, so no freshness window is needed. NRC 0x78
      # (response pending) is the one negative response UDS clients wait through, not fail on.
      resp = cp.vl_all["RADAR_UDS_RESPONSE"]
      self.radar_session_refused = any(
        sid == 0x7F and sub == uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL and nrc != 0x78
        for sid, sub, nrc in zip(resp["SID"], resp["SUB"], resp["NRC"], strict=True))
      silenced = self.stock_radar_silent_frames >= STOCK_RADAR_GUARD_FRAMES
      ret.accFaulted = self.radar_was_silenced and not silenced
      self.radar_was_silenced |= silenced

      ret.cruiseState.available = self.cruise_available and self.radar_was_silenced
      ret.cruiseState.enabled = self.cruise_enabled

      # FSC settle timer (the radar teardown gate): the camera broadcasts a boot-in-progress
      # state on CAM_LANEINFO (NO_ERR_BIT, a pure boot marker clearing at 2.8-6.0 s and never
      # set again while driving), then runs a radar-presence check in the following seconds.
      # A latched fault (ERR_BIT) also shows the boot marker clear, so it must hold the timer
      # at zero. The seen latch matters: before the first frame the parser reads all-zero,
      # which would count as settled.
      #
      # BIT2 used to gate this too. It is byte-identical to NO_ERR_BIT on every frame of the
      # 40 alpha-long routes this was developed against, so it carried no information there,
      # but another CX-5 2022 with identical camera firmware (GSH7-67XK2-U) cold-booted with
      # BIT2 latched high and NO_ERR_BIT clear for a whole ignition cycle. That pinned the
      # timer at zero, so the radar was never silenced and the two-master guard held
      # accFaulted for the entire drive with nothing to tell the driver why.
      laneinfo = cp_cam.vl["CAM_LANEINFO"]
      settled = cam_laneinfo_fresh and not (laneinfo["NO_ERR_BIT"] or laneinfo["ERR_BIT"])
      self.fsc_settled_frames = self.fsc_settled_frames + 1 if settled else 0
    else:
      # TODO: the signal used for available seems to be the adaptive cruise signal, instead of the main on
      #       it should be used for carState.cruiseState.nonAdaptive instead
      ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
      ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1
    self.brake_pressed_prev = ret.brakePressed
    # PEDALS.STANDSTILL is the PCM's "wheels are stopped" bit, not a stock-ACC hold state, so it
    # stays set for exactly as long as the car is not moving. LongControl gates its
    # starting_condition on this, so reporting it under openpilot longitudinal deadlocks every
    # stop: long control cannot leave stopping until the car moves, and the car cannot move until
    # long control leaves stopping. The stock MRCC is not in the loop here anyway -- its radar is
    # silenced and we synthesize its frames -- so there is no stock standstill to report. Hyundai
    # and Tesla report False for the same reason.
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1 and not self.CP.openpilotLongitudinalControl
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS

    # stock lkas should be on
    # TODO: is this needed?
    ret.invalidLkasSetting = not ti_enabled and cam_laneinfo_fresh and cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0

    if ret.cruiseState.enabled:
      if not self.lkas_allowed_speed and self.acc_active_last:
        self.low_speed_alert = True
      else:
        self.low_speed_alert = False
    ret.lowSpeedAlert = self.low_speed_alert

    # Check if LKAS is disabled due to lack of driver torque when all other states indicate
    # it should be enabled (steer lockout). Don't warn until we actually get lkas active
    # and lose it again, i.e, after initial lkas activation
    if ti_enabled:
      # TI drops out of RUN at low speed/standstill by design (steering-current
      # self-protection) and recovers on roll-out — only fault at real speed,
      # where not-ready is genuine and the soft-disable takeover is warranted.
      ret.steerFaultTemporary = not self.ti_lkas_allowed and ret.vEgo > 10
    elif self.CP.minSteerSpeed > 0:
      ret.steerFaultTemporary = self.lkas_allowed_speed and lkas_blocked
    else:
      # CX-5 2022: EPS accepts steering at all speeds regardless of LKAS_BLOCK.
      # Verified across 5.5M frames: LKAS_BLOCK never indicates a real steering failure.
      # (minSteerSpeed == 0 is the "2022+ EPS present" marker, here and in CarControllerParams)
      ret.steerFaultTemporary = False

    self.acc_active_last = ret.cruiseState.enabled

    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]

    # camera signals
    self.cam_lkas = cp_cam.vl["CAM_LKAS"]
    self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
    ret.steerFaultPermanent = not ti_enabled and cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1

    # cruise control button events: distance, inc, dec, resume, cancel, and main
    prev_distance_button = self.distance_button
    prev_accel_button = self.accel_button
    prev_decel_button = self.decel_button
    prev_cancel_button = self.cancel_button
    prev_resume_button = self.resume_button
    prev_main_button = self.main_button
    self.distance_button = cp.vl["CRZ_BTNS"]["DISTANCE_LESS"]
    # On CX-5 2022 the wheel "+" button toggles SET_P (not RES); RES is the resume button.
    # Verified against route 0000019c--84a5408a38 seg2/3: holding "+" emits SET_P=1, body ECU increments CRZ_SPEED.
    self.accel_button = cp.vl["CRZ_BTNS"]["SET_P"]
    self.decel_button = cp.vl["CRZ_BTNS"]["SET_M"]
    # CAN_OFF carries the cancel intent. Without an event here, ICBM's readiness gate never
    # learns the driver is canceling, so it keeps spamming CRZ_BTNS with cancel=0 and the
    # body ECU treats the latest non-cancel frame as authoritative. Critical for cancel-safety.
    self.cancel_button = cp.vl["CRZ_BTNS"]["CAN_OFF"]
    self.resume_button = cp.vl["CRZ_BTNS"]["RES"]
    self.main_button = int(cp.vl["CRZ_BTNS"]["MODE_X"] == 1 and cp.vl["CRZ_BTNS"]["MODE_Y"] == 1)

    ret.buttonEvents = [
      *create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.accel_button, prev_accel_button, {1: ButtonType.accelCruise}),
      *create_button_events(self.decel_button, prev_decel_button, {1: ButtonType.decelCruise}),
      *create_button_events(self.cancel_button, prev_cancel_button, {1: ButtonType.cancel}),
      *create_button_events(self.resume_button, prev_resume_button, {1: ButtonType.resumeCruise}),
      *create_button_events(self.main_button, prev_main_button, {1: ButtonType.mainCruise}),
    ]

    CarStateExt.update(self, ret, ret_sp, can_parsers)

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = []
    if CP.openpilotLongitudinalControl:
      # no liveness checks: the stock frame is expected to disappear after the radar
      # teardown (its presence is what the two-master guard watches for), and the UDS
      # response only arrives when a session request is answered
      pt_messages.append(("CRZ_INFO", float("nan")))
      pt_messages.append(("RADAR_UDS_RESPONSE", float("nan")))
    cam_messages = [
      # read through vl_all, which unlike vl has no lazy registration
      ("CAM_LANEINFO", 0),
      ("CAM_TRAFFIC_SIGNS", 0),
      ("CAM_EMPTY", 0),
    ]
    parsers = {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }
    if CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      parsers[Bus.body] = CANParser(DBC[CP.carFingerprint][Bus.pt], [("TI_FEEDBACK", 50)], 1)

    return parsers
