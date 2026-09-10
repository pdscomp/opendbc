#!/usr/bin/env python3
import unittest
from collections import deque

from opendbc.car import DT_CTRL
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.mazda.values import CAR, CarControllerParams, MazdaFlags, MazdaSafetyFlags, TorqueInterceptorControllerParams
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.mazda.values import MazdaSafetyFlagsSP
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg


def ti_command(torque: int, *, bus: int = 1, duplicate: int | None = None, key: int = 0xC461CE60,
               reserved_request: int = 0, reserved_duplicate: int = 0, length: int = 8):
  raw = torque + 2048
  duplicate_raw = raw if duplicate is None else duplicate + 2048
  dat = bytes([((raw >> 8) & 0xF) | reserved_request, raw & 0xFF,
               ((duplicate_raw >> 8) & 0xF) | reserved_duplicate, duplicate_raw & 0xFF]) + key.to_bytes(4, "big")
  return make_msg(bus, 0x249, length, dat[:length])


def ti_feedback(torque: int = 0, *, bus: int = 1, version: int = 1, state: int = 3,
                violation: int = 0, error: int = 0, ramp_down: int = 0, length: int = 8):
  dat = bytes([torque + 127, 0, version, state, violation, error, ramp_down, 0])
  return make_msg(bus, 0x24A, length, dat[:length])


def test_ti_bit_and_bus_mapping():
  # Every low safetyParam bit combination x bus: TI commands are admitted only under the
  # dedicated TI bit (8) on the AUX bus. Guards the flag-ABI migration in values.py/mazda.h.
  safety = libsafety_py.libsafety
  for param in range(16):
    for bus in (0, 1, 2):
      safety.init_tests()
      assert safety.set_safety_hooks(CarParams.SafetyModel.mazda, param) == 0
      for _ in range(6):
        safety.safety_rx_hook(ti_feedback())
      safety.set_controls_allowed(True)
      assert bool(safety.safety_tx_hook(ti_command(6, bus=bus))) == (
        bool(param & 8) and bus == 1
      )


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  """Upstream's envelope with no safety param bit. The interface no longer emits it for any
  Mazda; the panda keeps it as the default, so it stays proven."""

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0], [0x09d, 2]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  # camera 0x243/0x440 frames forward while openpilot is not controlling
  FWD_BLACKLISTED_ADDRS = {2: []}

  SAFETY_PARAM = 0

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [800]

  MAX_RT_DELTA = 300

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 1

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, self.SAFETY_PARAM)
    self.safety.init_tests()

  @classmethod
  def controller_params(cls):
    # the CarControllerParams branch this envelope pairs with: values.py keys it on the same
    # EPS bit interface.py hands the panda
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX5
      flags = 0
      if cls.SAFETY_PARAM & MazdaSafetyFlags.STEER_TO_ZERO_EPS:
        flags = MazdaFlags.STEER_TO_ZERO_EPS
      elif cls.SAFETY_PARAM & MazdaSafetyFlags.LEGACY_FW_EPS:
        flags = MazdaFlags.LEGACY_FW_EPS
    return CarControllerParams(FakeCP())

  def test_controller_rate_limits_equal_the_pandas(self):
    # driver_limit_check demands a retreat of at least max_rate_down per frame once the driver
    # bound is below the last command, and rejects anything above max_rate_up on the way up,
    # so the controller's per-frame deltas must be exactly the panda's, not merely within them
    params = self.controller_params()
    self.assertEqual(params.STEER_DELTA_UP, self.MAX_RATE_UP)
    self.assertEqual(params.STEER_DELTA_DOWN, self.MAX_RATE_DOWN)
    self.assertEqual(params.STEER_MAX, self.MAX_TORQUE)
    self.assertEqual(params.STEER_DRIVER_ALLOWANCE, self.DRIVER_TORQUE_ALLOWANCE)
    self.assertEqual(params.STEER_DRIVER_MULTIPLIER, self.DRIVER_TORQUE_FACTOR)

  def _closed_loop(self, params, frames, ctrl_last=0, frame0=0):
    """Run the real controller limiter frame by frame through the compiled safety model.
    frames yields (driver_torque, target); returns (rejected frames, full retreats, last cmd)."""
    rejected, full_retreats = [], 0
    for frame, (driver_torque, target) in enumerate(frames, start=frame0 + 1):
      self.safety.set_timer(frame * 10_000)
      self._rx(self._torque_driver_msg(driver_torque))
      cmd = apply_driver_steer_torque_limits(target, ctrl_last, driver_torque, params, self.MAX_TORQUE)
      if not self._tx(self._torque_cmd_msg(cmd)):
        rejected.append((frame, cmd, ctrl_last, driver_torque))
      full_retreats += abs(cmd) == abs(ctrl_last) - params.STEER_DELTA_DOWN
      ctrl_last = cmd
    return rejected, full_retreats, ctrl_last

  def test_driver_override_winddown_is_never_rejected(self):
    # Closed loop: the command is held at full torque while the driver ramps against it at
    # several slopes; the driver bound then falls faster than the controller retreats, so the
    # panda's max_rate_down requirement binds. Route 00000148 lost 171 consecutive frames here
    # when the panda demanded 25 and the controller retreated 12.
    params = self.controller_params()
    max_torque = self.MAX_TORQUE
    for sign in (1, -1):
      for slope in (1, 2, 5, 10, 30):
        with self.subTest(sign=sign, slope=slope):
          self.safety.init_tests()
          self.safety.set_controls_allowed(True)
          self._reset_torque_driver_measurement(0)
          # ramp up to full torque with no driver input
          ramp = [(0, max_torque * sign)] * (max_torque // self.MAX_RATE_UP + 5)
          rejected, _, last = self._closed_loop(params, ramp)
          self.assertEqual(rejected, [])
          self.assertEqual(last, max_torque * sign)
          # driver pushes back, harder each frame, to the top of the 8-bit sensor field
          rejected, full_retreats, _ = self._closed_loop(
            params, [(-sign * min(slope * f, 127), max_torque * sign) for f in range(300)],
            ctrl_last=last, frame0=len(ramp))
          self.assertEqual(rejected, [], f"{len(rejected)} frames rejected, first {rejected[:1]}")
          # the scenario only proves something if the bound outran the retreat at least once
          if slope * self.DRIVER_TORQUE_FACTOR > self.MAX_RATE_DOWN:
            self.assertGreater(full_retreats, 0)

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, values)

  def _laneinfo_msg(self):
    values = {"LINE_VISIBLE": 0}
    return self.packer.make_can_msg_safety("CAM_LANEINFO", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRZ_ACTIVE": enable}
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, values)

  def _button_msg(self, resume=False, cancel=False, set_m=False, set_p=False, tja=False, bus=0):
    values = {
      "TJA_BUTTON": tja,
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
      "SET_M": set_m,
      "SET_M_INV": (set_m + 1) % 2,
      "SET_P": set_p,
      "SET_P_INV": (set_p + 1) % 2,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", bus, values)

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))

  def test_steer_safety_check(self):
    # the common test, except that disengaged the camera owns 0x243 (test_stock_passthrough),
    # so the zero-torque frame upstream's rule lets through is vetoed too
    for speed in self._torque_speed_range:
      self._reset_speed_measurement(speed)
      max_torque = self._get_max_torque(speed)
      for enabled in [0, 1]:
        for t in range(int(-max_torque * 1.5), int(max_torque * 1.5)):
          self.safety.set_controls_allowed(enabled)
          self._set_prev_torque(t)
          self.assertEqual(bool(enabled) and abs(t) <= max_torque, self._tx(self._torque_cmd_msg(t)))

  def test_stock_passthrough(self):
    # one sender per address, the Tesla test_stock_lkas_passthrough shape: the camera owns
    # 0x243/0x440 whenever openpilot is not steering (stock lane keep, TJA/CTS and dash LDW
    # stay live); steering hands them to openpilot. The fwd hook forwards the camera copy
    # exactly while the tx hook vetoes ours, and vice versa. Under MADS lateral is its own
    # axis, so cruise alone leaves the camera in charge; with MADS off lateral follows cruise
    for mads in (False, True):
      self.safety.set_mads_params(mads, False, False)
      for controls_allowed, controls_allowed_lateral in [(False, False), (True, False), (False, True), (True, True)]:
        stock_active = not (controls_allowed_lateral or (controls_allowed and not mads))
        self.safety.set_controls_allowed(controls_allowed)
        self.safety.set_controls_allowed_lateral(controls_allowed_lateral)
        for addr, msg in ((0x243, self._torque_cmd_msg(0)), (0x440, self._laneinfo_msg())):
          fwd_bus = 0 if stock_active else -1
          self.assertEqual(fwd_bus, self.safety.safety_fwd_hook(2, addr), f"{mads=} {controls_allowed=} {controls_allowed_lateral=} {addr=:#x}")
          self.assertEqual(not stock_active, self._tx(msg), f"openpilot tx {mads=} {controls_allowed=} {controls_allowed_lateral=} {addr=:#x}")
    self.safety.set_mads_params(False, False, False)

  def _cam_tja_press(self, ctr=5, **overrides):
    # the wheel's idle pattern with the TJA bit: 00 09 ff Cx 00 00 00 00
    values = {"TJA_BUTTON": 1, "DISTANCE_LESS_INV": 1, "BIT1": 1, "BIT2": 1, "BIT3": 1, "CAN_OFF_INV": 1, "RES_INV": 1,
              "SET_P_INV": 1, "SET_M_INV": 1, "DISTANCE_MORE_INV": 1, "MODE_X_INV": 1, "MODE_Y_INV": 1, "CTR": ctr}
    values.update(overrides)
    return self.packer.make_can_msg_safety("CRZ_BTNS", 2, values)

  def test_cam_tja_press(self):
    # openpilot presses the camera's own TJA/CTS off on the camera bus whenever it is armed, so
    # the two lane-centering systems never run at once and a MADS-off press cannot hand the
    # wheel to the camera. Accepted in all eight states of test_stock_passthrough (the frame only
    # reaches the camera), and only byte-exact: the TJA bit over the wheel's idle pattern, any
    # counter, no other button
    self.assertEqual(bytes.fromhex("0009ffd400000000"), bytes(self.packer.make_can_msg("CRZ_BTNS", 2, {
      "TJA_BUTTON": 1, "DISTANCE_LESS_INV": 1, "BIT1": 1, "BIT2": 1, "BIT3": 1, "CAN_OFF_INV": 1, "RES_INV": 1, "SET_P_INV": 1,
      "SET_M_INV": 1, "DISTANCE_MORE_INV": 1, "MODE_X_INV": 1, "MODE_Y_INV": 1, "CTR": 5})[1]))
    for mads in (False, True):
      self.safety.set_mads_params(mads, False, False)
      for controls_allowed, controls_allowed_lateral in [(False, False), (True, False), (False, True), (True, True)]:
        self.safety.set_controls_allowed(controls_allowed)
        self.safety.set_controls_allowed_lateral(controls_allowed_lateral)
        for ctr in range(16):
          self.assertTrue(self._tx(self._cam_tja_press(ctr=ctr)), f"{mads=} {controls_allowed=} {controls_allowed_lateral=} {ctr=}")
        # anything else on the camera-side address is refused in every state
        for other in ({"TJA_BUTTON": 0}, {"CAN_OFF": 1, "CAN_OFF_INV": 0}, {"RES": 1, "RES_INV": 0}, {"SET_P": 1, "SET_P_INV": 0},
                      {"SET_M": 1, "SET_M_INV": 0}, {"DISTANCE_LESS": 1, "DISTANCE_LESS_INV": 0}, {"MODE_X": 1, "MODE_X_INV": 0},
                      {"MODE_Y": 1, "MODE_Y_INV": 0}, {"BIT1": 0}, {"BIT2": 0}, {"BIT3": 0}):
          self.assertFalse(self._tx(self._cam_tja_press(**other)), f"{other=} {mads=} {controls_allowed=} {controls_allowed_lateral=}")
        self.assertFalse(self._tx(make_msg(2, 0x09d, 8)))
    self.safety.set_mads_params(False, False, False)

  def test_tja_button_never_pressed_on_the_car_side(self):
    # bit 11 on bus 0 would toggle MADS through the rx hook and arm MRCC in the body
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertFalse(self._tx(self._button_msg(tja=True)))
      self.assertFalse(self._tx(self._button_msg(tja=True, resume=True)))
      self.assertFalse(self._tx(self._button_msg(tja=True, cancel=True)))
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))


class TestMazdaSteerToZeroEpsSafety(TestMazdaSafety):
  """2022+ steer-to-zero EPS (CX-5 2022, CX-9 2021, EPS swaps): MazdaSafetyFlags.STEER_TO_ZERO_EPS
  selects the 1200-count envelope with the EPS's own 12/12 slew."""

  SAFETY_PARAM = MazdaSafetyFlags.STEER_TO_ZERO_EPS

  MAX_RATE_UP = 12
  MAX_RATE_DOWN = 12
  MAX_TORQUE_LOOKUP = [0], [1200]

  MAX_RT_DELTA = 384

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 15

  def test_legacy_envelope_stays_upstreams_without_the_bit(self):
    # the bit only ever loosens the envelope for the EPS that can take it; with it clear the
    # legacy limits refuse the very first frame above upstream's 800 and the 12-count ramp
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self._reset_torque_driver_measurement(0)
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._torque_cmd_msg(TestMazdaSafety.MAX_RATE_UP + 1)))
    self._set_prev_torque(800)
    self.assertFalse(self._tx(self._torque_cmd_msg(801)))

  def _controller_loop(self, cc, cs, frames, driver_seen_by_controller, report_delay=1, report=True):
    """Run the real CarController through the compiled safety model, feeding the panda's
    rejection report back into CarState report_delay cycles after the refused frame (the report
    rides the can stream through pandad, one or two card cycles behind on the device). frames
    yields the driver torque the panda samples; the controller sees driver_seen_by_controller(frame)
    instead, so a stale sample can be staged. Returns (accepted torques by frame, longest run of
    rejected frames)."""
    from opendbc.car.mazda.tests.conftest import frame as tx_frame, step
    refused = deque([0] * report_delay, maxlen=report_delay)
    accepted, rejected_run, longest = [], 0, 0
    for i, driver_torque in enumerate(frames, start=1):
      self.safety.set_timer(i * 10_000)
      self._rx(self._torque_driver_msg(driver_torque))
      _, sends = step(cc, cs, long_active=False, enabled=True, lat_active=True, torque=1.0, v_ego=10.,
                      driver_torque=driver_seen_by_controller(i), lkas_rejected=refused[0] if report else 0)
      dat = tx_frame(sends, 0x243)
      torque = (((dat[0] & 0x0f) << 8) | dat[1]) - 2048
      if self._tx(libsafety_py.make_CANPacket(0x243, 0, dat)):
        accepted.append((i, torque))
        rejected_run = 0
        refused.append(0)
      else:
        rejected_run += 1
        longest = max(longest, rejected_run)
        refused.append(1 if torque != 0 else 0)
    return accepted, longest

  STALE_FRAMES = 8

  @classmethod
  def _stale_driver_sample(cls, frame):
    # what the controller sees of the -100 push on frames 61 to 72: nothing for eight frames
    return -100 if 60 + cls.STALE_FRAMES < frame <= 72 else 0

  def test_controller_recovers_the_stream_after_a_rejection(self):
    # A rejection zeroes the panda's rate-limit reference, so every later frame above one step
    # is rejected too and the EPS loses its stream: 1.72 s on route 00000148, 0.75 s on
    # 00000139, 0.63 s on drive_02, each followed by LKAS_FAULT and the camera fault. With the
    # panda's own report the controller restarts from zero as soon as it arrives.
    from opendbc.car.mazda.tests.conftest import car_controller, mazda_car_state
    for report_delay in (1, 2, 3):
      with self.subTest(report_delay=report_delay):
        cc = car_controller(alpha_long=False)
        cs = mazda_car_state(cc.CP, cc.CP_SP)
        self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, self.SAFETY_PARAM)
        self.safety.init_tests()
        self.safety.set_controls_allowed(True)
        # 60 frames ramping clean; the driver then pushes -100 for 12 frames, which the panda's
        # 6-sample window sees at once while the controller's sample runs 8 frames stale (the
        # route 00000148 staleness); then both agree again
        frames = [0] * 60 + [-100] * 12 + [0] * 120
        accepted, longest = self._controller_loop(cc, cs, frames, self._stale_driver_sample, report_delay)
        self.assertGreater(longest, 0, "the stale sample must reject at least one frame")
        # the restart lands one report behind the refusal, and each restart is refused again
        # while the controller's driver sample is still stale, so the outage is the staleness
        # plus the report delay: a sixth of the EPS's 0.6 s timeout at the slowest report
        self.assertLessEqual(longest, self.STALE_FRAMES + report_delay + 1)
        # and the ramp rebuilds to the rail afterwards
        self.assertEqual(accepted[-1][1], accepted[-2][1])
        self.assertGreater(accepted[-1][1], 500)

  def test_a_lone_rejection_costs_only_the_report_delay(self):
    # the same closed loop with the controller's sample fresh: the divergence is a single
    # frame, the panda's rate limits are exceeded by an outside push of the reference, and
    # the outage is exactly the time the report takes to come back
    from opendbc.car.mazda.tests.conftest import car_controller, mazda_car_state
    for report_delay in (1, 2, 3):
      with self.subTest(report_delay=report_delay):
        cc = car_controller(alpha_long=False)
        cs = mazda_car_state(cc.CP, cc.CP_SP)
        self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, self.SAFETY_PARAM)
        self.safety.init_tests()
        self.safety.set_controls_allowed(True)
        frames = [0] * 200
        # the panda's reference zeroed from outside after 60 clean frames (a disengage-and-arm
        # blip the controller never saw); the next command is far above one step
        self._controller_loop(cc, cs, frames[:60], lambda f: 0, report_delay)
        self._set_prev_torque(0)
        accepted, longest = self._controller_loop(cc, cs, frames[60:], lambda f: 0, report_delay)
        self.assertEqual(longest, report_delay)
        self.assertGreater(accepted[-1][1], 500)

  def test_without_the_rejection_report_a_rejection_starves_the_eps(self):
    # the same scenario with no report is the failure the captures show: rejected to the end
    from opendbc.car.mazda.tests.conftest import car_controller, mazda_car_state
    cc = car_controller(alpha_long=False)
    cs = mazda_car_state(cc.CP, cc.CP_SP)
    self.safety.set_controls_allowed(True)
    frames = [0] * 60 + [-100] * 12 + [0] * 120
    _, longest = self._controller_loop(cc, cs, frames, self._stale_driver_sample, report=False)
    self.assertGreaterEqual(longest, 60, "the EPS 0x243 timeout is about 60 frames")


class TestMazdaLegacyFwEpsSafety(TestMazdaSafety):
  """The same EPS hardware behind firmware that keeps the 45 kph floor (stock CX-9 2021, the
  older platforms, THACO CX-5 2023): MazdaSafetyFlags.LEGACY_FW_EPS selects the same measured
  envelope as the steer-to-zero bit."""

  SAFETY_PARAM = MazdaSafetyFlags.LEGACY_FW_EPS

  MAX_RATE_UP = 12
  MAX_RATE_DOWN = 12
  MAX_TORQUE_LOOKUP = [0], [1200]

  MAX_RT_DELTA = 384

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 15

  def test_legacy_bit_selects_the_steer_to_zero_envelope(self):
    for attr in ("MAX_RATE_UP", "MAX_RATE_DOWN", "MAX_TORQUE_LOOKUP", "MAX_RT_DELTA",
                 "DRIVER_TORQUE_ALLOWANCE", "DRIVER_TORQUE_FACTOR"):
      self.assertEqual(getattr(self, attr), getattr(TestMazdaSteerToZeroEpsSafety, attr), attr)


class TestMazdaLongitudinalSafety(TestMazdaSteerToZeroEpsSafety, common.LongitudinalAccelSafetyTest):
  """openpilot longitudinal is only offered on steer-to-zero EPS platforms, so LONG always
  travels with that bit."""

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0], [0x09d, 2], [0x21b, 0], [0x21c, 0], [0x499, 0],
             [0x361, 0], [0x362, 0], [0x363, 0], [0x364, 0], [0x365, 0], [0x366, 0], [0x764, 0],
             [0x21b, 2], [0x21c, 2], [0x499, 2], [0x361, 2], [0x362, 2], [0x363, 2], [0x364, 2], [0x365, 2], [0x366, 2]]

  SAFETY_PARAM = MazdaSafetyFlags.LONG | MazdaSafetyFlags.STEER_TO_ZERO_EPS

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, self.SAFETY_PARAM)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACTIVE": enable, "BRAKE_ON": 0}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _accel_msg(self, accel: float, bus: int = 0, active: bool = False):
    values = {"ACCEL_CMD": accel, "ACC_ACTIVE": active}
    return self.packer.make_can_msg_safety("CRZ_INFO", bus, values)

  def _crz_ctrl_cmd_msg(self, active: bool, bus: int = 0):
    values = {"CRZ_ACTIVE": active}
    return self.packer.make_can_msg_safety("CRZ_CTRL", bus, values)

  def _press_set(self):
    # arm the driver-intent qualifier the way every logged engagement does: a wheel press
    # lands 30-70 ms before PEDALS.ACC_ACTIVE rises
    self._rx(self._button_msg(set_m=True))

  def test_enable_control_allowed_from_cruise(self):
    # the common test plus the driver-intent qualifier this mode requires
    self._press_set()
    super().test_enable_control_allowed_from_cruise()

  def test_cruise_without_button_never_arms(self):
    # PEDALS.ACC_ACTIVE alone is the body answering our own fabricated frames; without a
    # SET/RES press heard from the wheel it must not arm controls
    self._rx(self._pcm_status_msg(False))
    for _ in range(12):
      self._rx(self._pcm_status_msg(True))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_button_window_expires(self):
    self._press_set()
    # 10 Hz CRZ_BTNS: run the countdown past the 1 s window with idle button frames
    for _ in range(12):
      self._rx(self._button_msg())
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_armed_controls_latch_past_the_window(self):
    self._press_set()
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    # the window expiring must not drop an active engagement
    for _ in range(12):
      self._rx(self._button_msg())
      self._rx(self._pcm_status_msg(True))
      self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_each_engage_button_arms(self):
    for btn in ("set_m", "set_p", "resume"):
      self._rx(self._button_msg(**{btn: True}))
      self._rx(self._pcm_status_msg(True))
      self.assertTrue(self.safety.get_controls_allowed(), btn)
      self._rx(self._pcm_status_msg(False))

  def test_cancel_button_exits_controls(self):
    self._press_set()
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    # the driver's cancel press always exits controls
    self._rx(self._button_msg(cancel=True))
    self.assertFalse(self.safety.get_controls_allowed())
    # ACC_ACTIVE alone does not re-arm without a fresh button press
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_camera_bus_accel_actuation_limits(self):
    # the synthetic radar frames are duplicated onto the camera bus; same limits apply there
    for accel in (self.MIN_ACCEL - 1, self.MIN_ACCEL, self.INACTIVE_ACCEL, self.MAX_ACCEL, self.MAX_ACCEL + 1):
      for controls_allowed in (True, False):
        self.safety.set_controls_allowed(controls_allowed)
        should_tx = controls_allowed and self.MIN_ACCEL <= accel <= self.MAX_ACCEL
        should_tx = should_tx or accel == self.INACTIVE_ACCEL
        self.assertEqual(should_tx, self._tx(self._accel_msg(accel, bus=2)))

  def test_stock_crz_info_standby_allowed(self):
    # every not-controlling stock pattern pegs the command field high: main-off standby and
    # both armed-idle variants (ACC_SET_ALLOWED follows the brake). All must pass byte-exactly,
    # checksum included, instead of being decoded as a huge accel command.
    def pegged_frame(d4, d5, counter):
      dat = bytes([0x01, 0xff, 0xe3, 0xff, d4, d5, counter])
      return dat + bytes([(0xff - sum(dat)) & 0xff])

    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for bus in (0, 2):
        for d4, d5 in ((0xc0, 0x00), (0xc0, 0x80), (0xc4, 0x80)):
          for counter in range(16):
            self.assertTrue(self._tx(common.make_msg(bus, 0x21b, 8, pegged_frame(d4, d5, counter))))

        bad_checksum = bytes.fromhex("01ffe3ffc0000000")
        self.assertFalse(self._tx(common.make_msg(bus, 0x21b, 8, bad_checksum)))
        # a pegged frame claiming ACC_ACTIVE must never ride the standby allowance
        self.assertFalse(self._tx(common.make_msg(bus, 0x21b, 8, pegged_frame(0xc6, 0x80, 0x00))))
        # and pegged with stop bits set is not a stock pattern either
        self.assertFalse(self._tx(common.make_msg(bus, 0x21b, 8, pegged_frame(0xc0, 0x84, 0x00))))

  def test_empty_radar_tracks_allowed(self):
    radar_messages = {
      0x499: bytes.fromhex("0008c00000000000"),
      0x361: bytes.fromhex("fff7fefe1fc00080"),
      0x362: bytes.fromhex("fff7fefe1fc78c80"),
      0x363: bytes.fromhex("fff7fefe1fc00000"),
      0x364: bytes.fromhex("fff7fefe1fc00000"),
      0x365: bytes.fromhex("fff7fe7ffbff3fc0"),
      0x366: bytes.fromhex("fff7fe7ffbff3fc0"),
    }

    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for bus in (0, 2):
        for addr, dat in radar_messages.items():
          self.assertTrue(self._tx(common.make_msg(bus, addr, 8, dat)))

  def test_g46l_radar_static_allowed(self):
    # the 2016.5 G46L body's own static capture; the 2022 one above is not its frame, and
    # the G46L never sends track messages at all
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for bus in (0, 2):
        self.assertTrue(self._tx(common.make_msg(bus, 0x499, 8, bytes.fromhex("0098400000000000"))))

    self.safety.set_controls_allowed(True)
    for bus in (0, 2):
      self.assertFalse(self._tx(common.make_msg(bus, 0x499, 8, bytes.fromhex("0098400100000000"))))

  def test_synthetic_lead_radar_track_allowed_disengaged(self):
    # Permit the measurement fields while requiring the occupied-track template. The slot is
    # perception, not actuation, so it remains valid with controls_allowed low like stock radar
    # reports objects with cruise off.
    lead_frames = [
      "0a4e00001c000000",  # stopped lead at 10.25 m
      "229e00007c00000e",  # lead at 34.56 m, closing slowly
      "22de00ff7c000004",  # lead at 34.81 m, opening slowly
      "000e00001c000000",  # zero range, zero relv corner (the template itself)
      "fffe00fffc00000f",  # max range, max relv corner
    ]
    for bus in (0, 2):
      for hexdat in lead_frames:
        dat = bytes.fromhex(hexdat)
        for controls_allowed in (False, True):
          self.safety.set_controls_allowed(controls_allowed)
          self.assertTrue(self._tx(common.make_msg(bus, 0x364, 8, dat)))

  def test_malformed_lead_radar_track_blocked(self):
    # each corrupts one template-owned field of a valid lead frame
    bad_frames = [
      "229100007c00000e",  # data[1] low nibble off the template
      "229e01007c00000e",  # data[2] not zero
      "229e00007d00000e",  # data[4] template bits wrong
      "229e00007cc0000e",  # data[5] wrong -- the retired capture's empty-slot signature
      "229e00007c00010e",  # data[6] not zero
      "229e00007c00100e",  # data[7] high nibble not zero
    ]
    self.safety.set_controls_allowed(True)
    for bus in (0, 2):
      for hexdat in bad_frames:
        self.assertFalse(self._tx(common.make_msg(bus, 0x364, 8, bytes.fromhex(hexdat))))

  def test_unexpected_radar_tracks_blocked(self):
    bad_messages = {
      0x499: bytes.fromhex("0008c00100000000"),
      0x361: bytes.fromhex("fff7fefe1fc00180"),
      0x362: bytes.fromhex("fff7fefe1fc00080"),
      0x363: bytes.fromhex("fff7fefe1fc00080"),
      0x364: bytes.fromhex("fff7fefe1fc00080"),
      0x365: bytes.fromhex("fff7fe7ffbff3f80"),
      0x366: bytes.fromhex("fff7fe7ffbff3f80"),
    }

    self.safety.set_controls_allowed(True)
    for bus in (0, 2):
      for addr, dat in bad_messages.items():
        self.assertFalse(self._tx(common.make_msg(bus, addr, 8, dat)))

  def test_radar_uds_allowlist(self):
    # tester present and session control only, main bus only
    self.assertTrue(self._tx(common.make_msg(0, 0x764, 8, bytes.fromhex("023e800000000000"))))
    self.assertTrue(self._tx(common.make_msg(0, 0x764, 8, bytes.fromhex("0210020000000000"))))
    self.assertFalse(self._tx(common.make_msg(0, 0x764, 8, bytes.fromhex("0210030000000000"))))
    self.assertFalse(self._tx(common.make_msg(0, 0x764, 8, bytes.fromhex("0227010000000000"))))
    self.assertFalse(self._tx(common.make_msg(2, 0x764, 8, bytes.fromhex("023e800000000000"))))

  def test_crz_ctrl_active_gated_on_controls(self):
    for bus in (0, 2):
      self.safety.set_controls_allowed(False)
      self.assertFalse(self._tx(self._crz_ctrl_cmd_msg(True, bus)))
      self.assertTrue(self._tx(self._crz_ctrl_cmd_msg(False, bus)))

      self.safety.set_controls_allowed(True)
      self.assertTrue(self._tx(self._crz_ctrl_cmd_msg(True, bus)))

  # a stock armed-idle CRZ_INFO standby frame, checksum-correct: what the controller emits
  # from the moment the radar teardown lands
  SYNTHETIC_CRZ_INFO_STANDBY = bytes.fromhex("01ffe3ffc000005d")

  def _acc_armed_msg(self, armed):
    # PEDALS with MRCC armed-but-idle (ACC_OFF), the state that persists across ignition
    values = {"ACC_OFF": armed, "BRAKE_ON": 0}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def test_acc_main_waits_for_the_radar_mastery_latch(self):
    # MADS uses acc_main_on's rising edge while software waits for stock-radar silence. panda
    # cannot receive stock CRZ_INFO because it goes stale at teardown, so it
    # mirrors the latch off the observable stand-in: our own first synthetic CRZ_INFO tx
    # (= the teardown landing) plus 1 s of the 50 Hz PEDALS clock. Both machines then arm on
    # the same frame; before that, MRCC-armed PEDALS must not raise acc_main_on, or the edge
    # is consumed at boot and the software's later MADS window transmits into rejections
    # that starve the EPS of 0x243.
    self.safety.set_mads_params(True, False, False)
    # boot: teardown not landed yet, MRCC main armed from the first frame
    for _ in range(120):
      self._rx(self._acc_armed_msg(True))
      self.assertFalse(self.safety.get_acc_main_on())
      self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._torque_cmd_msg(5)))
    # the teardown lands: the controller starts replaying the radar
    self.assertTrue(self._tx(common.make_msg(0, 0x21b, 8, self.SYNTHETIC_CRZ_INFO_STANDBY)))
    # the latch completes after 1 s of the 50 Hz PEDALS clock
    for _ in range(50):
      self.assertFalse(self.safety.get_acc_main_on())
      self._rx(self._acc_armed_msg(True))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._torque_cmd_msg(5)))

  def test_software_guard_is_derived_from_the_pandas(self):
    # values.py derives STOCK_RADAR_GUARD_T so the software's MADS edge trails this file's
    # latch; the derivation's panda term has to be this file's constant, not a copy of it
    import os
    import re
    import opendbc.safety
    header = open(os.path.join(os.path.dirname(opendbc.safety.__file__), "modes", "mazda.h")).read()
    silent_frames = int(re.search(r"#define MAZDA_RADAR_SILENT_FRAMES\s+(\d+)U", header).group(1))
    self.assertEqual(CarControllerParams.PANDA_RADAR_SILENT_T, silent_frames / 50.)  # PEDALS is 50 Hz
    self.assertGreater(CarControllerParams.STOCK_RADAR_GUARD_MARGIN_T, 0.)
    self.assertEqual(CarControllerParams.STOCK_RADAR_GUARD_T,
                     CarControllerParams.STOCK_RADAR_ALIVE_T + CarControllerParams.LONG_STEP * DT_CTRL +
                     CarControllerParams.PANDA_RADAR_SILENT_T + CarControllerParams.STOCK_RADAR_GUARD_MARGIN_T)

  def test_panda_arms_lateral_before_the_carstate_guard_lifts(self):
    # Both MADS machines arm off their own radar-silence guard, and the software's must complete
    # strictly AFTER the panda's: if the software arms first the controller ramps torque from
    # zero at STEER_DELTA_UP per frame into a panda that still rejects every 0x243, and when the
    # panda then arms, its rate limiter (desired_torque_last = 0) allows one step, rejects the
    # 36-84 counts by then commanded, resets, and keeps rejecting until the command falls back
    # under a step -- the 0x243 starvation that latched the camera fault on routes 116/117.
    # Same PEDALS frames to both machines; the stock CRZ_INFO only the software sees; our first
    # synthetic CRZ_INFO tx at the controller's latest possible frame (alive window + LONG_STEP).
    from opendbc.can import CANPacker
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.mazda import mazdacan
    from opendbc.car.mazda.carstate import STOCK_RADAR_ALIVE_FRAMES, STOCK_RADAR_GUARD_FRAMES
    from opendbc.car.mazda.interface import CarInterface
    self.safety.set_mads_params(True, False, False)
    CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], alpha_long=True, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], alpha_long=True,
                                       is_release_sp=False, docs=False)
    CI = CarInterface(CP, CP_SP)
    packer = CANPacker("mazda_2017")
    pedals = packer.make_can_msg("PEDALS", 0, {"ACC_OFF": 1})
    last_stock = 200  # 100 Hz control frames; the stock radar's last CRZ_INFO lands here
    first_tx = last_stock + STOCK_RADAR_ALIVE_FRAMES + CarControllerParams.LONG_STEP
    panda_armed_at = software_armed_at = None
    for i in range(last_stock + 3 * STOCK_RADAR_GUARD_FRAMES):
      msgs = []
      if i % 2 == 0:  # the 50 Hz PEDALS clock, MRCC main armed from the first frame
        self._rx(self._acc_armed_msg(True))
        msgs.append(pedals)
        if i <= last_stock:
          msgs.append(mazdacan.create_acc_command(packer, 0, i // 2, 0., long_active=False, acc_available=True))
      # the radar-bus health witnesses include ENGINE_DATA (0x202); without it the software
      # guard treats the bus as disconnected and never counts silence
      msgs.append((0x202, b"\x00" * 8, 0))
      ret, _ = CI.update([(int(i * DT_CTRL * 1e9), [(m[0], m[1], m[2]) for m in msgs])])
      if i == first_tx:
        self.assertTrue(self._tx(common.make_msg(0, 0x21b, 8, self.SYNTHETIC_CRZ_INFO_STANDBY)))
        # The controller claims replacement traffic with the first synthetic frame and sets
        # CS.radar_control_active on its next update; emulate the claim (this loop never runs
        # the controller, so without it the software guard can never complete).
        CI.CS.radar_control_active = True
      if panda_armed_at is None and self.safety.get_controls_allowed_lateral():
        panda_armed_at = i
      if software_armed_at is None and ret.cruiseState.available:
        software_armed_at = i
    self.assertIsNotNone(panda_armed_at)
    self.assertIsNotNone(software_armed_at)
    self.assertGreater(software_armed_at, panda_armed_at, msg="the software armed MADS before the panda would accept torque")
    # by roughly the margin values.py budgets for PEDALS jitter and pipeline latency
    margin_frames = int(CarControllerParams.STOCK_RADAR_GUARD_MARGIN_T / DT_CTRL)
    self.assertGreaterEqual(software_armed_at - panda_armed_at, margin_frames - 2)
    # and the panda's edge has not been consumed by the time the software arrives
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._torque_cmd_msg(5)))

  def test_camera_bus_radar_tx_does_not_master(self):
    # only the main-bus replay marks mastery; the camera-bus copy is a duplicate
    self.safety.set_mads_params(True, False, False)
    self.assertTrue(self._tx(common.make_msg(2, 0x21b, 8, self.SYNTHETIC_CRZ_INFO_STANDBY)))
    for _ in range(60):
      self._rx(self._acc_armed_msg(True))
    self.assertFalse(self.safety.get_acc_main_on())

  def test_acc_main_follows_armed_state_after_the_latch(self):
    # after the latch, acc_main_on tracks PEDALS arming both ways (main off must still exit)
    self.safety.set_mads_params(True, False, False)
    self.assertTrue(self._tx(common.make_msg(0, 0x21b, 8, self.SYNTHETIC_CRZ_INFO_STANDBY)))
    for _ in range(60):
      self._rx(self._acc_armed_msg(True))
    self.assertTrue(self.safety.get_acc_main_on())
    self._rx(self._acc_armed_msg(False))
    self.assertFalse(self.safety.get_acc_main_on())
    self._rx(self._acc_armed_msg(True))
    self.assertTrue(self.safety.get_acc_main_on())

  def _pedals_msg(self, armed, brake):
    values = {"ACC_OFF": armed, "BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _armed_and_latched(self):
    self.safety.set_mads_params(True, False, False)
    self.assertTrue(self._tx(common.make_msg(0, 0x21b, 8, self.SYNTHETIC_CRZ_INFO_STANDBY)))
    for _ in range(60):
      self._rx(self._acc_armed_msg(True))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_brake_only_dropout_holds_main(self):
    # carstate holds cruise_available through a both-low PEDALS sample under braking; the
    # panda must hold too, or MADS exits on the panda alone and the software steers into
    # rejections
    self._armed_and_latched()
    for _ in range(10):
      self._rx(self._pedals_msg(armed=True, brake=True))
    for _ in range(100):
      self._rx(self._pedals_msg(armed=False, brake=True))
      self.assertTrue(self.safety.get_acc_main_on())
      self.assertTrue(self.safety.get_controls_allowed_lateral())
    # the dropout lands once the brake is free
    self._rx(self._pedals_msg(armed=False, brake=False))
    self._rx(self._pedals_msg(armed=False, brake=False))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_cancel_lands_through_the_brake(self):
    # route 000001c9--0b2a64a214 seg 0: main toggled at a red light with the brake held. The
    # software's cancel context let its main fall; the panda's held, so the next main press had
    # no rising edge, lateral never re-armed, and MADS ran into Controls Mismatch: Lateral
    self._armed_and_latched()
    for _ in range(10):
      self._rx(self._pedals_msg(armed=True, brake=True))
    self._rx(self._button_msg(cancel=True))
    self._rx(self._pedals_msg(armed=False, brake=True))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    # main again with the foot still on the brake: a real rising edge, lateral re-arms
    self._rx(self._button_msg())
    for _ in range(10):
      self._rx(self._pedals_msg(armed=False, brake=True))
    self._rx(self._pedals_msg(armed=True, brake=True))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_cancel_context_outlives_the_press(self):
    # PEDALS trails the button: the bits can drop after the button is back up
    self._armed_and_latched()
    self._rx(self._button_msg(cancel=True))
    self._rx(self._button_msg())
    for _ in range(20):
      self._rx(self._pedals_msg(armed=True, brake=True))
    self.assertTrue(self.safety.get_acc_main_on())
    self._rx(self._pedals_msg(armed=False, brake=True))
    self.assertFalse(self.safety.get_acc_main_on())

  def test_cancel_context_expires(self):
    # past the window a both-low sample under braking is a dropout again
    self._armed_and_latched()
    self._rx(self._button_msg(cancel=True))
    self._rx(self._button_msg())
    for _ in range(25):
      self._rx(self._pedals_msg(armed=True, brake=True))
    self._rx(self._pedals_msg(armed=False, brake=True))
    self.assertTrue(self.safety.get_acc_main_on())

  def test_cancel_context_is_derived_from_the_software(self):
    import os
    import re
    import opendbc.safety
    header = open(os.path.join(os.path.dirname(opendbc.safety.__file__), "modes", "mazda.h")).read()
    frames = int(re.search(r"#define MAZDA_CANCEL_CONTEXT_FRAMES\s+(\d+)U", header).group(1))
    self.assertEqual(CarControllerParams.CANCEL_CONTEXT_T, frames / 50.)  # PEDALS is 50 Hz

  def test_crz_info_active_gated_on_controls(self):
    # ACC_ACTIVE mirrors CRZ_CTRL's gate: an engaged-claiming accel frame must not flow while
    # controls are not allowed. The body raises PEDALS.ACC_ACTIVE off the SET press before
    # our first engaged frame in every logged engagement, so there is no deadlock.
    for bus in (0, 2):
      for active in (False, True):
        msg = self._accel_msg(self.INACTIVE_ACCEL, bus=bus, active=active)
        self.safety.set_controls_allowed(False)
        self.assertEqual(not active, self._tx(msg))
        self.safety.set_controls_allowed(True)
        self.assertTrue(self._tx(msg))


class TestMazdaTjaMads(unittest.TestCase):
  """The physical TJA button as the MADS lateral switch, declared by the driver.

  The button is fitted to some trims only and neither MAZDA_CX5_2022 nor MAZDA_CX9_2021
  predicts it, so a sunnypilot safety param carries the driver's declaration. Declared, bit 11
  drives the MADS button and MRCC no longer touches the main edge in either direction: its
  falling edge would otherwise exit the panda's lateral while the software's MADS stays on.
  Undeclared cars keep the MRCC-derived main edge and bit 11 is ignored.
  """

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self._init(tja_button=False)

  def _init(self, tja_button, param=0):
    self.safety.set_current_safety_param_sp(MazdaSafetyFlagsSP.TJA_BUTTON if tja_button else 0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, param)
    self.safety.init_tests()
    self.safety.set_mads_params(True, False, False)

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)
    self.safety.set_mads_params(False, False, False)

  def _btns(self, tja=False):
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": tja})

  def _crz_ctrl(self, main_on):
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, {"CRZ_AVAILABLE": main_on})

  def _pedals(self, acc_off):
    return self.packer.make_can_msg_safety("PEDALS", 0, {"ACC_OFF": acc_off})

  def test_undeclared_keeps_mrcc_path_and_ignores_the_bit(self):
    self.safety.safety_rx_hook(self._btns(True))
    self.safety.safety_rx_hook(self._btns(False))
    self.assertEqual(-1, self.safety.get_mads_button_press())  # UNAVAILABLE
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.safety.safety_rx_hook(self._crz_ctrl(False))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_declared_button_allows_lateral_without_mrcc(self):
    self._init(tja_button=True)
    self.safety.safety_rx_hook(self._btns(False))
    self.assertEqual(0, self.safety.get_mads_button_press())  # NOT_PRESSED
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    self.safety.safety_rx_hook(self._btns(True))
    self.assertEqual(1, self.safety.get_mads_button_press())  # PRESSED
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    self.safety.safety_rx_hook(self._btns(False))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_declared_button_mrcc_does_not_drive_the_main_edge(self):
    self._init(tja_button=True)
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    self.safety.safety_rx_hook(self._btns(True))
    self.safety.safety_rx_hook(self._btns(False))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # the MRCC master button (route 00000018 seg 12) disarms MRCC and the camera, not MADS:
    # no falling edge here, so the panda's lateral stays with the software's
    self.safety.safety_rx_hook(self._crz_ctrl(False))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_declared_button_under_openpilot_longitudinal(self):
    # the PEDALS-derived main edge is guarded the same way
    self._init(tja_button=True, param=MazdaSafetyFlags.LONG | MazdaSafetyFlags.STEER_TO_ZERO_EPS)
    self.safety.safety_rx_hook(self._pedals(True))
    self.assertFalse(self.safety.get_acc_main_on())
    self.safety.safety_rx_hook(self._btns(True))
    self.safety.safety_rx_hook(self._btns(False))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.safety.safety_rx_hook(self._pedals(False))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_declaration_is_read_at_init(self):
    self._init(tja_button=True)
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.assertFalse(self.safety.get_acc_main_on())

    self._init(tja_button=False)
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.assertTrue(self.safety.get_acc_main_on())


class MazdaTorqueInterceptorSafetyMixin:
  """TI command-path tests. The generic 0x243 envelope tests are inherited unchanged from the
  STZ/long parents (their SAFETY_PARAM bits select the same envelope here); this mixin adds the
  0x249 path: TI panda limits 600/12/25/384 with driver 15/40 are exercised functionally below
  (601 rejection, +13 rate rejection, 384 RT window, driver-bound rejection). The CX-8's tighter
  DELTA_DOWN (15 vs the panda's 25) is strictly inside the panda envelope, never outside."""
  TI_PARAM: int

  def _reset_ti_safety(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, self.TI_PARAM)
    self.safety.init_tests()

  def _enable_ti(self, torque: int = 0):
    for _ in range(6):
      self.assertTrue(self._rx(ti_feedback(torque)))
    self.safety.set_controls_allowed(True)

  def _torque_driver_msg(self, torque):
    return ti_feedback(torque)

  def test_reset_driver_torque_measurements(self):
    for torque in (-85, 85):
      for _ in range(6):
        self.assertTrue(self._rx(ti_feedback(torque)))
    self.assertNotEqual(self.safety.get_torque_driver_min(), 0)
    self.assertNotEqual(self.safety.get_torque_driver_max(), 0)
    self._reset_ti_safety()
    self.assertEqual(self.safety.get_torque_driver_min(), 0)
    self.assertEqual(self.safety.get_torque_driver_max(), 0)

  def test_ti_requires_healthy_fresh_feedback(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(ti_command(6)))
    self._enable_ti()
    self.assertTrue(self._tx(ti_command(6)))
    self.safety.set_timer(40_000)
    self.assertTrue(self._tx(ti_command(12)))
    self.safety.set_timer(40_001)
    self.assertFalse(self._tx(ti_command(18)))
    self.assertTrue(self._tx(ti_command(0)))
    self.assertTrue(self._rx(ti_feedback()))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(ti_command(6)))

  def test_ti_feedback_semantics_and_recovery(self):
    # integrity rejections: unknown version bytes — not a frame from a known TI,
    # so the rx check fails (this is what flags safetyRxChecksInvalid)
    for fields in ({"version": 0}, {"version": 2}, {"version": 17}):
      with self.subTest(fields=fields):
        self._reset_ti_safety()
        self._enable_ti()
        self.assertFalse(self._rx(ti_feedback(**fields)))
        self.assertFalse(self._tx(ti_command(6)))
        self.assertTrue(self._rx(ti_feedback()))
        self.safety.set_controls_allowed(True)
        self.assertTrue(self._tx(ti_command(6)))

    # health rejections: OFF/DISCOVER/DRIVER_OVER and fault-byte frames are
    # legitimate TI states (boot, standstill self-protection) — the frame itself
    # is valid and must NOT poison the rx check, but torque stays blocked
    for fields in ({"state": 0}, {"state": 1}, {"state": 2},
                   {"violation": 1}, {"violation": 0x11}, {"error": 1}, {"ramp_down": 1}):
      with self.subTest(fields=fields):
        self._reset_ti_safety()
        self._enable_ti()
        self.assertTrue(self._rx(ti_feedback(**fields)))
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._tx(ti_command(6)))
        self.assertTrue(self._rx(ti_feedback()))
        self.assertTrue(self._tx(ti_command(6)))

  def test_ti_accepts_current_firmware_version_byte(self):
    # current TI firmware reports version byte 0x10; captured healthy frame 7f7f100300000030
    self._reset_ti_safety()
    self._enable_ti()
    self.assertTrue(self._rx(ti_feedback(version=0x10)))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(ti_command(6)))

  def test_ti_wrong_feedback_bus_or_length_cannot_enable(self):
    for msg in (ti_feedback(bus=0), ti_feedback(bus=2), ti_feedback(length=7)):
      with self.subTest(bus=msg.bus, length=msg.data_len_code):
        self._reset_ti_safety()
        self.safety.set_controls_allowed(True)
        self._rx(msg)
        self.assertFalse(self._tx(ti_command(6)))

  def test_ti_command_structure(self):
    for msg in (
      ti_command(6, duplicate=5), ti_command(6, key=0xC461CE61),
      ti_command(6, reserved_request=0xA0), ti_command(6, reserved_duplicate=0x50),
      ti_command(6, bus=0), ti_command(6, bus=2), ti_command(6, length=7),
    ):
      with self.subTest(bus=msg.bus, length=msg.data_len_code):
        self._reset_ti_safety()
        self._enable_ti()
        self.assertFalse(self._tx(msg))

  def test_ti_controls_disabled(self):
    self.assertTrue(self._rx(ti_feedback()))
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(ti_command(6)))
    self.assertTrue(self._tx(ti_command(0)))

  def test_ti_rate_and_absolute_limits(self):
    self._enable_ti()
    timer = 0
    for torque in range(12, 601, 12):
      if torque > 12 and torque % 384 == 12:
        timer += 250_001
        self.safety.set_timer(timer)
        self.assertTrue(self._rx(ti_feedback()))
        self.assertTrue(self._tx(ti_command(torque - 12)))
      self.assertTrue(self._tx(ti_command(torque)))
    self.assertFalse(self._tx(ti_command(601)))

    self._reset_ti_safety()
    self._enable_ti()
    self.assertTrue(self._tx(ti_command(12)))
    self.assertFalse(self._tx(ti_command(25)))

    self._reset_ti_safety()
    self._enable_ti()
    for torque in range(12, 61, 12):
      self.assertTrue(self._tx(ti_command(torque)))
    self.assertTrue(self._tx(ti_command(35)))
    self.assertFalse(self._tx(ti_command(-13)))

  def test_ti_driver_and_realtime_limits(self):
    self._enable_ti(-30)
    self.assertFalse(self._tx(ti_command(12)))

    self._reset_ti_safety()
    self._enable_ti()
    for torque in range(12, 385, 12):
      self.assertTrue(self._tx(ti_command(torque)))
    self.assertFalse(self._tx(ti_command(396)))

    self._reset_ti_safety()
    self._enable_ti()
    for torque in range(12, 385, 12):
      self.assertTrue(self._tx(ti_command(torque)))
    self.safety.set_timer(250_001)
    self.assertTrue(self._rx(ti_feedback()))
    self.assertTrue(self._tx(ti_command(384)))
    self.assertTrue(self._tx(ti_command(396)))

  def test_stock_and_ti_torque_histories_are_independent(self):
    self._enable_ti()
    stock = self.packer.make_can_msg_safety("CAM_LKAS", 0, {"LKAS_REQUEST": 12})
    for ti_torque in range(6, 31, 6):
      self.assertTrue(self._tx(stock))
      self.assertTrue(self._tx(ti_command(ti_torque)))


class TestMazdaTorqueInterceptorSafety(MazdaTorqueInterceptorSafetyMixin, TestMazdaSteerToZeroEpsSafety):
  TI_PARAM = MazdaSafetyFlags.STEER_TO_ZERO_EPS | MazdaSafetyFlags.TORQUE_INTERCEPTOR
  SAFETY_PARAM = TI_PARAM
  TX_MSGS = TestMazdaSteerToZeroEpsSafety.TX_MSGS + [[0x249, 1]]

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self._reset_ti_safety()

  def test_ti_torque_only_while_openpilot_owns_lkas(self):
    # One torque master: the TI drives exactly when openpilot owns the LKAS addresses
    # (the same predicate the camera-LKAS forward block uses). The MADS-pause hole
    # (mads on, cruise on, lateral off) forwards the camera's 0x243, so the TI must
    # stay silent there or two torque masters overlap. Pins the Gilfoyle finding.
    for mads in (False, True):
      self.safety.set_mads_params(mads, False, False)
      for controls_allowed, controls_allowed_lateral in [(False, False), (True, False), (False, True), (True, True)]:
        self._reset_ti_safety()
        self.safety.set_mads_params(mads, False, False)
        for _ in range(6):
          self.assertTrue(self._rx(ti_feedback()))
        self.safety.set_controls_allowed(controls_allowed)
        self.safety.set_controls_allowed_lateral(controls_allowed_lateral)
        op_controlling = controls_allowed_lateral or (controls_allowed and not mads)
        self.assertEqual(op_controlling, bool(self.safety.safety_tx_hook(ti_command(6, bus=1))),
                         f"{mads=} {controls_allowed=} {controls_allowed_lateral=}")
    self.safety.set_mads_params(False, False, False)


class TestMazdaLongitudinalTorqueInterceptorSafety(MazdaTorqueInterceptorSafetyMixin, TestMazdaLongitudinalSafety):
  TI_PARAM = MazdaSafetyFlags.LONG | MazdaSafetyFlags.STEER_TO_ZERO_EPS | MazdaSafetyFlags.TORQUE_INTERCEPTOR
  SAFETY_PARAM = TI_PARAM
  TX_MSGS = TestMazdaLongitudinalSafety.TX_MSGS + [[0x249, 1]]

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self._reset_ti_safety()

  def test_longitudinal_controls_remain_enabled(self):
    standby = bytes.fromhex("01ffe3ffc000005d")
    self.assertTrue(self._tx(make_msg(0, 0x21B, dat=standby)))
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("CRZ_CTRL", 0, {"CRZ_ACTIVE": True})))


class TestMazdaIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def _msg(self, byte0):
    return make_msg(0, 0x9E, dat=bytes([byte0]) + b"\x00" * 7)

  # 0x9E byte 0 high 3 bits == 6 (0xC0)
  def test_ignition_on(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0x20))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
