#!/usr/bin/env python3
import unittest

from opendbc.car.mazda.values import MazdaSafetyFlags
from opendbc.car.structs import CarParams
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


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  FWD_BLACKLISTED_ADDRS = {0: [0x249], 2: [0x243, 0x249, 0x440]}

  MAX_RATE_UP = 12
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [1200]

  MAX_RT_DELTA = 384

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 15

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, values)

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

  def _button_msg(self, resume=False, cancel=False, set_m=False, set_p=False):
    values = {
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
      "SET_M": set_m,
      "SET_M_INV": (set_m + 1) % 2,
      "SET_P": set_p,
      "SET_P_INV": (set_p + 1) % 2,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))


class TestMazdaLongitudinalSafety(TestMazdaSafety, common.LongitudinalAccelSafetyTest):

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0], [0x21b, 0], [0x21c, 0], [0x499, 0],
             [0x361, 0], [0x362, 0], [0x363, 0], [0x364, 0], [0x365, 0], [0x366, 0], [0x764, 0],
             [0x21b, 2], [0x21c, 2], [0x499, 2], [0x361, 2], [0x362, 2], [0x363, 2], [0x364, 2], [0x365, 2], [0x366, 2]]

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, MazdaSafetyFlags.LONG)
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

  def test_synthetic_lead_radar_track_allowed_disengaged(self):
    # DIST_OBJ and RELV_OBJ are free fields; the template bytes must match. The non-template
    # frames are real on-road emissions (route 6bb2dc61c4), which a byte-exact check silently
    # dropped -- 982 asked, 0 transmitted -- starving the camera of the track. The slot is
    # perception, not actuation, so it flows with controls_allowed low the way a stock radar
    # reports objects with cruise off.
    lead_frames = [
      "0a4000001dc00000",  # the fabricated stopped lead at 10.25 m
      "229000007dc0000e",  # lead at 34.56 m, closing slowly
      "22d000ff7dc00004",  # lead at 34.81 m, opening slowly
      "000000001dc00000",  # zero range, zero relv corner
      "fff000fffdc0000f",  # max range, max relv corner
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
      "229100007dc0000e",  # data[1] low nibble not zero
      "229001007dc0000e",  # data[2] not zero
      "229000007cc0000e",  # data[4] template bits wrong
      "229000007dc1000e",  # data[5] wrong
      "229000007dc0010e",  # data[6] not zero
      "229000007dc0100e",  # data[7] high nibble not zero
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


class MazdaTorqueInterceptorSafetyMixin:
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


class TestMazdaTorqueInterceptorSafety(MazdaTorqueInterceptorSafetyMixin, TestMazdaSafety):
  TI_PARAM = MazdaSafetyFlags.TORQUE_INTERCEPTOR
  TX_MSGS = TestMazdaSafety.TX_MSGS + [[0x249, 1]]

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self._reset_ti_safety()


class TestMazdaLongitudinalTorqueInterceptorSafety(MazdaTorqueInterceptorSafetyMixin, TestMazdaLongitudinalSafety):
  TI_PARAM = MazdaSafetyFlags.LONG | MazdaSafetyFlags.TORQUE_INTERCEPTOR
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
