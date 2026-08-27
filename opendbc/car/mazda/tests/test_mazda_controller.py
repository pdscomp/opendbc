#!/usr/bin/env python3
"""Tests for the Mazda CX-5 2022+ EPS steering parameters (gated on the EPS, not the model)
and the longitudinal message builders and standstill hold."""

from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.longitudinal import (BREAKAWAY_FRAMES, LEAD_DEBOUNCE_FRAMES, RADAR_SESSION_LIMIT_FRAMES,
                                            RELEASE_DEBOUNCE_FRAMES, RESUME_UNLATCH_LATCHED_FRAMES,
                                            AdvertisedLead, RadarSessionManager, RadarSessionState, StandstillHold)
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, CarControllerParams, MazdaFlags, TorqueInterceptorControllerParams


class TestCarControllerParams:

  @pytest.fixture
  def cx5_2022_params(self):
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX5_2022
      flags = MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO
    return CarControllerParams(FakeCP())

  @pytest.fixture
  def eps_swap_params(self):
    # A CX-5 2022+ EPS swapped into (or shared by) another Mazda: different model, same EPS.
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX9_2021
      flags = MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO
    return CarControllerParams(FakeCP())

  def test_eps_ceiling_never_exceeds_steer_max_scale(self, cx5_2022_params):
    # The ceiling is a clamp on delivered-torque counts; the scale is STEER_MAX. The clamp is
    # only meaningful if it sits at or below the scale at every speed.
    bp, vals = cx5_2022_params.EPS_CEILING_LOOKUP
    for v in np.arange(0.0, 40.0, 0.25):
      ceiling = np.interp(v, bp, vals)
      steer_max = np.interp(v, cx5_2022_params.STEER_MAX_LOOKUP[0],
                            cx5_2022_params.STEER_MAX_LOOKUP[1])
      assert 0 < ceiling <= steer_max, f"ceiling {ceiling} vs steer_max {steer_max} at {v} m/s"

  def test_eps_ceiling_is_monotone_and_matches_the_measured_rails(self, cx5_2022_params):
    # Measured over 11.4M clean frames: 1148 below 18 mph, a monotone rolloff, hard 620 from
    # 32.5 mph up (docs/mazda-lkas-camera-tx-census.md). Nothing above 620 was ever delivered
    # above 32.5 mph in 7.5M frames, so the high-speed leg must not drift back up.
    bp, vals = cx5_2022_params.EPS_CEILING_LOOKUP
    assert list(vals) == sorted(vals, reverse=True), "ceiling must fall monotonically with speed"
    assert np.interp(5.0, bp, vals) == 1148
    assert np.interp(14.5, bp, vals) == 620
    assert np.interp(35.0, bp, vals) == 620

  def test_steer_delta_matches_the_eps_rate_limit_at_this_steer_step(self, cx5_2022_params):
    # The EPS rate limit is per unit TIME (~1200 units/s), while STEER_DELTA_UP/DOWN are per
    # frame, so the two are only matched at STEER_STEP = 1. Changing one without the other
    # silently rescales the commanded slew rate. Both directions: the measured rail is
    # symmetric (p99 and p99.9 of the delivered step are 12 either way), and a winddown above
    # it only lets the command run ahead of the wheel.
    rate_hz = 1.0 / DT_CTRL / CarControllerParams.STEER_STEP
    assert cx5_2022_params.STEER_DELTA_UP * rate_hz == pytest.approx(1200, rel=0.01)
    assert cx5_2022_params.STEER_DELTA_DOWN * rate_hz == pytest.approx(1200, rel=0.01)

  @pytest.fixture
  def pre_2022_params(self):
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX5
      flags = MazdaFlags.GEN1
    return CarControllerParams(FakeCP())

  def test_cx5_2022_has_lookup(self, cx5_2022_params):
    assert hasattr(cx5_2022_params, 'STEER_MAX_LOOKUP')
    assert cx5_2022_params.STEER_MAX == 1200

  def test_cx5_2022_low_speed(self, cx5_2022_params):
    p = cx5_2022_params
    for v in [0.0, 5.0, 10.0, 14.2]:
      sm = round(float(np.interp(v, p.STEER_MAX_LOOKUP[0], p.STEER_MAX_LOOKUP[1])))
      assert sm == 1200

  def test_cx5_2022_high_speed(self, cx5_2022_params):
    p = cx5_2022_params
    for v in [14.5, 20.0, 30.0]:
      sm = round(float(np.interp(v, p.STEER_MAX_LOOKUP[0], p.STEER_MAX_LOOKUP[1])))
      assert sm == 800

  def test_cx5_2022_rate_limits(self, cx5_2022_params):
    assert cx5_2022_params.STEER_DELTA_UP == 12
    assert cx5_2022_params.STEER_DELTA_DOWN == 12

  def test_cx5_2022_winddown_stays_within_the_panda_backstop(self, cx5_2022_params):
    # opendbc/safety/modes/mazda.h declares max_rate_down = 25 for every Mazda; commanding a
    # tighter winddown is allowed, commanding a looser one would be blocked frame by frame.
    assert cx5_2022_params.STEER_DELTA_DOWN <= 25

  def test_cx5_eps_driver_multiplier(self, cx5_2022_params):
    # 15 is the CX-5-EPS tune (upstream stock is 1)
    assert cx5_2022_params.STEER_DRIVER_MULTIPLIER == 15

  def test_eps_swap_gets_cx5_tune(self, eps_swap_params):
    # EPS capability on a non-CX-5 model still gets the higher-authority tune
    assert eps_swap_params.STEER_MAX == 1200
    assert eps_swap_params.STEER_DRIVER_MULTIPLIER == 15
    assert hasattr(eps_swap_params, 'STEER_MAX_LOOKUP')

  def test_no_eps_no_lookup(self, pre_2022_params):
    assert not hasattr(pre_2022_params, 'STEER_MAX_LOOKUP')
    assert pre_2022_params.STEER_MAX == 800
    assert pre_2022_params.STEER_DRIVER_MULTIPLIER == 1

  def test_ti_speed_override_does_not_select_high_authority_stock_tune(self):
    params = CarControllerParams(SimpleNamespace(flags=MazdaFlags.GEN1 | MazdaFlags.TORQUE_INTERCEPTOR))
    assert params.STEER_MAX == 800
    assert not hasattr(params, 'STEER_MAX_LOOKUP')

  def test_steer_to_zero_eps_keeps_high_authority_tune_with_ti(self):
    flags = MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO | MazdaFlags.TORQUE_INTERCEPTOR
    params = CarControllerParams(SimpleNamespace(flags=flags))
    assert params.STEER_MAX == 1200
    assert hasattr(params, 'STEER_MAX_LOOKUP')


def _steering_controller(*, ti=False, steer_to_zero=False):
  CP = CarInterface.get_params(CAR.MAZDA_CX5, {0: {}, 1: {}, 2: {}}, [], alpha_long=False,
                               is_release=False, docs=False)
  if ti:
    CP.flags |= MazdaFlags.TORQUE_INTERCEPTOR.value
  if steer_to_zero:
    CP.flags |= MazdaFlags.STEER_TO_ZERO.value
  controller = CarController({Bus.pt: "mazda_2017"}, CP, structs.CarParamsSP())
  controller.frame = 1  # HUD output is unrelated to these steering checks
  return controller


def _steering_state(*, healthy=True, driver_torque=0, speed=5.):  # above the TI standstill gate
  return SimpleNamespace(
    out=SimpleNamespace(vEgoRaw=speed, steeringTorque=driver_torque, brakePressed=False),
    ti_lkas_allowed=healthy,
    crz_btns_counter=0,
    cancel_button=1,  # suppress unrelated ICBM output
    lkas_allowed_speed=True,
    cam_lkas={"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0},
    cam_laneinfo={s: 0 for s in (
      "LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT2", "BIT3", "NO_ERR_BIT", "ERR_BIT", "S1", "S1_HBEAM",
    )},
  )


def _steering_step(controller, state, torque=1., lat_active=True, now_nanos=0):
  control = structs.CarControl()
  control.latActive = lat_active
  control.actuators.torque = torque
  _, sends = controller.update(control.as_reader(), structs.CarControlSP(), state, now_nanos)
  return sends


def _command_torque(sends, address):
  dat = next(dat for addr, dat, _ in sends if addr == address)
  return (((dat[0] & 0xf) << 8) | dat[1]) - 2048


class TestMazdaTorqueInterceptorController:
  def test_standstill_zeroes_ti_command(self):
    controller = _steering_controller(ti=True)
    state = _steering_state(speed=0.)
    for frame in range(50):
      sends = _steering_step(controller, state, now_nanos=frame * 10_000_000)
    assert _command_torque(sends, 0x249) == 0
    state.out.vEgoRaw = 5.  # above the 1 m/s standstill gate
    for frame in range(50, 200):
      sends = _steering_step(controller, state, now_nanos=frame * 10_000_000)
    assert _command_torque(sends, 0x249) == 600

  @pytest.mark.parametrize(("torque", "expected"), [
    (-600, "05a805a8c461ce60"),
    (0, "08000800c461ce60"),
    (600, "0a580a58c461ce60"),
  ])
  def test_canonical_command_vectors_have_duplicate_request_key_and_no_counter(self, torque, expected):
    msg = mazdacan.create_ti_steering_control(CANPacker("mazda_2017"), torque)
    assert msg == (0x249, bytes.fromhex(expected), 1)
    assert mazdacan.create_ti_steering_control(CANPacker("mazda_2017"), torque) == msg

  def test_stock_output_is_unchanged_and_ti_adds_an_independent_command(self):
    state = _steering_state()
    stock_sends = _steering_step(_steering_controller(), state)
    ti_sends = _steering_step(_steering_controller(ti=True), state)
    assert not any(addr == 0x249 for addr, _, _ in stock_sends)
    assert next(msg for msg in stock_sends if msg[0] == 0x243) == next(msg for msg in ti_sends if msg[0] == 0x243)
    assert _command_torque(ti_sends, 0x249) == TorqueInterceptorControllerParams.STEER_DELTA_UP

  @pytest.mark.parametrize(("healthy", "lat_active", "expected"), [
    (True, True, 6),
    (False, True, 0),
    (True, False, 0),
  ])
  def test_ti_health_and_lateral_request_gate_torque(self, healthy, lat_active, expected):
    sends = _steering_step(_steering_controller(ti=True), _steering_state(healthy=healthy), lat_active=lat_active)
    assert _command_torque(sends, 0x249) == expected

  @pytest.mark.parametrize(("steer_to_zero", "stock_max"), [(False, 800), (True, 1200)])
  def test_stock_and_ti_authority_are_independent(self, steer_to_zero, stock_max):
    controller = _steering_controller(ti=True, steer_to_zero=steer_to_zero)
    state = _steering_state()
    for frame in range(119):
      _steering_step(controller, state, now_nanos=frame * 10_000_000)
    sends = _steering_step(controller, state, now_nanos=1_190_000_000)
    # Stock authority saturates at STEER_MAX, further clamped by the EPS ceiling lookup at
    # this speed (1148 below 8.5 m/s) — the ceiling is upstream's, the scale is ours.
    ceiling = stock_max
    if hasattr(controller.params, 'EPS_CEILING_LOOKUP'):
      ceiling = round(float(np.interp(state.out.vEgoRaw, controller.params.EPS_CEILING_LOOKUP[0],
                                      controller.params.EPS_CEILING_LOOKUP[1])))
    assert _command_torque(sends, 0x243) == min(stock_max, ceiling)
    assert _command_torque(sends, 0x249) == 600

  def test_ti_history_does_not_reuse_stock_history(self):
    controller = _steering_controller(ti=True)
    state = _steering_state(healthy=False)
    for frame in range(19):
      _steering_step(controller, state, now_nanos=frame * 10_000_000)
    sends = _steering_step(controller, state, now_nanos=190_000_000)
    assert _command_torque(sends, 0x243) == 200
    assert _command_torque(sends, 0x249) == 0

    state.ti_lkas_allowed = True
    sends = _steering_step(controller, state, now_nanos=200_000_000)
    assert _command_torque(sends, 0x249) == 6

  def test_ti_rate_and_driver_limits(self):
    controller = _steering_controller(ti=True)
    state = _steering_state()
    assert _command_torque(_steering_step(controller, state), 0x249) == 6
    assert _command_torque(_steering_step(controller, state), 0x249) == 12
    for _ in range(14):
      _steering_step(controller, state)
    sends = _steering_step(controller, state)
    assert _command_torque(sends, 0x249) == 102
    assert _command_torque(_steering_step(controller, state, torque=-1.), 0x249) == 87
    # soft release: latActive loss slews to zero at DELTA_DOWN (15/frame), no hard cut
    assert _command_torque(_steering_step(controller, state, lat_active=False), 0x249) == 72
    for expected in (57, 42, 27, 12, 0):
      assert _command_torque(_steering_step(controller, state, lat_active=False), 0x249) == expected

    controller = _steering_controller(ti=True)
    assert _command_torque(_steering_step(controller, _steering_state(driver_torque=-30)), 0x249) == 0

  def test_ti_real_time_delta_is_limited_over_250_ms(self):
    controller = _steering_controller(ti=True)
    state = _steering_state()
    torques = [_command_torque(_steering_step(controller, state, now_nanos=1_000_000_000), 0x249) for _ in range(40)]
    assert torques[-1] == TorqueInterceptorControllerParams.STEER_MAX_RT_DELTA == 192

    assert _command_torque(_steering_step(controller, state, now_nanos=1_250_000_001), 0x249) == 192
    assert _command_torque(_steering_step(controller, state, now_nanos=1_250_000_001), 0x249) == 198


def crz_info_reference_checksum(dat):
  # independent reimplementation of the CRZ_INFO checksum, validated against 1.67M stock
  # frames with zero mismatches: the sum excludes the STOPPING bit (byte 5, 9,681 frames)
  # and the RESUME_UNLATCHING bit (byte 6, 269 frames)
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04) - (dat[6] & 0x40)) & 0xFF)) & 0xFF


def decode_accel_cmd_raw(dat):
  return (((dat[2] & 0x3) << 11) | (dat[3] << 3) | (dat[4] >> 5)) - 4096


class TestMazdaLongitudinalMessages:
  """The synthetic CRZ_INFO/CRZ_CTRL/radar frames must reproduce stock captures byte for
  byte; the hex values below come from real radar traffic."""

  @pytest.fixture
  def packer(self):
    return CANPacker("mazda_2017")

  def test_alert_command_relays_state_but_not_the_tja_churn(self, packer):
    # camera error and line state pass through to the dash; the camera's own TJA/CTS state
    # machine churns against steering it did not command (442 TJA_TRANSITION toggles in 22
    # min, route 0000010b) and relaying it flapped the dash, so those two fields stay zeroed
    cam_msg = {"LINE_VISIBLE": 1, "LINE_NOT_VISIBLE": 0, "LANE_LINES": 2, "BIT1": 1,
               "BIT2": 0, "BIT3": 0, "NO_ERR_BIT": 0, "ERR_BIT": 1,
               "TJA": 4, "TJA_TRANSITION": 3, "S1": 1, "S1_HBEAM": 0}
    dat = mazdacan.create_alert_command(packer, cam_msg, ldw=False, steer_required=False)[1]
    cp = CANParser("mazda_2017", [("CAM_LANEINFO", float("nan"))], 0)
    cp.update([(0, [(0x440, dat, 0)])])
    out = cp.vl["CAM_LANEINFO"]
    assert out["ERR_BIT"] == 1 and out["LINE_VISIBLE"] == 1 and out["LANE_LINES"] == 2 and out["S1"] == 1
    assert out["TJA"] == 0 and out["TJA_TRANSITION"] == 0

  def test_crz_info_standby_matches_stock(self, packer):
    for counter in range(16):
      checksum = (0x5d - counter) & 0xff
      expected = f"01ffe3ffc000{counter:02x}{checksum:02x}"
      dat = mazdacan.create_acc_command(packer, 0, counter, 0.0, long_active=False, acc_available=False)[1]
      assert dat.hex() == expected

  def test_crz_info_armed_idle_matches_stock(self, packer):
    # armed-idle pegs the command like standby (47,752/47,752 stock armed-idle frames carry
    # raw 8190) and follows the brake on ACC_SET_ALLOWED; the zero-command armed-idle this
    # used to emit exists nowhere in the stock corpus
    for brake_pressed, byte4, base in ((False, 0xc4, 0xd9), (True, 0xc0, 0xdd)):
      for counter in range(16):
        checksum = (base - counter) & 0xff
        expected = f"01ffe3ff{byte4:02x}80{counter:02x}{checksum:02x}"
        dat = mazdacan.create_acc_command(packer, 0, counter, 0.0, long_active=False, acc_available=True,
                                          brake_pressed=brake_pressed)[1]
        assert dat.hex() == expected

  @pytest.mark.parametrize(("accel", "stopping", "unlatching", "counter", "expected"), [
    (0.0, False, False, 0, "01ffe20006800097"),     # engaged, zero command
    (2.0, False, False, 3, "01ffe2fa0680039a"),     # ISO max accel, raw 2000
    (-3.5, False, False, 7, "01ffe04a868007c8"),    # ISO max brake, raw -3500
    (-1.024, True, False, 5, "01ffe18006841503"),   # standstill hold, raw -1024 + stop bits
    (-0.001, False, False, 9, "01ffe1ffe68009b0"),  # latched hold, raw -1
    (0.0, False, True, 11, "01ffe20006804b8c"),     # resume unlatch pulse
    # wire-attested twin: stock drive_0b's first pulse frame is 01ffe1ffe68040b9 -- the
    # checksum matches the counter-0 hold frame (b9) because the unlatch bit is not summed
    (-0.001, False, True, 0, "01ffe1ffe68040b9"),
  ])
  def test_crz_info_engaged_golden_bytes(self, packer, accel, stopping, unlatching, counter, expected):
    dat = mazdacan.create_acc_command(packer, 0, counter, accel, long_active=True, acc_available=False,
                                      stopping=stopping, resume_unlatching=unlatching)[1]
    assert dat.hex() == expected

  def test_crz_info_accel_encoding_and_checksum(self, packer):
    # the packed command must round-trip at the 0.001 factor and carry a valid masked-bit
    # checksum over the whole command window, stop bits set or not
    for raw in range(-3500, 2001, 137):
      for stopping, unlatching in ((False, False), (True, False), (False, True)):
        dat = mazdacan.create_acc_command(packer, 0, raw % 16, raw / 1000.0, long_active=True, acc_available=False,
                                          stopping=stopping, resume_unlatching=unlatching)[1]
        assert decode_accel_cmd_raw(dat) == raw
        assert dat[7] == crz_info_reference_checksum(dat)
        assert bool(dat[5] & 0x04) == stopping
        assert bool(dat[6] & 0x10) == stopping
        assert bool(dat[6] & 0x40) == unlatching
        # the excluded event bits must not move the checksum: stripping them yields the
        # same byte a bare frame carries (stock 0b: 8040b9 vs 8000b9, both chk b9)
        bare = bytearray(dat)
        bare[5] &= ~0x04
        bare[6] &= ~0x40
        assert dat[7] == crz_info_reference_checksum(bytes(bare))

  @pytest.mark.parametrize(("long_active", "acc_available", "gap", "has_lead", "phase", "acc_active_2", "expected"), [
    (False, False, 0, False, 0, False, "0201010000000000"),  # standby
    (False, True, 2, False, 0, False, "02010b0000000000"),   # MRCC armed, SET allowed
    (True, True, 2, True, 1, True, "0a018b2000001000"),      # engaged, cruise, no lead
    (True, True, 2, True, 2, True, "0a018b4000001000"),      # engaged, following a lead
    (True, True, 2, True, 3, True, "0a018b6000001000"),      # stop-and-go hold (near phase)
    (True, True, 2, True, 4, True, "0a018b8000001000"),      # stop-and-go hold (far phase)
    (True, True, 2, True, 3, False, "0a018b6000000000"),     # relaxed hold, ACC_ACTIVE_2 drops
    (True, True, 1, True, 2, True, "0a01874000001000"),      # driver gap 1 mirrored to the dash
  ])
  def test_crz_ctrl_golden_bytes(self, packer, long_active, acc_available, gap, has_lead, phase, acc_active_2, expected):
    dat = mazdacan.create_crz_ctrl(packer, 0, long_active, acc_available, gap, has_lead, phase, acc_active_2)[1]
    assert dat.hex() == expected

  def test_radar_frames_match_stock(self):
    expected = [
      (0x499, "0008c00000000000"),
      (0x361, "fff7fefe1fc00080"),
      (0x362, "fff7fefe1fc78c80"),
      (0x363, "fff7fefe1fc00000"),
      (0x364, "fff7fefe1fc00000"),
      (0x365, "fff7fe7ffbff3fc0"),
      (0x366, "fff7fe7ffbff3fc0"),
    ]
    frames = mazdacan.create_radar_frames(0, 0, None)
    assert [(f.address, f.dat.hex()) for f in frames] == expected

  def test_radar_frames_counter_and_lead_track(self):
    frames = mazdacan.create_radar_frames(2, 15, (10.25, 0.))
    assert all(f.src == 2 for f in frames)
    # counter stamps the low nibble of the last byte on every track
    assert [f.dat[7] & 0x0f for f in frames[1:]] == [15] * 6
    tracks = {f.address: f.dat.hex() for f in frames}
    assert tracks[0x364] == "0a4e00001c00000f"

  def test_lead_track_constant_bytes_match_the_stock_release_capture(self):
    # the template's measurement fields are zeroed, so a zero-range lead reproduces it exactly
    assert mazdacan.create_lead_track(0., 0.) == mazdacan.LEAD_TRACK_TEMPLATE
    # the status pair the camera watches: drive_0b's occupied-slot 1c/00, never the
    # empty-slot c0 in byte 5 the old capture carried
    assert mazdacan.LEAD_TRACK_TEMPLATE[4] & 0x1f == 0x1c
    assert mazdacan.LEAD_TRACK_TEMPLATE[5] == 0x00

  @pytest.mark.parametrize("d_rel,v_rel", [
    (0., 0.), (6.5, 1.5), (10.25, -2.0), (29.4, 2.9375), (255.875, 63.9375), (400., 100.), (5., -80.),
  ])
  def test_lead_track_round_trips_through_the_dbc(self, d_rel, v_rel):
    dat = mazdacan.create_lead_track(d_rel, v_rel)
    cp = CANParser("mazda_2017", [("RADAR_TRACK_364", float("nan"))], 0)
    cp.update([(0, [(0x364, dat, 0)])])
    vl = cp.vl["RADAR_TRACK_364"]
    assert vl["DIST_OBJ"] == pytest.approx(min(max(d_rel, 0.), 255.875), abs=0.0625)
    assert vl["RELV_OBJ"] == pytest.approx(min(max(v_rel, -64.), 63.9375), abs=0.0625)
    # the bits outside the two fields we drive stay exactly as captured
    assert dat[1] & 0x0f == mazdacan.LEAD_TRACK_TEMPLATE[1] & 0x0f
    assert dat[2] == mazdacan.LEAD_TRACK_TEMPLATE[2]
    assert dat[4] & 0x1f == mazdacan.LEAD_TRACK_TEMPLATE[4] & 0x1f
    assert dat[5:] == mazdacan.LEAD_TRACK_TEMPLATE[5:]


class TestStandstillHold:

  @pytest.fixture
  def sm(self):
    return StandstillHold()

  @staticmethod
  def run(sm, frames, **kwargs):
    defaults = dict(long_engaged=True, stopping=False, standstill=False, plan_accel=-1.024,
                    brake_hold=False, gas_pressed=False)
    defaults.update(kwargs)
    for _ in range(frames):
      sm.update(**defaults)
    return sm

  def test_holds_while_the_plan_is_stopping(self, sm):
    self.run(sm, 1)
    assert not sm.holding
    self.run(sm, 1, stopping=True)
    assert sm.holding and sm.stop_bits and sm.acc_active_2
    # arriving at a standstill changes nothing: the plan is still asking for the brakes
    self.run(sm, 500, stopping=True, standstill=True)
    assert sm.holding and sm.stop_bits

  def test_hold_never_relaxes_on_its_own(self, sm):
    # the creep-into-the-lead regression: without the car taking the hold over, the command
    # must stay on the plan's brake no matter how long the stop lasts
    self.run(sm, 1, stopping=True)
    self.run(sm, int(30.0 / DT_CTRL), stopping=True, standstill=True)
    assert sm.holding and sm.stop_bits and sm.acc_active_2
    assert not sm.car_has_hold

  def test_relax_follows_the_car_taking_the_hold(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 10, stopping=True, standstill=True)
    assert not sm.car_has_hold
    self.run(sm, 1, stopping=True, standstill=True, brake_hold=True)
    # stop bits and ACC_ACTIVE_2 drop with the command, together, exactly as stock does
    assert sm.car_has_hold and not sm.stop_bits and not sm.acc_active_2
    # and it is not a latch: if the car lets go, we brake again
    self.run(sm, 1, stopping=True, standstill=True, brake_hold=False)
    assert not sm.car_has_hold and sm.stop_bits and sm.acc_active_2

  def test_released_when_the_plan_asks_to_move(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 500, stopping=True, standstill=True, brake_hold=True)
    assert sm.holding
    # the release is debounced: a plan asking to move for less than the window changes nothing
    # (the body keeps its own latch until the pulse plays, so brake_hold stays up here)
    self.run(sm, RELEASE_DEBOUNCE_FRAMES - 1, standstill=True, brake_hold=True, plan_accel=0.1)
    assert sm.holding and not sm.resume_unlatching
    self.run(sm, 1, standstill=True, brake_hold=True, plan_accel=0.1)
    assert not sm.holding and not sm.car_has_hold
    # the body owned the brakes, so this is the latched family: the pulse fires with the
    # release. The body answers nothing else -- deferring behind silence (route 0000011d)
    # and behind a positive nudge (route 0000012c) both just delayed the resume.
    assert sm.latched_release and sm.resume_unlatching

  def test_release_holds_for_as_long_as_the_plan_wants_to_move(self, sm):
    # the failed-resume regression: no release window to run out from under the plan
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True)
    self.run(sm, int(5.0 / DT_CTRL), standstill=True, plan_accel=0.4)
    assert not sm.holding and not sm.stop_bits

  def test_hold_comes_back_if_the_plan_changes_its_mind(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True)
    self.run(sm, RELEASE_DEBOUNCE_FRAMES, standstill=True, plan_accel=0.2)
    assert not sm.holding
    # nothing was latched, so this release emits no unlatch bit at all, deferred or otherwise
    assert sm.unlatch_frames == 0 and not sm.resume_unlatching
    self.run(sm, 1, stopping=True, standstill=True, plan_accel=-1.0)
    assert sm.holding
    assert not sm.resume_unlatching and sm.unlatch_frames == 0
    assert sm.stop_bits

  def test_never_latched_release_emits_no_pulse(self, sm):
    # a never-latched release has nothing latched to unlatch, so it puts no RESUME_UNLATCHING
    # on the wire at all. Stock blips here, but every pulse this port has emitted latched the
    # camera's SCBS fault (4/4), and mimicking a blip that unlatches nothing is not worth one
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True)
    assert not sm.resume_unlatching
    self.run(sm, RELEASE_DEBOUNCE_FRAMES, standstill=True, plan_accel=0.1)
    assert not sm.holding and not sm.latched_release
    self.run(sm, int(1.0 / DT_CTRL), standstill=True, plan_accel=0.1)
    assert not sm.resume_unlatching and sm.unlatch_frames == 0

  def test_latched_release_pulses_immediately_and_runs_its_length(self, sm):
    # the pulse is the release protocol: the body ignores everything else (routes 0000011d
    # and 0000012c), so waiting only delays the resume. One pulse, stock's latched length.
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True, brake_hold=True)
    self.run(sm, RELEASE_DEBOUNCE_FRAMES - 1, standstill=True, brake_hold=True, plan_accel=0.1)
    assert not sm.resume_unlatching
    self.run(sm, 1, standstill=True, brake_hold=True, plan_accel=0.1)
    assert sm.resume_unlatching, "the pulse must fire with the release"
    self.run(sm, RESUME_UNLATCH_LATCHED_FRAMES, standstill=True, brake_hold=True, plan_accel=0.1)
    assert not sm.resume_unlatching, "pulse outran its length"

  def test_long_disengage_resets(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True, brake_hold=True)
    self.run(sm, 1, long_engaged=False)
    assert not sm.holding and not sm.car_has_hold and not sm.stop_bits

  def test_gas_override_drive_off_releases_the_hold(self, sm):
    # a driver-gas drive-off under an override zeroes the plan's command, so the plan never
    # asks to move but the car does; the stop bits must not follow it up to speed. Stock keeps
    # STOPPING strictly to the final creep, below 0.55 m/s across all rolling frames.
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True)
    assert sm.holding
    self.run(sm, 1, plan_accel=0.0)
    assert not sm.holding and not sm.stop_bits and not sm.resume_unlatching

  def test_stop_abort_releases(self, sm):
    self.run(sm, 1, stopping=True)
    assert sm.holding
    # lead speeds up again before the car reaches standstill
    self.run(sm, 1, stopping=False, plan_accel=0.3)
    assert not sm.holding

  def test_driver_gas_releases_the_hold_without_a_pulse(self, sm):
    # the driver's pedal outranks the hold, the way Toyota's PCM lets the pedal outrank its
    # standstill request -- but the pulse is the ACC's resume protocol, not the driver's:
    # stock's captured gas-ended hold drops the stop bits with no RESUME_UNLATCHING at all,
    # and pulsing there latched an SCBS fault (route 00000103 t+163.8)
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True)
    assert sm.holding
    self.run(sm, 1, stopping=True, standstill=True, gas_pressed=True)
    assert not sm.holding and not sm.resume_unlatching, "gas release must not fire the ACC resume pulse"
    # no re-hold while the pedal is down, and a fresh hold once it lifts with the car stopped
    self.run(sm, RESUME_UNLATCH_LATCHED_FRAMES + 5, stopping=True, standstill=True, gas_pressed=True)
    assert not sm.holding and not sm.resume_unlatching
    self.run(sm, 1, stopping=True, standstill=True)
    assert sm.holding

  def test_plan_flap_below_the_debounce_never_releases(self, sm):
    # the SCBS-axis contamination shape: at a held standstill the lead inches forward and
    # stops, the plan flapping across zero. Sub-debounce flaps must not release at all, and
    # no frame may ever carry the stop bits and the release pulse together
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True)
    for i in range(600):
      accel = 0.3 if (i // 10) % 2 == 0 else -1.0  # 0.1 s swings, below the 0.2 s debounce
      sm.update(long_engaged=True, stopping=accel < 0, standstill=True, plan_accel=accel,
                brake_hold=False, gas_pressed=False)
      assert not (sm.stop_bits and sm.resume_unlatching), "stop bits and pulse on one frame"
      assert not sm.resume_unlatching, "a sub-debounce flap fired a release pulse"
    assert sm.holding

  @pytest.mark.parametrize("brake_hold", [False, True])
  def test_slow_flap_never_mixes_stop_bits_with_the_pulse(self, sm, brake_hold):
    # swings long enough to release each time. Nothing latched (brake_hold False) must never
    # put an unlatch bit on the wire; a body that holds on through every swing falls back to
    # at most one pulse per release, and a re-hold mid-pulse waits it out before re-asserting
    # the stop bits
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True, brake_hold=brake_hold)
    pulses = 0
    prev_unlatch = False
    swing = RELEASE_DEBOUNCE_FRAMES + 30  # long enough for the release and its pulse to play
    for i in range(1200):
      accel = 0.3 if (i // swing) % 2 == 0 else -1.0
      sm.update(long_engaged=True, stopping=accel < 0, standstill=True, plan_accel=accel,
                brake_hold=brake_hold, gas_pressed=False)
      assert not (sm.stop_bits and sm.resume_unlatching), "stop bits and pulse on one frame"
      pulses += int(sm.resume_unlatching and not prev_unlatch)
      prev_unlatch = sm.resume_unlatching
    if brake_hold:
      assert pulses > 0
      assert pulses <= 1 + 1200 // (2 * swing), "more pulses than releases"
    else:
      assert pulses == 0, "a never-latched release put an unlatch bit on the wire"

  def test_latched_pulse_runs_to_completion_through_a_re_hold(self, sm):
    # a latched pulse spans the body's actual unlatch, so a re-hold mid-pulse waits it out
    # (stop bits blocked, stock never emits STOPPING with RESUME_UNLATCHING) instead of
    # canceling it; a second release cannot fire a fresh pulse before the first ends because
    # the release debounce is at least as long as any pulse window
    assert RELEASE_DEBOUNCE_FRAMES >= RESUME_UNLATCH_LATCHED_FRAMES
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True, brake_hold=True)
    self.run(sm, RELEASE_DEBOUNCE_FRAMES, standstill=True, brake_hold=True, plan_accel=0.3)
    assert sm.latched_release and sm.resume_unlatching  # the pulse fires with the release
    self.run(sm, 3, standstill=True, plan_accel=0.3)
    self.run(sm, 1, stopping=True, standstill=True)  # re-hold mid-pulse, body already let go
    assert sm.holding and not sm.stop_bits
    assert sm.resume_unlatching, "a latched pulse mid-release must run to completion"
    self.run(sm, RESUME_UNLATCH_LATCHED_FRAMES, stopping=True, standstill=True, plan_accel=-1.0)
    assert sm.holding and sm.stop_bits and not sm.resume_unlatching


class TestAdvertisedLead:
  """has_lead, the phase and the track slot are one decision, so they are asserted together."""

  @pytest.fixture
  def al(self):
    return AdvertisedLead()

  @staticmethod
  def run(al, frames, **kwargs):
    defaults = dict(lead_visible=True, d_rel=40.0, v_rel=0.0, holding=False)
    defaults.update(kwargs)
    for _ in range(frames):
      al.update(**defaults)
    return al

  def test_lead_follows_only_a_steady_state(self, al):
    # a lead is adopted once leadVisible has held for the debounce window, not before
    self.run(al, LEAD_DEBOUNCE_FRAMES - 1)
    assert not al.has_lead and al.ctrl_phase == 0
    self.run(al, 1)
    assert al.has_lead and al.lead == (40.0, 0.0) and al.ctrl_phase == 2
    # and dropped the same way
    self.run(al, LEAD_DEBOUNCE_FRAMES - 1, lead_visible=False, d_rel=0.)
    assert al.has_lead
    self.run(al, 1, lead_visible=False, d_rel=0.)
    assert not al.has_lead and al.ctrl_phase == 0

  def test_lead_flicker_never_reaches_the_bus(self, al):
    # the measured failure: a marginal 120 m vision lead toggled leadVisible 6 times in 1.4 s
    # (route 6bb2dc61c4 t+400); none of it may reach RADAR_HAS_LEAD or the track slot
    for frames, visible in ((15, True), (5, False), (7, True), (13, False), (10, True)):
      self.run(al, frames, lead_visible=visible)
      assert not al.has_lead, "a flickering lead leaked through the debounce"

  def test_measurement_is_coasted_across_a_dropout(self, al):
    # leadOne goes to zero the instant vision drops the lead, well before the debounce expires.
    # Advertising a fabricated stand-in there put a stationary object 10.25 m dead ahead on the
    # bus at 22 m/s; the last real measurement carries the gap instead -- propagated by its own
    # range rate, never repeated frozen (a frozen range is the camera's proven SCBS trigger)
    self.run(al, 2 * LEAD_DEBOUNCE_FRAMES, d_rel=120.0, v_rel=0.5)
    assert al.lead == (120.0, 0.5)
    coast_frames = LEAD_DEBOUNCE_FRAMES - 1
    self.run(al, coast_frames, lead_visible=False, d_rel=0., v_rel=0.)
    assert al.lead is not None, "dropped the measurement inside the debounce window"
    d, v = al.lead
    assert v == 0.5
    assert d == pytest.approx(120.0 + 0.5 * coast_frames * DT_CTRL, abs=1e-6), \
      "the coast must propagate the range, not freeze it"

  def test_holding_reports_the_stop_phase_only_with_a_lead(self, al):
    self.run(al, 2 * LEAD_DEBOUNCE_FRAMES, holding=True)
    assert al.ctrl_phase == 3
    self.run(al, 2 * LEAD_DEBOUNCE_FRAMES, lead_visible=False, d_rel=0., holding=True)
    assert not al.has_lead and al.ctrl_phase == 0

def _mock_cc(long_active=True, accel=0.5, long_state=None, standstill=False, gas=False,
             resume=False, cancel=False, lead_visible=True, gap=2, available=True,
             stock_radar_alive=False, fsc_settled=True, handback=False, cruise_engaged=False,
             enabled=None, lead_d_rel=12.0, lead_v_rel=0.0, brake_hold=False, brake_pressed=False,
             radar_was_silenced=False):
  # openpilot is enabled whenever it is longitudinally active; a gas override is the case
  # where it stays enabled with longActive low. The mock carries everything the full
  # CarController.update() path reads, so tests can drive update() as well as
  # update_longitudinal() from the one builder.
  enabled = long_active if enabled is None else enabled
  out = SimpleNamespace(standstill=standstill, gasPressed=gas, brakePressed=brake_pressed,
                        vEgoRaw=0., steeringTorque=0.,
                        cruiseState=SimpleNamespace(available=available, enabled=cruise_engaged))
  actuators = SimpleNamespace(accel=accel, longControlState=long_state, torque=0.,
                              as_builder=lambda: SimpleNamespace(torque=0., torqueOutputCan=0, accel=0.))
  cruise = SimpleNamespace(resume=resume, cancel=cancel)
  hud = SimpleNamespace(leadVisible=lead_visible, leadDistanceBars=gap, visualAlert=None)
  cc = SimpleNamespace(enabled=enabled, longActive=long_active, latActive=False,
                       actuators=actuators, cruiseControl=cruise, hudControl=hud)
  cc_sp = SimpleNamespace(stockEcuHandBack=handback,
                          leadOne=SimpleNamespace(dRel=lead_d_rel, vRel=lead_v_rel))
  cs = SimpleNamespace(out=out, resume_button=0, brake_hold=brake_hold,
                       stock_radar_alive=stock_radar_alive, fsc_settled=fsc_settled,
                       radar_session_refused=False, radar_was_silenced=radar_was_silenced,
                       crz_btns_counter=0, cancel_button=0, lkas_allowed_speed=True,
                       cam_lkas={"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0})
  return cc, cc_sp, cs


@pytest.fixture
def cc():
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], alpha_long=True,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], True, False, False)
  assert CP.openpilotLongitudinalControl
  return CarController({Bus.pt: "mazda_2017"}, CP, CP_SP)


@pytest.fixture
def stock_cc():
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], alpha_long=False,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], False, False, False)
  assert not CP.openpilotLongitudinalControl
  return CarController({Bus.pt: "mazda_2017"}, CP, CP_SP)


def _long_frames(sends):
  """(ACCEL_CMD raw, CRZ_INFO.ACC_ACTIVE, CRZ_CTRL.CRZ_ACTIVE) from a bus 0 emission, or None."""
  info = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
  ctrl = next((d for a, d, b in sends if a == 0x21c and b == 0), None)
  if info is None:
    return None
  cp = CANParser("mazda_2017", [("CRZ_INFO", float("nan")), ("CRZ_CTRL", float("nan"))], 0)
  cp.update([(0, [(0x21b, info, 0), (0x21c, ctrl, 0)])])
  return decode_accel_cmd_raw(info), cp.vl["CRZ_INFO"]["ACC_ACTIVE"], cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"]


CRZ_BTNS = 0x9d

# create_radar_frames stamps the counter into the last byte, so an empty slot is the first seven
_EMPTY_TRACK = mazdacan.RADAR_TRACK_MSGS[0x364][:7]


def _decode(msg, addr, dat):
  """CANParser view of a single frame."""
  cp = CANParser("mazda_2017", [(msg, float("nan"))], 0)
  cp.update([(0, [(addr, dat, 0)])])
  return cp.vl[msg]


def _frames(sends, addr, bus=0):
  return [d for a, d, b in sends if a == addr and b == bus]


def _frame(sends, addr, bus=0):
  return next(iter(_frames(sends, addr, bus)), None)


def _track_occupied(dat):
  return dat[:7] != _EMPTY_TRACK


def _crz_ctrl(dat):
  """(RADAR_HAS_LEAD, RADAR_LEAD_RELATIVE_DISTANCE) from a CRZ_CTRL frame."""
  v = _decode("CRZ_CTRL", 0x21c, dat)
  return int(v["RADAR_HAS_LEAD"]), int(v["RADAR_LEAD_RELATIVE_DISTANCE"])


def _lead_track(dat):
  """(DIST_OBJ, RELV_OBJ) decoded from a 0x364 track frame."""
  v = _decode("RADAR_TRACK_364", 0x364, dat)
  return v["DIST_OBJ"], v["RELV_OBJ"]


def _step(cc, **kw):
  kw.setdefault("long_state", structs.CarControl.Actuators.LongControlState.pid)
  control, control_sp, carstate = _mock_cc(**kw)
  sends = cc.update_longitudinal(control, control_sp, carstate)
  cc.frame += 1
  return sends


class TestLongitudinalIntegration:
  """Drives the real CarController.update_longitudinal through an engage -> cruise -> stop ->
  hold -> resume timeline and checks the emitted CAN, not just the state machine in isolation."""

  def test_engaged_frame_rates_and_counters(self, cc):
    long = structs.CarControl.Actuators.LongControlState
    crz_info = crz_ctrl = radar_static = tester = 0
    for _ in range(100):  # 1 s at 100 Hz
      sends = _step(cc, long_state=long.pid, accel=1.0, gap=2)
      addrs = [a for a, _, _ in sends]
      buses = {a: [] for a, _, _ in sends}
      for a, _, b in sends:
        buses[a].append(b)
      crz_info += addrs.count(0x21b)
      crz_ctrl += addrs.count(0x21c)
      radar_static += addrs.count(0x499)
      tester += sum(1 for a, _, _ in sends if a == 0x764)
      # CRZ_INFO/CRZ_CTRL, when emitted, always go to both bus 0 and bus 2
      if 0x21b in buses:
        assert sorted(buses[0x21b]) == [0, 2]
        assert sorted(buses[0x21c]) == [0, 2]

    # 100 Hz loop: long msgs at 50 Hz (x2 buses), radar at 10 Hz (x2), tester at 2 Hz
    assert crz_info == crz_ctrl == 100    # 50 frames x 2 buses
    assert radar_static == 20             # 10 frames x 2 buses
    assert tester == 2                    # 2 Hz, single bus
    assert cc.long_counter == 50 and cc.radar_counter == 10

  def test_gap_setting_mirrors_driver(self, cc):
    for gap in (1, 2, 3):
      cc.frame = 0  # force emission on the first step
      sends = _step(cc, gap=gap, long_state=structs.CarControl.Actuators.LongControlState.pid)
      ctrl = next(dat for a, dat, b in sends if a == 0x21c and b == 0)
      cp = CANParser("mazda_2017", [("CRZ_CTRL", float("nan"))], 0)
      cp.update([(0, [(0x21c, ctrl, 0)])])
      assert cp.vl["CRZ_CTRL"]["DISTANCE_SETTING"] == gap

  def test_stop_emits_hold_then_relaxes(self, cc):
    long = structs.CarControl.Actuators.LongControlState

    def accel_cmd(sends):
      dat = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
      return None if dat is None else decode_accel_cmd_raw(dat)

    # approach the stop
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=False)
    # hold at a standstill: the command is the plan's own and must not relax on its own, no
    # matter how long the stop lasts (the creep-into-the-lead regression)
    cmds = []
    for _ in range(int(30.0 / 0.01)):
      cmd = accel_cmd(_step(cc, long_state=long.stopping, accel=-1.024, standstill=True))
      if cmd is not None:
        cmds.append(cmd)
    settled = cmds[len(cmds) // 2:]
    assert settled and set(settled) == {-1024}, f"hold command drifted off the plan: {sorted(set(settled))}"

    # once the body ECU takes the hold over, stock stops asking for the brakes and so do we
    relaxed = []
    for _ in range(int(1.0 / 0.01)):
      cmd = accel_cmd(_step(cc, long_state=long.stopping, accel=-1.024, standstill=True,
                            brake_hold=True))
      if cmd is not None:
        relaxed.append(cmd)
    assert relaxed and set(relaxed) == {round(CarControllerParams.ACCEL_HOLD_LATCHED * 1000)}

  def test_gas_override_stays_engaged(self, cc):
    """A gas press is an override, not a disengagement. The command goes to zero as on every
    other port, but the engaged bits stay set the way Honda drives CONTROL_ON off CC.enabled.
    Clearing them mid-decel takes the PCM out of ACC mode (docs/mazda-gas-override.md)."""
    long = structs.CarControl.Actuators.LongControlState

    # braking hard, then the driver taps the gas
    for _ in range(200):
      _step(cc, long_state=long.pid, accel=-2.0, cruise_engaged=True)
    assert cc.accel_last == pytest.approx(-2.0)

    cmds = []
    for _ in range(100):  # 1 s of override
      sends = _step(cc, long_active=False, enabled=True, long_state=long.off, accel=0.,
                    gas=True, cruise_engaged=True)
      frame = _long_frames(sends)
      if frame is not None:
        cmds.append(frame)

    raw, acc_active, crz_active = zip(*cmds, strict=True)
    assert all(acc_active), "ACC_ACTIVE dropped during a gas override"
    assert all(crz_active), "CRZ_ACTIVE dropped during a gas override"
    assert set(raw) == {0}, f"command should be zero through the override, got {sorted(set(raw))}"

  def test_command_slew_is_rate_limited(self, cc):
    """The plan can step; the wire should not. Windup is limited tightly because dumping the
    brake in one frame is what the driver feels, winddown loosely so braking is never delayed."""
    long = structs.CarControl.Actuators.LongControlState
    for _ in range(200):
      _step(cc, long_state=long.pid, accel=-2.0, cruise_engaged=True)
    assert cc.accel_last == pytest.approx(-2.0)

    # plan jumps straight to +1.0: the command must ramp, not step
    prev = cc.accel_last
    for _ in range(5):
      _step(cc, long_state=long.pid, accel=1.0, cruise_engaged=True)
      assert cc.accel_last - prev == pytest.approx(CarControllerParams.ACCEL_WINDUP_LIMIT, abs=1e-6)
      prev = cc.accel_last

    # and the other way, at the looser winddown limit
    for _ in range(200):
      _step(cc, long_state=long.pid, accel=1.0, cruise_engaged=True)
    prev = cc.accel_last
    for _ in range(5):
      _step(cc, long_state=long.pid, accel=-3.0, cruise_engaged=True)
      assert cc.accel_last - prev == pytest.approx(CarControllerParams.ACCEL_WINDDOWN_LIMIT, abs=1e-6)
      prev = cc.accel_last

  def test_accel_last_tracks_the_wire_not_the_plan(self, cc):
    # update() reports accel_last as actuatorsOutput.accel, the way Toyota, Ford and Honda
    # report the value they sent. It must be the wire value, clip and hold included.
    long = structs.CarControl.Actuators.LongControlState

    # a plan beyond the envelope is reported clipped, not as asked
    for _ in range(400):
      sends = _step(cc, long_state=long.pid, accel=-9.0, cruise_engaged=True)
    assert cc.accel_last == pytest.approx(CarControllerParams.ACCEL_MIN)
    frame = _long_frames(sends)
    if frame is not None:
      assert frame[0] == round(cc.accel_last * 1000)

    # the standstill hold is the plan's own command, and that is what gets reported
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=True, cruise_engaged=True)
    assert cc.accel_last == pytest.approx(-1.5)

    # through a gas override we report the zero we actually send
    for _ in range(10):
      _step(cc, long_active=False, enabled=True, long_state=long.off, accel=0., gas=True,
            cruise_engaged=True)
    assert cc.accel_last == 0.

  def test_gas_from_standstill_hold_releases_the_brake(self, cc):
    # gas out of a hold is a resume, not a slow release: the hold command must go straight to
    # zero rather than ramping off at the cruising override rate
    long = structs.CarControl.Actuators.LongControlState
    for _ in range(int(3.0 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=True, cruise_engaged=True)
    assert cc.accel_last < -0.5, "never reached the standstill hold"

    for _ in range(20):
      _step(cc, long_active=False, enabled=True, long_state=long.off, accel=0., gas=True,
            standstill=True, cruise_engaged=True)
    assert cc.accel_last == 0., f"hold not released for the driver's gas: {cc.accel_last}"

  def test_release_command_holds_through_the_debounce_then_jumps(self, cc):
    """Stock never lets ACCEL_CMD climb while STOPPING is asserted: through the release
    debounce the command stays at the hold value. Once the stop bits drop it relax-jumps
    into stock's release band and ramps. Pre-ramping toward the plan during the debounce put
    the zero-cross inside the pulse (route 00000100 t+353); slewing up off the hold value put
    hold-grade braking under it (route 00000053 t+714.8). Both latched SCBS."""
    long = structs.CarControl.Actuators.LongControlState
    lead = dict(lead_visible=True, lead_d_rel=4.0, lead_v_rel=0.0)
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=False, **lead)
    for _ in range(int(3.0 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.3, standstill=True, **lead)
    assert cc.accel_last == pytest.approx(-1.3)

    rows = []
    for _ in range(int(1.5 / 0.01)):
      sends = _step(cc, long_state=long.pid, accel=1.0, standstill=True, **lead)
      dat = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
      if dat is not None:
        rows.append((decode_accel_cmd_raw(dat), (dat[5] >> 2) & 1, (dat[6] >> 6) & 1))

    debounce = [r for r in rows if r[1]]
    assert debounce, "no stop-bit frames through the release debounce"
    assert all(cmd == -1300 for cmd, _, _ in debounce), \
      f"command moved off the hold while STOPPING was asserted: {sorted({c for c, _, _ in debounce})}"

    # nothing was latched, so no unlatch bit goes out at all
    assert not any(unl for _, _, unl in rows), "a never-latched release pulsed"
    assert max(cmd for cmd, _, _ in rows) > 500, "command never ramped up after the release"

  def test_near_zero_hold_release_emits_no_pulse(self, cc):
    # a no-lead hold relaxes the plan to ~0, so the release ramp would cross zero in the first
    # pulse frame -- the shape behind the routes 000000fe t+44.54 / 00000100 t+353.18 latches.
    # Nothing is latched here, so the release now carries no unlatch bit for it to land in.
    long = structs.CarControl.Actuators.LongControlState
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-0.5, standstill=False)
    for _ in range(int(2.0 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-0.02, standstill=True)
    assert cc.accel_last == pytest.approx(-0.02)

    rows = []
    for _ in range(int(1.5 / 0.01)):
      sends = _step(cc, long_state=long.pid, accel=1.0, standstill=True)
      dat = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
      if dat is not None:
        rows.append((decode_accel_cmd_raw(dat), (dat[6] >> 6) & 1))

    assert not any(unl for _, unl in rows), "a never-latched release pulsed"
    assert max(cmd for cmd, _ in rows) > 500, "command never ramped up after the release"

  def test_release_keeps_climbing_until_the_car_actually_moves(self, cc):
    """Route 00000009--ad9e22f986 t+452.9 (EPS-swapped CX-9): the release ran correctly, the
    ramp caught the plan at +0.47 with a vision lead 2.5 m ahead, handed the command back --
    and the car sat dead still for 1.5 s until the driver used the pedal. Mazda runs ki=0, so
    LongControl's pid state emits a_target verbatim and nothing ever escalates. The ramp must
    keep climbing past the plan while the car has not moved."""
    long = structs.CarControl.Actuators.LongControlState
    lead = dict(lead_visible=True, lead_d_rel=2.5, lead_v_rel=0.0)
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.024, standstill=False, **lead)
    for _ in range(int(2.0 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.024, standstill=True, **lead)

    # the plan asks for exactly the creep the CX-9 could not move on, and the car stays stopped
    peak = -10.
    for _ in range(int(2.0 / 0.01)):
      _step(cc, long_state=long.pid, accel=0.47, standstill=True, **lead)
      peak = max(peak, cc.accel_last)
    assert peak > 0.47 + 0.2, f"command plateaued at the plan and never asked harder: {peak:.2f}"
    assert peak <= CarControllerParams.ACCEL_BREAKAWAY_MAX + 1e-6, f"climbed past the cap: {peak:.2f}"
    # the override sits on top of the plan, so it is bounded by what stock itself commands
    # pulling away from a stop: over all 31 stock stop->go episodes the breakaway command
    # spans +0.405..+1.425 (latched median +0.958), so this must not exceed stock's own worst
    assert CarControllerParams.ACCEL_BREAKAWAY_MAX <= 1.45, "breakaway ceiling past stock's own max"

    # once it moves, the plan owns the command again
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.pid, accel=0.47, standstill=False, **lead)
    assert cc.accel_last == pytest.approx(0.47, abs=0.01)

  def test_breakaway_gives_up_so_a_stuck_car_is_not_leaned_on(self, cc):
    # something we cannot see is holding the car (kerb, grade). Asking forever is worse than
    # settling back onto the plan, which the driver can then override with the pedal.
    long = structs.CarControl.Actuators.LongControlState
    for _ in range(int(2.0 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.024, standstill=True)
    for _ in range(BREAKAWAY_FRAMES + int(1.0 / 0.01)):
      _step(cc, long_state=long.pid, accel=0.3, standstill=True)
    assert cc.accel_last == pytest.approx(0.3, abs=0.01), \
      f"still leaning on a car that never moved: {cc.accel_last:.2f}"

  def test_breakaway_never_climbs_against_a_latched_body(self, cc):
    """The freeze that keeps the command pinned while GEAR.BRAKE_HOLD is still set (route
    00000115 t+381.3, camera faulted 90 ms into the pulse) outranks the breakaway climb: a
    body-latched hold is pulsed, never leaned on."""
    long = structs.CarControl.Actuators.LongControlState
    lead = dict(lead_visible=True, lead_d_rel=4.0, lead_v_rel=0.0)
    for _ in range(int(2.0 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.3, standstill=True, brake_hold=True, **lead)
    assert cc.stop_and_go.car_has_hold

    for _ in range(RESUME_UNLATCH_LATCHED_FRAMES + int(1.0 / 0.01)):
      _step(cc, long_state=long.pid, accel=0.5, standstill=True, brake_hold=True, **lead)
      assert cc.accel_last <= CarControllerParams.ACCEL_RESUME_PULSE_MAX + 1e-6, \
        f"breakaway climbed against a still-latched body: {cc.accel_last:.2f}"

  def test_never_latched_release_speaks_the_stock_wire_grammar(self, cc):
    """Route 00000053 t+714.8 (second CX-5): slewing off the hold value under a 13-frame pulse
    put hold-grade braking beneath RESUME_UNLATCHING, a (stop, unlatch, cmd) tuple stock never
    emits, and the camera latched SCBS 90 ms in with a real departing lead advertised. Stock's
    never-latched grammar (33-pulse census): the command relax-jumps into the -0.27..-0.11 band
    in one frame and the ramp climbs ~+25 raw per wire frame. Stock also blips RESUME_UNLATCHING
    here; we do not -- nothing is latched, and 4 of 4 pulses this port ever emitted latched the
    camera -- so the blip assertions are replaced by requiring no unlatch bit at all."""
    long = structs.CarControl.Actuators.LongControlState
    lead = dict(lead_visible=True, lead_d_rel=4.0, lead_v_rel=0.0)
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.0, standstill=False, **lead)
    for _ in range(int(2.0 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.024, standstill=True, **lead)
    assert cc.accel_last == pytest.approx(-1.024)

    rows = []
    for _ in range(int(1.5 / 0.01)):
      sends = _step(cc, long_state=long.pid, accel=0.45, standstill=True, **lead)
      dat = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
      if dat is not None:
        rows.append((decode_accel_cmd_raw(dat), (dat[5] >> 2) & 1, (dat[6] >> 6) & 1))

    assert not any(stop and unl for _, stop, unl in rows), "stop bits and pulse on one frame"
    drop = next(i for i, (_, stop, _) in enumerate(rows) if not stop)
    post = rows[drop:]
    # the relax jump: no post-drop frame ever carries hold-grade braking again
    assert all(cmd >= -280 for cmd, _, _ in post), f"command stayed at hold depth after the drop: {min(c for c, _, _ in post)}"
    assert post[0][0] <= -180, f"release did not start inside the stock band: {post[0][0]}"
    assert not any(unl for _, _, unl in rows), "a never-latched release pulsed"
    # the ramp: stock's +25 raw per wire frame, straight through the drive-off
    ramping = [c for c, _, _ in post][:20]
    assert all(20 <= b - a <= 30 for a, b in zip(ramping, ramping[1:], strict=False)), f"off the stock ramp: {ramping}"

  @pytest.mark.parametrize("drop_wire_frames", [1, 2, 3])
  def test_latched_release_speaks_the_stock_pulse_shape(self, cc, drop_wire_frames):
    # a body-latched hold releases with a 9-wire-frame pulse: the command sits pinned at the
    # relaxed -1 raw for as long as the body still reports GEAR.BRAKE_HOLD (as in every latched
    # release of the census -- climbing before the drop faulted the camera 90 ms in, route
    # 00000115 t+381.3), then climbs stock's ~+25 raw per frame ramp, peaking inside stock's
    # family and never past the +0.25 ceiling (census: 6-11 frames, hold drop 1-3 frames in,
    # cmd -1 climbing to +24..+342)
    long = structs.CarControl.Actuators.LongControlState
    lead = dict(lead_visible=True, lead_d_rel=4.0, lead_v_rel=0.0)
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=False, **lead)
    for _ in range(int(2.0 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.3, standstill=True, brake_hold=True, **lead)
    assert cc.accel_last == pytest.approx(CarControllerParams.ACCEL_HOLD_LATCHED)

    # the body reacts to the pulse: BRAKE_HOLD drops 1-3 wire frames after it starts (census)
    rows = []
    pulse_started = None
    window = RELEASE_DEBOUNCE_FRAMES + RESUME_UNLATCH_LATCHED_FRAMES + int(1.0 / 0.01)
    for i in range(window):
      body_holds = pulse_started is None or i < pulse_started + 2 * drop_wire_frames
      sends = _step(cc, long_state=long.pid, accel=1.0, standstill=True, brake_hold=body_holds, **lead)
      dat = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
      if dat is not None:
        unl = (dat[6] >> 6) & 1
        rows.append((decode_accel_cmd_raw(dat), unl, body_holds))
        if pulse_started is None and unl:
          pulse_started = i

    pulse = [(cmd, held) for cmd, unl, held in rows if unl]
    cap = round(CarControllerParams.ACCEL_RESUME_PULSE_MAX * 1000)
    assert len(pulse) == RESUME_UNLATCH_LATCHED_FRAMES // 2, f"pulse ran {len(pulse)} wire frames"
    # the contract the route 115 fault turned on: no pulse frame moves off the relaxed hold
    # while the body still reports its latch
    pinned = [cmd for cmd, held in pulse if held]
    assert len(pinned) == drop_wire_frames and all(cmd == -1 for cmd in pinned), \
      f"command moved under the latched hold: {pinned}"
    # then the ramp: stock's +25 raw per wire frame from the relaxed hold
    ramp = [cmd for cmd, held in pulse if not held]
    assert -1 <= ramp[0] <= 15, f"ramp must start off the relaxed hold: {ramp[0]}"
    assert all(20 <= b - a <= 30 for a, b in zip(ramp, ramp[1:], strict=False)), f"off the stock ramp: {ramp}"
    assert -1 + (len(ramp) - 1) * 20 <= max(ramp) <= cap, f"in-pulse peak outside the ramp's own family: {max(ramp)}"
    assert max(cmd for cmd, _, _ in rows) > cap, "command never ramped past the cap after the pulse"

  def test_latched_release_pulse_starts_at_the_release(self, cc):
    """Routes 0000011d and 0000012c: the body ignores silence and a positive nudge alike,
    and answers the pulse within 2-3 wire frames. The pulse fires with the release, so the
    resume happens when the plan commands it."""
    long = structs.CarControl.Actuators.LongControlState
    lead = dict(lead_visible=True, lead_d_rel=4.0, lead_v_rel=0.0)
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=False, **lead)
    for _ in range(int(2.0 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.3, standstill=True, brake_hold=True, **lead)

    rows = []
    for _ in range(RELEASE_DEBOUNCE_FRAMES + int(0.5 / 0.01)):
      sends = _step(cc, long_state=long.pid, accel=1.0, standstill=True, brake_hold=True, **lead)
      dat = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
      if dat is not None:
        rows.append((decode_accel_cmd_raw(dat), (dat[5] >> 2) & 1, (dat[6] >> 6) & 1))

    # the stop bits are already down during a body-latched hold, so the pulse's deadline is
    # the debounce itself: it must start on the first wire frame after the plan's request lands
    assert not any(stop for _, stop, _ in rows), "stop bits reappeared during a latched hold"
    first_unl = next(i for i, (_, _, unl) in enumerate(rows) if unl)
    assert first_unl <= RELEASE_DEBOUNCE_FRAMES // 2 + 1, \
      f"pulse lagged the release: {first_unl - RELEASE_DEBOUNCE_FRAMES // 2} wire frames late"
    assert rows[first_unl][0] == -1, f"pulse did not start at the relaxed hold: {rows[first_unl][0]}"

  def test_lead_track_follows_the_measured_lead(self, cc):
    # a frozen track is what latches the camera's SCBS fault, so the range we advertise has to
    # move with the lead we are actually following
    long = structs.CarControl.Actuators.LongControlState
    # let the lead debounce adopt the visible lead before sampling the track
    for _ in range(LEAD_DEBOUNCE_FRAMES):
      _step(cc, long_state=long.pid, accel=0.5, lead_visible=True, lead_d_rel=20.0, lead_v_rel=-1.5)
    seen = []
    for i in range(60):
      sends = _step(cc, long_state=long.pid, accel=0.5, lead_visible=True,
                    lead_d_rel=20.0 - 0.1 * i, lead_v_rel=-1.5)
      track = next((d for a, d, b in sends if a == 0x364 and b == 0), None)
      if track is not None:
        seen.append(_lead_track(track))
    assert len(seen) > 1
    dists = [d for d, _ in seen]
    assert all(a > b for a, b in zip(dists, dists[1:], strict=False)), f"range did not close with the lead: {dists}"
    assert all(v == pytest.approx(-1.5, abs=0.0625) for _, v in seen)

  def test_hold_with_nothing_ahead_advertises_nothing(self, cc):
    # No fabricated object. The body does not decide the latch on the advertisement: across 32
    # stock engaged standstills the radar said has_lead=1 in every one, yet 23 latched
    # GEAR.BRAKE_HOLD and 9 did not (one held 104 s), and 89 of 115 stock latches happened at
    # has_lead=0 / phase=0. A phantom the camera can refute is the SCBS trigger.
    long = structs.CarControl.Actuators.LongControlState
    held, ctrls = [], []
    for _ in range(400):
      sends = _step(cc, long_state=long.stopping, accel=-1.024, standstill=True,
                    lead_visible=False, lead_d_rel=0.0, cruise_engaged=True)
      held += _frames(sends, 0x364)
      ctrls += _frames(sends, 0x21c)
    assert held and ctrls
    assert not any(map(_track_occupied, held)), "fabricated a lead for a hold with nothing ahead"
    assert all(_crz_ctrl(d) == (0, 0) for d in ctrls), "advertised a lead with nothing in view"
    # and the hold itself is untouched: the plan's brake and the stop bits still go out
    assert cc.stop_and_go.holding and cc.stop_and_go.stop_bits

  def test_vision_lead_dropout_does_not_fabricate_a_lead_at_speed(self, cc):
    # leadOne goes to zero the instant the vision lead drops while sm.lead_visible is still
    # latched. Falling through to the hold fallback there put a stationary object 10.25 m dead
    # ahead on the bus at 22 m/s, 20 times across the two 2026-08-25 drives.
    long = structs.CarControl.Actuators.LongControlState
    for _ in range(200):  # settle a real lead at 120 m while cruising
      _step(cc, long_state=long.pid, accel=0.5, lead_visible=True, lead_d_rel=120.0,
            lead_v_rel=0.5, cruise_engaged=True)

    dropped = []
    for _ in range(int(LEAD_DEBOUNCE_FRAMES * 0.8)):  # inside the debounce window
      dropped += _frames(_step(cc, long_state=long.pid, accel=0.5, lead_visible=False,
                               lead_d_rel=0.0, lead_v_rel=0.0, cruise_engaged=True), 0x364)
    assert dropped
    for d in dropped:
      dist = _lead_track(d)[0]
      assert dist == pytest.approx(120.0, abs=1.0), f"track teleported to {dist} m"

  def test_has_lead_phase_and_track_never_disagree(self, cc):
    # stock pairs all three absolutely: has_lead=0 <=> phase=0, and RADAR_HAS_LEAD=1 with all six
    # slots empty appears 8 times in 1,095,826 stock samples. We shipped has_lead=0 with phase=1
    # for 22-84% of every engaged drive before this was derived from one decision.
    long = structs.CarControl.Actuators.LongControlState
    cases = [
      dict(long_state=long.pid, accel=0.5, lead_visible=True, lead_d_rel=40.0),
      dict(long_state=long.pid, accel=0.5, lead_visible=False, lead_d_rel=0.0),
      dict(long_state=long.stopping, accel=-1.024, standstill=True, lead_visible=False, lead_d_rel=0.0),
      dict(long_state=long.pid, accel=0.3, standstill=True, lead_visible=False, lead_d_rel=0.0),
      dict(long_state=long.pid, accel=0.5, lead_visible=True, lead_d_rel=0.0),
    ]
    for kw in cases:
      for _ in range(120):
        sends = _step(cc, cruise_engaged=True, **kw)
        trk, ctl = _frame(sends, 0x364), _frame(sends, 0x21c)
        if trk is None or ctl is None:
          continue
        has_lead, phase = _crz_ctrl(ctl)
        assert bool(has_lead) == _track_occupied(trk), f"has_lead/track disagree for {kw}"
        assert (phase == 0) == (has_lead == 0), f"has_lead/phase disagree for {kw}"

  def test_lead_survives_disengagement(self, cc):
    # perception is engagement-independent: stock advertises RADAR_HAS_LEAD=1 with cruise off in
    # 19.5% of all frames. Dropping the advertisement at disengage made a real car 4.5 m ahead
    # vanish from the bus in one frame while the driver braked toward it, and the camera ran its
    # SCBS display six seconds (route 0000004d t+212)
    long = structs.CarControl.Actuators.LongControlState
    for _ in range(120):
      _step(cc, cruise_engaged=True, lead_d_rel=4.8, accel=-0.5)
    for _ in range(60):
      sends = _step(cc, long_active=False, enabled=False, long_state=long.off, accel=0.,
                    lead_d_rel=4.8)
      trk, ctl = _frame(sends, 0x364), _frame(sends, 0x21c)
      if ctl is None:
        continue
      has_lead, phase = _crz_ctrl(ctl)
      assert has_lead == 1 and phase != 0, "disengaging dropped a real lead off the bus"
      if trk is not None:
        assert _lead_track(trk)[0] == pytest.approx(4.8, abs=0.1)

  def test_no_resume_button_while_openpilot_owns_longitudinal(self, cc):
    # We are the ACC here, so the hold is released in-protocol. The car's own MRCC never presses
    # RES either: 0 of 23 stock body-latched-hold releases put one on the bus. A press would also
    # put a second writer on CRZ_BTNS, which ICBM owns.
    for accel in (0.3, -1.024):
      for standstill in (True, False):
        control, _, _ = _mock_cc(standstill=standstill, accel=accel, resume=True)
        assert not cc.resume_requested(control)

  def test_resume_button_still_sent_with_stock_longitudinal(self, stock_cc):
    # stock ACC owns the hold there, and the button is the only lever openpilot has on it
    control, _, _ = _mock_cc(standstill=True, accel=0.3, resume=True)
    assert stock_cc.resume_requested(control)

    control, _, _ = _mock_cc(standstill=True, accel=0.3, resume=False)
    assert not stock_cc.resume_requested(control)

  def test_body_latched_hold_releases_in_protocol(self, cc):
    # the release the button used to stand in for: stop bits already relaxed to the body, then
    # the plan asks to move and the unlatch pulse fires with the release
    long = structs.CarControl.Actuators.LongControlState
    for _ in range(200):
      _step(cc, long_state=long.stopping, accel=-1.024, standstill=True,
            cruise_engaged=True, brake_hold=True)
    assert cc.stop_and_go.holding and cc.stop_and_go.car_has_hold
    assert not cc.stop_and_go.stop_bits  # body owns the brakes, stock relaxes here

    for _ in range(RELEASE_DEBOUNCE_FRAMES):
      sends = _step(cc, long_state=long.pid, accel=0.3, standstill=True,
                    cruise_engaged=True, brake_hold=True)
      assert not any(a == CRZ_BTNS for a, _, _ in sends), "CRZ_BTNS written at the release"
    assert not cc.stop_and_go.holding
    assert cc.stop_and_go.resume_unlatching, "the pulse must fire with the release"

  def test_gas_pedal_without_cruise_stays_disengaged(self, cc):
    # gas pressed while openpilot is not enabled must not advertise an engaged ACC
    off = structs.CarControl.Actuators.LongControlState.off
    cc.frame = 0
    sends = _step(cc, long_active=False, enabled=False, long_state=off, gas=True, available=True)
    info = next(dat for a, dat, b in sends if a == 0x21b and b == 0)
    assert info.hex().startswith("01ffe3ffc480")  # armed-but-idle pattern, command pegged

  def test_disengaged_emits_stock_patterns(self, cc):
    off = structs.CarControl.Actuators.LongControlState.off
    # main off, not available: the exact standby pattern the panda allowlists byte-for-byte
    cc.frame = 0
    sends = _step(cc, long_active=False, long_state=off, available=False)
    info = next(dat for a, dat, b in sends if a == 0x21b and b == 0)
    assert info.hex().startswith("01ffe3ffc000")
    # MRCC armed but not engaged: the command stays pegged and ACC_SET_ALLOWED follows the
    # brake, exactly the two patterns stock alternates between at an armed idle
    cc.frame = 0
    sends = _step(cc, long_active=False, long_state=off, available=True)
    info = next(dat for a, dat, b in sends if a == 0x21b and b == 0)
    assert info.hex().startswith("01ffe3ffc480")
    cc.frame = 0
    sends = _step(cc, long_active=False, long_state=off, available=True, brake_pressed=True)
    info = next(dat for a, dat, b in sends if a == 0x21b and b == 0)
    assert info.hex().startswith("01ffe3ffc080")


class TestCancelCarveOut:
  """controlsd raises cruiseControl.cancel whenever cruiseState.enabled has no matching
  CC.enabled (mazda reports pcmCruise). While the stock radar still owns the bus that
  engagement is the driver's own stock MRCC and a CANCEL turns its main off within ~100 ms,
  so the documented stay-stock fallback used to leave the driver with no cruise at all. Once
  the radar has been silenced a stock engagement is impossible and cancel handles desync."""

  def _full_update(self, cc, cancel, radar_was_silenced, stock_radar_alive):
    control, control_sp, carstate = _mock_cc(long_active=False, enabled=False, accel=0.,
                                             long_state=structs.CarControl.Actuators.LongControlState.off,
                                             available=False, cruise_engaged=True, cancel=cancel,
                                             stock_radar_alive=stock_radar_alive, fsc_settled=False,
                                             radar_was_silenced=radar_was_silenced)
    cc.frame = 10  # off the 50-frame alert cadence, on the 10-frame cancel cadence
    _, sends = cc.update(control, control_sp, carstate, 0)
    return [a for a, _, _ in sends]

  def test_no_cancel_while_the_radar_is_stock(self, cc):
    # pre-teardown settle window, and equally the silencing-failed drive: a driver SET is
    # their own stock MRCC and must be left alone
    addrs = self._full_update(cc, cancel=True, radar_was_silenced=False, stock_radar_alive=True)
    assert CRZ_BTNS not in addrs, "CANCELed the driver's own stock MRCC"

  def test_cancel_still_sent_after_the_teardown(self, cc):
    # post-teardown a stock engagement is impossible: cancel keeps handling state desync
    addrs = self._full_update(cc, cancel=True, radar_was_silenced=True, stock_radar_alive=False)
    assert CRZ_BTNS in addrs

  def test_stock_longitudinal_cancel_unaffected(self, stock_cc):
    addrs = self._full_update(stock_cc, cancel=True, radar_was_silenced=False, stock_radar_alive=True)
    assert CRZ_BTNS in addrs


SESSION_PROG_DAT = bytes([0x02, 0x10, 0x02, 0, 0, 0, 0, 0])
SESSION_DFLT_DAT = bytes([0x02, 0x10, 0x01, 0, 0, 0, 0, 0])
TESTER_PRESENT_DAT = bytes([0x02, 0x3e, 0x80, 0, 0, 0, 0, 0])


class TestRadarSessionBounds:
  """The fire-and-forget UDS session has no readable NRC, so every episode is bounded the
  way disable_ecu bounds its retries."""

  def test_silencing_gives_up_bounded(self):
    m = RadarSessionManager()
    for _ in range(RADAR_SESSION_LIMIT_FRAMES + 2):
      state = m.update(True, True, False, standstill=True, session_refused=False)
    assert state == RadarSessionState.STOCK and m.silencing_failed
    # and stays given up for the drive: stock keeps the bus
    for _ in range(10):
      assert m.update(True, True, False, standstill=True, session_refused=False) == RadarSessionState.STOCK

  def test_negative_response_gives_up_immediately(self):
    # route 000000fe t+15.0 shows the radar answers a session request within 10 ms, so a
    # negative response is definitive: no reason to burn the silence budget
    m = RadarSessionManager()
    m.update(True, True, False, standstill=True, session_refused=False)
    assert m.state == RadarSessionState.SILENCING
    assert m.update(True, True, False, standstill=True, session_refused=True) == RadarSessionState.STOCK
    assert m.silencing_failed

  def test_handback_stops_waiting_for_a_dead_radar(self):
    m = RadarSessionManager()
    m.update(True, False, False, standstill=True, session_refused=False)
    assert m.state == RadarSessionState.SILENCED
    for _ in range(RADAR_SESSION_LIMIT_FRAMES + 2):
      state = m.update(True, False, True, standstill=True, session_refused=False)
    assert state == RadarSessionState.STOCK

  def test_completed_handback_never_resilences(self):
    # the parked toggle-off regression: the monitor's CC_SP assert used to drop after its done
    # latch, the manager read that as a withdrawal, fell to STOCK, and re-entered SILENCING on
    # the same call (parked, gate still passed) -- re-silencing the radar it had just handed
    # back, right before shutdown, leaving it to a degraded unattended S3 recovery
    m = RadarSessionManager()
    m.update(True, False, False, standstill=True, session_refused=False)
    assert m.state == RadarSessionState.SILENCED
    m.update(True, False, True, standstill=True, session_refused=False)
    assert m.state == RadarSessionState.HANDBACK
    assert m.update(True, True, True, standstill=True, session_refused=False) == RadarSessionState.STOCK
    for handback in (True, False):
      for alive in (True, False):
        for _ in range(5):
          assert m.update(True, alive, handback, standstill=True, session_refused=False) == RadarSessionState.STOCK

  def test_withdrawn_handback_allows_retakeover(self):
    # only a hand-back that ran to completion latches: a genuine toggle-flip-back
    # mid-hand-back gets the normal takeover again
    m = RadarSessionManager()
    m.update(True, False, False, standstill=True, session_refused=False)
    m.update(True, False, True, standstill=True, session_refused=False)
    assert m.state == RadarSessionState.HANDBACK
    state = m.update(True, False, False, standstill=True, session_refused=False)
    assert state == RadarSessionState.SILENCED and not m.handback_completed

  def test_silencing_waits_for_standstill_but_adoption_does_not(self):
    # actively silencing disables AEB, so it only starts pre-motion like disable_ecu;
    # adopting an already-quiet radar disables nothing and proceeds anywhere
    m = RadarSessionManager()
    for _ in range(10):
      assert m.update(True, True, False, standstill=False, session_refused=False) == RadarSessionState.STOCK
    assert m.update(True, True, False, standstill=True, session_refused=False) == RadarSessionState.SILENCING
    m2 = RadarSessionManager()
    assert m2.update(True, False, False, standstill=False, session_refused=False) == RadarSessionState.SILENCED


def test_non_gen1_platform_refused_at_admission():
  # one init-time check instead of per-frame guards in the message builders, which every
  # frame layout in mazdacan assumes; the fall-throughs used to emit an all-zero CAM_LKAS
  # and return None from the button builder, straight into can_sends
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], alpha_long=False,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], False, False, False)
  CP.flags = 0
  with pytest.raises(NotImplementedError):
    CarController({Bus.pt: "mazda_2017"}, CP, CP_SP)


class TestRadarSessionSequencing:
  """Boot teardown deferral and the ordered hand-back: what goes on the bus in each
  radar session state, driven through the real CarController.update_longitudinal."""

  def _step(self, cc, stock_radar_alive, fsc_settled, handback=False, cruise_engaged=False, standstill=True):
    # standstill=True models the parked boot; actively silencing a live radar is gated on it
    off = structs.CarControl.Actuators.LongControlState.off
    return _step(cc, long_active=False, accel=0., long_state=off, lead_visible=False, available=False,
                 stock_radar_alive=stock_radar_alive, fsc_settled=fsc_settled,
                 handback=handback, cruise_engaged=cruise_engaged, standstill=standstill)

  @staticmethod
  def _uds(sends):
    return [dat for a, dat, b in sends if a == 0x764]

  @staticmethod
  def _synthetic(sends):
    return [a for a, _, _ in sends if a in (0x21b, 0x21c, 0x499)]

  def test_stock_state_is_silent(self, cc):
    # radar alive, gate not yet passed: nothing at all goes on the bus
    for _ in range(200):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=False)
      assert sends == []

  def test_boot_teardown_sequence(self, cc):
    # gate passes with the stock radar alive: programming-session requests at 2 Hz,
    # still no synthetic frames and no tester present
    for i in range(100):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=True)
      if i % CarControllerParams.RADAR_UDS_STEP == 0:
        assert self._uds(sends) == [SESSION_PROG_DAT]
      else:
        assert self._uds(sends) == []
      assert self._synthetic(sends) == []
    # radar goes quiet: synthetic frames + tester present take over, session requests stop
    saw_tester = False
    for _ in range(100):
      frame = cc.frame
      sends = self._step(cc, stock_radar_alive=False, fsc_settled=True)
      assert SESSION_PROG_DAT not in self._uds(sends)
      if frame % CarControllerParams.LONG_STEP == 0:
        assert len(self._synthetic(sends)) > 0
      saw_tester |= TESTER_PRESENT_DAT in self._uds(sends)
    assert saw_tester

  def test_handback_sequence(self, cc):
    # reach SILENCED
    self._step(cc, stock_radar_alive=False, fsc_settled=True)
    # hand-back requested: default-session requests at 2 Hz, tester present stops,
    # synthetic frames continue while the radar is still quiet
    saw_default = False
    for _ in range(100):
      frame = cc.frame
      sends = self._step(cc, stock_radar_alive=False, fsc_settled=True, handback=True)
      assert TESTER_PRESENT_DAT not in self._uds(sends)
      saw_default |= SESSION_DFLT_DAT in self._uds(sends)
      if frame % CarControllerParams.LONG_STEP == 0:
        assert len(self._synthetic(sends)) > 0
    assert saw_default
    # stock radar returns: everything stops
    for _ in range(200):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=True, handback=True)
      assert sends == []

  def test_handback_before_teardown_stops_everything(self, cc):
    # toggle-off while still waiting on the gate: no session ever entered, so no
    # hand-back traffic either
    self._step(cc, stock_radar_alive=True, fsc_settled=False)
    for _ in range(120):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=False, handback=True)
      assert sends == []

  def test_teardown_waits_for_stock_cruise_disengage(self, cc):
    # driver engaged stock MRCC before the gate passed (warm boot): hold the teardown
    for _ in range(120):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=True, cruise_engaged=True)
      assert sends == []
    # driver disengages: teardown proceeds
    cc.frame = 0
    sends = self._step(cc, stock_radar_alive=True, fsc_settled=True, cruise_engaged=False)
    assert SESSION_PROG_DAT in self._uds(sends)

  def test_completed_handback_stays_stock_after_the_assert_drops(self, cc):
    # CC_SP is rebuilt every frame, so once the toggle monitor's done latch stops asserting
    # the hand-back the manager sees handback=False; a completed hand-back must not turn
    # into a fresh takeover on the very next frame (parked => standstill, gate still passed)
    self._step(cc, stock_radar_alive=False, fsc_settled=True)
    self._step(cc, stock_radar_alive=False, fsc_settled=True, handback=True)
    self._step(cc, stock_radar_alive=True, fsc_settled=True, handback=True)
    for _ in range(200):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=True, handback=False)
      assert sends == []

  def test_s3_recovery_resilences(self, cc):
    # radar reappears mid-drive (dropped tester present, S3 timeout): re-request the session
    self._step(cc, stock_radar_alive=False, fsc_settled=True)
    cc.frame = CarControllerParams.RADAR_UDS_STEP  # align to a session-request frame
    sends = self._step(cc, stock_radar_alive=True, fsc_settled=True)
    assert SESSION_PROG_DAT in self._uds(sends)
    # and settles back to silenced once quiet again
    sends = self._step(cc, stock_radar_alive=False, fsc_settled=True)
    assert SESSION_PROG_DAT not in self._uds(sends)
