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
from opendbc.car.mazda.longitudinal import (LEAD_DEBOUNCE_FRAMES, RADAR_SESSION_LIMIT_FRAMES, RELEASE_DEBOUNCE_FRAMES,
                                            RESUME_UNLATCH_FRAMES,
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
    assert cx5_2022_params.STEER_DELTA_DOWN == 25

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


def _steering_state(*, healthy=True, driver_torque=0, speed=0.):
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
    assert _command_torque(sends, 0x243) == stock_max
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
    assert _command_torque(_steering_step(controller, state, lat_active=False), 0x249) == 0

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
  # independent reimplementation of the CRZ_INFO checksum, validated against 1.94M stock
  # frames including all 10,350 stop-bit frames
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04)) & 0xFF)) & 0xFF


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
    (0.0, False, True, 11, "01ffe20006804b4c"),     # resume unlatch pulse
  ])
  def test_crz_info_engaged_golden_bytes(self, packer, accel, stopping, unlatching, counter, expected):
    dat = mazdacan.create_acc_command(packer, 0, counter, accel, long_active=True, acc_available=False,
                                      stopping=stopping, resume_unlatching=unlatching)[1]
    assert dat.hex() == expected

  def test_crz_info_accel_encoding_and_checksum(self, packer):
    # the packed command must round-trip at the 0.001 factor and carry a valid masked-bit
    # checksum over the whole command window, stop bits set or not
    for raw in range(-3500, 2001, 137):
      for stopping in (False, True):
        dat = mazdacan.create_acc_command(packer, 0, raw % 16, raw / 1000.0, long_active=True, acc_available=False,
                                          stopping=stopping)[1]
        assert decode_accel_cmd_raw(dat) == raw
        assert dat[7] == crz_info_reference_checksum(dat)
        assert bool(dat[5] & 0x04) == stopping
        assert bool(dat[6] & 0x10) == stopping

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
    frames = mazdacan.create_radar_frames(2, 15, (mazdacan.LEAD_TRACK_DIST, 0.))
    assert all(f.src == 2 for f in frames)
    # counter stamps the low nibble of the last byte on every track
    assert [f.dat[7] & 0x0f for f in frames[1:]] == [15] * 6
    tracks = {f.address: f.dat.hex() for f in frames}
    assert tracks[0x364] == "0a4000001dc0000f"

  def test_lead_track_at_template_range_is_the_capture(self):
    assert mazdacan.create_lead_track(mazdacan.LEAD_TRACK_DIST, 0.) == mazdacan.LEAD_TRACK_TEMPLATE

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
    self.run(sm, RELEASE_DEBOUNCE_FRAMES - 1, standstill=True, plan_accel=0.1)
    assert sm.holding and not sm.resume_unlatching
    self.run(sm, 1, standstill=True, plan_accel=0.1)
    assert not sm.holding and not sm.car_has_hold
    assert sm.resume_unlatching

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
    self.run(sm, 1, stopping=True, standstill=True, plan_accel=-1.0)
    assert sm.holding
    # the release pulse is still playing: the stop bits wait it out (stock never emits
    # STOPPING together with RESUME_UNLATCHING) and reassert the frame it completes
    assert sm.resume_unlatching and not sm.stop_bits
    self.run(sm, RESUME_UNLATCH_FRAMES, stopping=True, standstill=True, plan_accel=-1.0)
    assert sm.holding and sm.stop_bits and not sm.resume_unlatching

  def test_unlatch_pulses_once_at_the_release(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True)
    assert not sm.resume_unlatching
    self.run(sm, RELEASE_DEBOUNCE_FRAMES, standstill=True, plan_accel=0.1)
    assert sm.resume_unlatching
    self.run(sm, RESUME_UNLATCH_FRAMES, standstill=True, plan_accel=0.1)
    assert not sm.resume_unlatching

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
    self.run(sm, RESUME_UNLATCH_FRAMES + 5, stopping=True, standstill=True, gas_pressed=True)
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

  def test_slow_flap_never_mixes_stop_bits_with_the_pulse(self, sm):
    # swings long enough to release each time: each release still pulses exactly once, and a
    # re-hold mid-pulse waits the pulse out before re-asserting the stop bits
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True)
    pulses = 0
    prev_unlatch = False
    for i in range(1200):
      accel = 0.3 if (i // 30) % 2 == 0 else -1.0  # 0.3 s swings, above the debounce
      sm.update(long_engaged=True, stopping=accel < 0, standstill=True, plan_accel=accel,
                brake_hold=False, gas_pressed=False)
      assert not (sm.stop_bits and sm.resume_unlatching), "stop bits and pulse on one frame"
      pulses += int(sm.resume_unlatching and not prev_unlatch)
      prev_unlatch = sm.resume_unlatching
    assert pulses > 0
    assert pulses <= 1200 // (2 * 30), "more pulses than releases"

  def test_pulse_never_retriggers_mid_pulse(self, sm):
    # release -> re-hold -> release inside one pulse window: the playing pulse must run to
    # completion, not restart (the debounce is short enough to fit a second release inside)
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True)
    self.run(sm, RELEASE_DEBOUNCE_FRAMES, stopping=True, standstill=True, plan_accel=0.3)
    assert sm.resume_unlatching
    self.run(sm, 3, standstill=True, plan_accel=0.3)
    self.run(sm, 1, stopping=True, standstill=True)  # re-hold mid-pulse
    assert sm.holding and not sm.stop_bits
    remaining = sm.unlatch_frames
    self.run(sm, RELEASE_DEBOUNCE_FRAMES, stopping=True, standstill=True, plan_accel=0.3)  # release again mid-pulse
    assert not sm.holding
    assert sm.unlatch_frames == remaining - RELEASE_DEBOUNCE_FRAMES, "pulse restarted mid-pulse"


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

  def test_release_command_ramps_from_the_hold_value(self, cc):
    """Stock never lets ACCEL_CMD climb while STOPPING is asserted: through the release
    debounce the command stays at the hold value, and the ramp starts only once the stop
    bits drop, so the zero-cross lands after the pulse like every captured stock release.
    Pre-ramping toward the plan during the debounce put the zero-cross inside the pulse,
    which the camera latched as an SCBS fault (route 00000100 t+353)."""
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

    pulse = [cmd for cmd, _, unl in rows if unl]
    assert pulse, "release never pulsed"
    assert max(pulse) < 0, f"command crossed zero inside the pulse: {max(pulse)}"
    assert max(cmd for cmd, _, _ in rows) > 500, "command never ramped up after the release"

  def test_near_zero_hold_release_never_goes_positive_in_the_pulse(self, cc):
    # a no-lead hold relaxes the plan to ~0, so the release ramp would cross zero in the
    # first pulse frame; both observed SCBS latches fired at exactly that zero-cross inside
    # a non-latched pulse (routes 000000fe t+44.54, 00000100 t+353.18)
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

    pulse = [cmd for cmd, unl in rows if unl]
    assert pulse and max(pulse) <= 0, f"non-latched pulse went positive: {max(pulse, default=None)}"
    assert max(cmd for cmd, _ in rows) > 500, "command never ramped up after the pulse"

  def test_latched_release_command_is_capped_through_the_pulse(self, cc):
    # a body-latched hold releases from ACCEL_HOLD_LATCHED, so the ramp would cross zero
    # almost immediately; stock peaks at +0.25 m/s2 inside these pulses and so must we
    long = structs.CarControl.Actuators.LongControlState
    lead = dict(lead_visible=True, lead_d_rel=4.0, lead_v_rel=0.0)
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=False, **lead)
    for _ in range(int(2.0 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.3, standstill=True, brake_hold=True, **lead)
    assert cc.accel_last == pytest.approx(CarControllerParams.ACCEL_HOLD_LATCHED)

    rows = []
    for _ in range(int(1.5 / 0.01)):
      sends = _step(cc, long_state=long.pid, accel=1.0, standstill=True, brake_hold=True, **lead)
      dat = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
      if dat is not None:
        rows.append((decode_accel_cmd_raw(dat), (dat[6] >> 6) & 1))

    pulse = [cmd for cmd, unl in rows if unl]
    cap = round(CarControllerParams.ACCEL_RESUME_PULSE_MAX * 1000)
    assert pulse and max(pulse) == cap, f"in-pulse command off the stock ceiling: {max(pulse, default=None)}"
    assert max(cmd for cmd, _ in rows) > cap, "command never ramped past the cap after the pulse"

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
    # the plan asks to move and RESUME_UNLATCHING pulses while the command ramps positive
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
    assert cc.stop_and_go.resume_unlatching

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
