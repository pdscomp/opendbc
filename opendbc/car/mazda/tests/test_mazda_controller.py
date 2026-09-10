#!/usr/bin/env python3
"""Fork-only Mazda controller tests: the torque-interceptor command path, the cancel
carve-out while the stock radar owns cruise, and the steer-to-zero CAM_LANEINFO LKAS-on
presentation. Everything else lives in the upstream split suites (steering, longitudinal,
standstill hold, lead, mazdacan, golden tx)."""

from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.carcontroller import CarController, laneinfo_present_lkas_on
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, MazdaFlags, TorqueInterceptorControllerParams

CRZ_BTNS = 0x9d


def _steering_controller(*, ti=False, steer_to_zero=False):
  CP = CarInterface.get_params(CAR.MAZDA_CX5, {0: {}, 1: {}, 2: {}}, [], alpha_long=False,
                               is_release=False, docs=False)
  if ti:
    CP.flags |= MazdaFlags.TORQUE_INTERCEPTOR.value
  if steer_to_zero:
    CP.flags |= MazdaFlags.STEER_TO_ZERO_EPS.value
  controller = CarController({Bus.pt: "mazda_2017"}, CP, structs.CarParamsSP())
  controller.frame = 1  # HUD output is unrelated to these steering checks
  return controller


def _steering_state(*, healthy=True, driver_torque=0, speed=5.):  # above the TI standstill gate
  return SimpleNamespace(
    out=SimpleNamespace(vEgoRaw=speed, steeringTorque=driver_torque, brakePressed=False,
                        canValid=True, cruiseState=SimpleNamespace(available=False, enabled=False),
                        standstill=False),
    ti_lkas_allowed=healthy,
    crz_btns_counter=0,
    cancel_button=1,  # suppress unrelated ICBM output
    accel_button=0,
    decel_button=0,
    resume_button=0,
    lkas_allowed_speed=True,
    lkas_rejected=0,
    steer_undelivered=False,
    steer_first_engage_hold=False,
    stock_tja=0,
    stock_cts_stuck=False,
    radar_was_silenced=False,
    radar_control_active=False,
    radar_restore_failed=False,
    radar_handback_active=False,
    radar_session_response=0,
    radar_session_refused=False,
    radar_bus_healthy=False,
    stock_radar_gone=False,
    cruise_available=False,
    cruise_enabled=False,
    fsc_settled=False,
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

  @pytest.mark.parametrize("speed", [0.99, 1.0, 1.01])
  def test_ti_not_ready_soft_releases_host_request_at_speed_boundary(self, speed):
    controller = _steering_controller(ti=True)
    state = _steering_state(speed=2.0)
    for frame in range(10):
      sends = _steering_step(controller, state, now_nanos=frame * 10_000_000)
    previous = _command_torque(sends, 0x249)
    assert previous > 0

    state.out.vEgoRaw = speed
    state.ti_lkas_allowed = False
    sends = _steering_step(controller, state, now_nanos=100_000_000)
    assert _command_torque(sends, 0x249) == previous - controller.ti_params.STEER_DELTA_DOWN

  def test_stock_and_ti_authority_are_independent(self):
    # Under the EPS_HW scheme every gen1 EPS runs the measured envelope; at this speed the
    # applied-torque ceiling (1148 below 8.5 m/s) binds before the 1200 scale does.
    controller = _steering_controller(ti=True, steer_to_zero=True)
    state = _steering_state()
    for frame in range(119):
      _steering_step(controller, state, now_nanos=frame * 10_000_000)
    sends = _steering_step(controller, state, now_nanos=1_190_000_000)
    ceiling = round(float(np.interp(state.out.vEgoRaw, controller.params.EPS_CEILING_LOOKUP[0],
                                    controller.params.EPS_CEILING_LOOKUP[1])))
    assert _command_torque(sends, 0x243) == ceiling == 1148
    assert _command_torque(sends, 0x249) == 600

  def test_ti_history_does_not_reuse_stock_history(self):
    controller = _steering_controller(ti=True)
    state = _steering_state(healthy=False)
    for frame in range(19):
      _steering_step(controller, state, now_nanos=frame * 10_000_000)
    sends = _steering_step(controller, state, now_nanos=190_000_000)
    assert _command_torque(sends, 0x243) == 240
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


def _mock_cc(long_active=True, accel=0.5, long_state=None, standstill=False, gas=False,
             resume=False, cancel=False, lead_visible=True, gap=2, available=True,
             stock_radar_alive=False, fsc_settled=True, handback=False, cruise_engaged=False,
             enabled=None, lead_d_rel=12.0, lead_v_rel=0.0, brake_hold=False, brake_pressed=False,
             radar_was_silenced=False):
  # openpilot is enabled whenever it is longitudinally active; a gas override is the case
  # where it stays enabled with longActive low. The mock carries everything the full
  # CarController.update() path reads.
  enabled = long_active if enabled is None else enabled
  out = SimpleNamespace(standstill=standstill, gasPressed=gas, brakePressed=brake_pressed,
                        vEgoRaw=0., steeringTorque=0., canValid=True,
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
                       radar_session_refused=False, radar_session_response=0,
                       radar_was_silenced=radar_was_silenced,
                       radar_control_active=False, radar_restore_failed=False,
                       radar_handback_active=False, radar_bus_healthy=True,
                       stock_radar_gone=False, cruise_available=available,
                       cruise_enabled=cruise_engaged,
                       lkas_rejected=0, steer_undelivered=False, steer_first_engage_hold=False,
                       stock_tja=0, stock_cts_stuck=False,
                       accel_button=0, decel_button=0,
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


class TestLaneinfoLkasSpoof:
  CAM_MSG_KEYS = ["LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT2", "BIT3", "NO_ERR_BIT", "ERR_BIT", "S1", "S1_HBEAM"]

  @staticmethod
  def _cam_laneinfo():
    return {k: 0 for k in TestLaneinfoLkasSpoof.CAM_MSG_KEYS}

  def test_spoof_only_on_steer_to_zero_eps(self):
    cam = self._cam_laneinfo()
    stz = SimpleNamespace(flags=MazdaFlags.STEER_TO_ZERO_EPS)
    non_stz = SimpleNamespace(flags=0)
    out = laneinfo_present_lkas_on(cam, stz)
    assert (out["LANE_LINES"], out["LINE_VISIBLE"], out["LINE_NOT_VISIBLE"]) == (2, 1, 0)
    assert laneinfo_present_lkas_on(cam, non_stz) is cam

  def test_spoof_survives_wire_roundtrip(self):
    out = laneinfo_present_lkas_on(self._cam_laneinfo(), SimpleNamespace(flags=MazdaFlags.STEER_TO_ZERO_EPS))
    addr, dat, bus = mazdacan.create_alert_command(CANPacker("mazda_2017"), out, False, False)
    cp = CANParser("mazda_2017", [("CAM_LANEINFO", float("nan"))], 0)
    cp.update([(0, [(addr, dat, bus)])])
    assert (cp.vl["CAM_LANEINFO"]["LANE_LINES"], cp.vl["CAM_LANEINFO"]["LINE_VISIBLE"]) == (2, 1)

  def test_spoof_preserves_camera_fault_state(self):
    cam = self._cam_laneinfo() | {"BIT1": 1, "BIT2": 1, "BIT3": 1, "NO_ERR_BIT": 1, "ERR_BIT": 1, "S1": 1, "S1_HBEAM": 1}
    out = laneinfo_present_lkas_on(cam, SimpleNamespace(flags=MazdaFlags.STEER_TO_ZERO_EPS))
    addr, dat, bus = mazdacan.create_alert_command(CANPacker("mazda_2017"), out, False, False)
    cp = CANParser("mazda_2017", [("CAM_LANEINFO", float("nan"))], 0)
    cp.update([(0, [(addr, dat, bus)])])
    assert {key: cp.vl["CAM_LANEINFO"][key] for key in self.CAM_MSG_KEYS[3:]} == {key: cam[key] for key in self.CAM_MSG_KEYS[3:]}

  def test_call_site_emits_spoofed_laneinfo(self, cc):
    control, control_sp, carstate = _mock_cc()
    control.latActive = True
    control.hudControl.visualAlert = structs.CarControl.HUDControl.VisualAlert.none
    carstate.cam_laneinfo = self._cam_laneinfo()
    carstate.cam_lkas = {"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0}
    carstate.crz_btns_counter = 0
    carstate.cancel_button = 1
    carstate.lkas_allowed_speed = True
    carstate.out.steeringTorque = 0
    carstate.out.vEgoRaw = 10.0
    cc.frame = 50

    _, sends = cc.update(control, control_sp, carstate, 0)
    laneinfo = next(d for a, d, b in sends if a == 0x440 and b == 0)
    cp = CANParser("mazda_2017", [("CAM_LANEINFO", float("nan"))], 0)
    cp.update([(0, [(0x440, laneinfo, 0)])])
    assert cp.vl["CAM_LANEINFO"]["LANE_LINES"] == 2


if __name__ == "__main__":
  pytest.main([__file__, "-q"])
