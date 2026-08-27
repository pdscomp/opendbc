import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, gen_empty_fingerprint
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, CarControllerParams, MazdaFlags
from opendbc.sunnypilot.car.interfaces import setup_interfaces

CAM_LANEINFO = 0x440
CAM_EMPTY = 0x21d
CAM_PEDESTRIAN = 0x25d
RADAR_UDS_RESP = 0x76c

# Real CAM_LANEINFO prefixes, captured on two CX-5 2022s running the same FSC firmware
# (GSH7-67XK2-U). Only byte 1 differs: bit 5 is BIT2, bit 6 is NO_ERR_BIT.
BOOTING = bytes([0x42, 0b01000001, 0, 0, 0, 0, 0, 0])       # NO_ERR_BIT set: still booting
SETTLED = bytes([0x42, 0b00000001, 0, 0, 0, 0, 0, 0])       # markers clear: settled
BIT2_LATCHED = bytes([0x41, 0b00100001, 0, 0, 0, 0, 0, 0])  # BIT2 stuck high for a whole cycle
FAULTED = bytes([0x42, 0b00000001, 0, 0, 0, 0x01, 0, 0])    # ERR_BIT (bit 40) set


def _interface(alpha_long=True, ti=False):
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=alpha_long,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fingerprint, [],
                                     alpha_long=alpha_long, is_release_sp=False, docs=False)
  if ti:
    setup_interfaces(CarInterface, CP, CP_SP, [{"TorqueInterceptorEnabled": True}])
  return CarInterface(CP, CP_SP)


# CAM_LANEINFO's real cadence: tests feed it at the longest measured period (values.py), not
# per control frame. Feeding it at 100 Hz masked a freshness window shorter than the message
# period (the gate never settled on the car while every test passed).
CAM_LANEINFO_PERIOD_FRAMES = int(CarControllerParams.CAM_LANEINFO_PERIOD_T / DT_CTRL)


def _ti_feedback(torque=0, **overrides):
  values = {
    "TI_TORQUE_SENSOR": torque,
    "CHKSUM": 0,
    "VERSION_NUMBER": 1,
    "STATE": 3,
    "VIOL": 0,
    "ERROR": 0,
    "RAMP_DOWN": 0,
    "SPARE": 0,
  }
  values.update(overrides)
  return CANPacker("mazda_2017").make_can_msg("TI_FEEDBACK", 1, values)


def _feed_ti(CI, frames=1, torque=0, speed_kph=None, **overrides):
  if speed_kph is not None and frames == 1:
    frames = 6  # vEgo kalman needs a few samples to converge
  ret = None
  packer = CANPacker("mazda_2017")
  for i in range(1, frames + 1):
    msgs = [_ti_feedback(torque, **overrides)]
    if speed_kph is not None:
      msgs.append(packer.make_can_msg("WHEEL_SPEEDS", 0, {"FL": speed_kph, "FR": speed_kph, "RL": speed_kph, "RR": speed_kph}))
    ret, _ = CI.update([(i * 20_000_000, msgs)])
  return ret


def _feed(CI, payload, seconds):
  # payload None = a camera dropout, nothing on the bus at all
  frames = int(seconds / DT_CTRL)
  for i in range(frames):
    msgs = [(CAM_LANEINFO, payload, 2)] if payload is not None and i % CAM_LANEINFO_PERIOD_FRAMES == 0 else []
    CI.update([(int(i * DT_CTRL * 1e9), msgs)])
  return CI.CS.fsc_settled


@pytest.mark.parametrize("alpha_long", [False, True])
def test_carstate_runs_with_real_parsers(alpha_long):
  # vl_all, unlike vl, has no lazy message registration: every message read through it
  # must be listed in get_can_parsers. The op-long FSC settle gate crashed card on its
  # first update when CAM_LANEINFO was missing from the cam parser (KeyError, 2026-07-29).
  CI = _interface(alpha_long)
  assert CI.CP.openpilotLongitudinalControl == alpha_long
  for _ in range(10):
    CI.update([])


class TestFscSettleGate:
  """The gate that defers the radar teardown past the FSC's cold-boot radar-presence check.

  It must hold while the camera is booting or faulted, and must not be vetoed indefinitely
  by a bit that carries no boot information.
  """

  def test_never_settles_while_boot_marker_is_set(self):
    settle = CarControllerParams.FSC_SETTLE_T
    assert not _feed(_interface(), BOOTING, settle * 2)

  def test_never_settles_while_err_bit_is_set(self):
    # a latched i-ACTIVSENSE fault shows the boot markers clear, so ERR_BIT must veto on its own
    settle = CarControllerParams.FSC_SETTLE_T
    assert not _feed(_interface(), FAULTED, settle * 2)

  def test_settles_once_the_boot_marker_clears(self):
    CI = _interface()
    assert not _feed(CI, BOOTING, 3.0)
    assert not _feed(CI, SETTLED, CarControllerParams.FSC_SETTLE_T - 1.0)
    assert _feed(CI, SETTLED, 1.5)

  def test_a_latched_bit2_does_not_block_the_teardown_forever(self):
    # One CX-5 2022 cold-booted with BIT2 high and NO_ERR_BIT clear for an entire ignition
    # cycle (36.5 s, route 7c735af5fce56485|00000011). BIT2 was in the gate, so the radar was
    # never silenced and the two-master guard held accFaulted for the whole drive.
    assert _feed(_interface(), BIT2_LATCHED, CarControllerParams.FSC_SETTLE_T * 1.5)

  def test_settles_at_the_longest_observed_camera_period(self):
    # _feed runs at the longest measured period, the worst case the freshness window has to
    # ride through: a shorter window zeroes the settle counter on every gap and the gate
    # never opens (the regression that shipped in baf0f383c3 with CAM_LANEINFO_FRESH_T = 0.5)
    assert _feed(_interface(), SETTLED, CarControllerParams.FSC_SETTLE_T * 1.5)

  def test_camera_dropout_resets_the_settle_timer(self):
    # the window is a freshness gate, not decoration: a genuine dropout, well past any real
    # inter-frame gap, must start the settle timer over
    CI = _interface()
    _feed(CI, SETTLED, CarControllerParams.FSC_SETTLE_T * 0.8)
    _feed(CI, None, CarControllerParams.CAM_LANEINFO_FRESH_T + 0.5)
    assert not _feed(CI, SETTLED, CarControllerParams.FSC_SETTLE_T * 0.5)
    assert _feed(CI, SETTLED, CarControllerParams.FSC_SETTLE_T * 0.6)

  def test_gate_starts_closed_before_any_camera_frame(self):
    # the parser reads all-zero before the first frame, which would otherwise look settled
    CI = _interface()
    for i in range(int(CarControllerParams.FSC_SETTLE_T * 2 / DT_CTRL)):
      CI.update([(int(i * DT_CTRL * 1e9), [])])
    assert not CI.CS.fsc_settled


class TestStockFcw:
  """0x21d (CAM_EMPTY) idles at STATUS 0x7f and leaves it only while the camera actively
  shows its SCBS collision display (route 0000004d t+213). The payloads are the captured
  idle and active frames from that route."""

  IDLE = bytes.fromhex("7f3fff00000affff")
  ACTIVE = bytes.fromhex("52124b00000ad294")

  def _feed_21d(self, CI, payload, i=0):
    ret, _ = CI.update([(int(i * DT_CTRL * 1e9), [(CAM_EMPTY, payload, 2)])])
    return ret

  def test_display_active_sets_fcw(self):
    CI = _interface()
    assert self._feed_21d(CI, self.IDLE).stockFcw is False
    assert self._feed_21d(CI, self.ACTIVE, 1).stockFcw is True
    assert self._feed_21d(CI, self.IDLE, 2).stockFcw is False

  def test_no_fcw_before_first_camera_frame(self):
    # the parser reads STATUS as 0 before the first frame, which is != 0x7f
    CI = _interface()
    ret, _ = CI.update([(0, [])])
    assert ret.stockFcw is False

  def test_ped_warning_bit_sets_fcw(self):
    # never observed in 1.57M corpus frames, wired for coverage: PED_WARNING is bit 9
    CI = _interface()
    self._feed_21d(CI, self.IDLE)
    ret, _ = CI.update([(int(1 * DT_CTRL * 1e9), [(CAM_PEDESTRIAN, bytes.fromhex("07fa3c0000000000"), 2), (CAM_EMPTY, self.IDLE, 2)])])
    assert ret.stockFcw is True


class TestRadarSessionResponse:
  """The radar answers session requests within ~10 ms (route 000000fe t+15.0), and the
  session manager consumes the flag on the same control frame it is set."""

  def test_negative_response_sets_refused(self):
    CI = _interface()
    assert not CI.CS.radar_session_refused
    # 03 7F 10 22: conditionsNotCorrect to a session-control request
    CI.update([(0, [(RADAR_UDS_RESP, bytes.fromhex("037f102200000000"), 0)])])
    assert CI.CS.radar_session_refused
    CI.update([(int(DT_CTRL * 1e9), [])])
    assert not CI.CS.radar_session_refused, "the flag is same-frame, not latched"

  def test_positive_response_is_not_a_refusal(self):
    CI = _interface()
    # the real capture: 06 50 02 with the session parameter record (P2*=5.0 s)
    CI.update([(0, [(RADAR_UDS_RESP, bytes.fromhex("065002001901f400"), 0)])])
    assert not CI.CS.radar_session_refused

  def test_response_pending_is_not_a_refusal(self):
    CI = _interface()
    # 03 7F 10 78: requestCorrectlyReceived-ResponsePending; UDS clients wait through it
    CI.update([(0, [(RADAR_UDS_RESP, bytes.fromhex("037f107800000000"), 0)])])
    assert not CI.CS.radar_session_refused


class TestBrakeHold:
  """GEAR.BRAKE_HOLD is the body ECU reporting that it owns the standstill hold. Stock relaxes
  its own command the instant this sets, so the payloads below come straight off the two logs
  that pinned the signal down: a hold that latched (route caace206f6 seg 8, 0x17 at 1157.34 s)
  and one that never did (route 00000065 seg 4, stuck at 0x07 while the car crept)."""

  @pytest.mark.parametrize(("payload", "expected"), [
    ("142007ff02f00000", False),  # hold not taken over: keep braking
    ("142017ff02f00000", True),   # body has the brakes
    ("14200fff02f00000", False),  # released again at the resume
  ])
  def test_decodes_the_hold_bit(self, payload, expected):
    CI = _interface()
    # CANParser registers a message lazily on first access, so the first frame only arms it
    for i in range(2):
      CI.update([(int(i * DT_CTRL * 1e9), [(0x228, bytes.fromhex(payload), 0)])])
    assert CI.CS.brake_hold is expected

  def test_defaults_to_not_held(self):
    # nothing parsed yet must read as "the car is not holding", the direction that keeps braking
    assert not _interface().CS.brake_hold


class TestTwoMasterGuard:
  """The stock-radar guard wears two hats: before the first teardown it is the expected boot
  phase and must only hold availability low (no fault alert); once the radar has been silenced,
  hearing it again is a genuine two-master conflict and must raise accFaulted."""

  def _feed_guard(self, CI, seconds, radar_alive, start_frame=0, acc_active=False):
    from opendbc.can import CANPacker
    from opendbc.car.mazda import mazdacan
    packer = CANPacker("mazda_2017")
    ret = None
    frames = int(seconds / DT_CTRL)
    for i in range(start_frame, start_frame + frames):
      msgs = [packer.make_can_msg("PEDALS", 0, {"ACC_OFF": 0 if acc_active else 1,
                                                "ACC_ACTIVE": 1 if acc_active else 0})]
      if radar_alive:
        msgs.append(mazdacan.create_acc_command(packer, 0, i, 0., long_active=False, acc_available=True))
      ret, _ = CI.update([(int(i * DT_CTRL * 1e9), [(m[0], m[1], m[2]) for m in msgs])])
    return ret, start_frame + frames

  def test_boot_phase_is_not_a_fault(self):
    # radar broadcasting, teardown not started: engagement blocked quietly, no Cruise Fault
    CI = _interface()
    ret, _ = self._feed_guard(CI, 5.0, radar_alive=True)
    assert not ret.accFaulted
    assert not ret.cruiseState.available

  def test_availability_arrives_with_radar_silence(self):
    CI = _interface()
    ret, n = self._feed_guard(CI, 5.0, radar_alive=True)
    ret, n = self._feed_guard(CI, CarControllerParams.STOCK_RADAR_GUARD_T + 0.5,
                              radar_alive=False, start_frame=n)
    assert not ret.accFaulted
    assert ret.cruiseState.available

  def test_radar_return_after_teardown_is_a_fault(self):
    CI = _interface()
    ret, n = self._feed_guard(CI, 5.0, radar_alive=True)
    ret, n = self._feed_guard(CI, CarControllerParams.STOCK_RADAR_GUARD_T + 0.5,
                              radar_alive=False, start_frame=n)
    ret, n = self._feed_guard(CI, 0.5, radar_alive=True, start_frame=n)
    assert ret.accFaulted
    # availability keys on the latched "was silenced", so a transient return does not
    # yank lateral out from under MADS on top of the fault
    assert ret.cruiseState.available
    # silence restores the clean state
    ret, n = self._feed_guard(CI, CarControllerParams.STOCK_RADAR_GUARD_T + 0.5,
                              radar_alive=False, start_frame=n)
    assert not ret.accFaulted
    assert ret.cruiseState.available

  def test_stock_engagement_inside_the_guard_is_not_reported(self):
    # The radar is still master during the boot phase, so a stock MRCC engage is not ours to
    # report. Availability was already gated; leaking enabled through it opened MADS with
    # every off-switch shut (route 00000057).
    CI = _interface()
    ret, _ = self._feed_guard(CI, 5.0, radar_alive=True, acc_active=True)
    assert not ret.cruiseState.available
    assert not ret.cruiseState.enabled

  def test_engagement_still_live_when_the_guard_lifts_is_not_adopted(self):
    # Silence alone must not turn a pre-existing stock engagement into an openpilot engage:
    # that edge would arrive with no driver input behind it.
    CI = _interface()
    ret, n = self._feed_guard(CI, 5.0, radar_alive=True, acc_active=True)
    ret, n = self._feed_guard(CI, CarControllerParams.STOCK_RADAR_GUARD_T + 0.5,
                              radar_alive=False, acc_active=True, start_frame=n)
    assert ret.cruiseState.available
    assert not ret.cruiseState.enabled

  def test_engagement_after_an_idle_sample_is_adopted(self):
    CI = _interface()
    ret, n = self._feed_guard(CI, 5.0, radar_alive=True, acc_active=True)
    ret, n = self._feed_guard(CI, CarControllerParams.STOCK_RADAR_GUARD_T + 0.5,
                              radar_alive=False, acc_active=True, start_frame=n)
    ret, n = self._feed_guard(CI, 0.2, radar_alive=False, acc_active=False, start_frame=n)
    assert not ret.cruiseState.enabled
    ret, n = self._feed_guard(CI, 0.2, radar_alive=False, acc_active=True, start_frame=n)
    assert ret.cruiseState.enabled


class TestSpeedSignLimit:
  """CAM_TRAFFIC_SIGNS.SPEED_SIGN_ON is a 2-bit field carrying the display unit, not a 1-bit
  on-flag: 1 = limit displayed in mph, 2 = displayed in km/h, 0 = none. Which value an FSC
  emits tracks its market, not the cluster's unit setting. Payloads are real captures: mph
  frames from a US CX-5 2022 (drive_1x local set), km/h frames from a NZ CX-5 (route
  ded445e51c0e1830|00000007--4b5a89a1ce) where the old 1-bit decode at bit 12 read 0 and SLA
  never saw a limit."""

  @pytest.mark.parametrize(("payload", "expected_ms"), [
    ("0000000002005300", 0.0),                 # no limit displayed
    ("0650000002005300", 25 * CV.MPH_TO_MS),   # US 25 mph
    ("0a10000003001300", 40 * CV.MPH_TO_MS),   # US 40 mph
    ("0b50000002005300", 45 * CV.MPH_TO_MS),   # US 45 mph
    ("0a20000002000900", 40 * CV.KPH_TO_MS),   # NZ 40 km/h
    ("0ca0000002000900", 50 * CV.KPH_TO_MS),   # NZ 50 km/h
    ("1920000002010900", 100 * CV.KPH_TO_MS),  # NZ 100 km/h
  ])
  def test_unit_comes_from_the_frame(self, payload, expected_ms):
    CI = _interface()
    ret_sp = None
    for i in range(2):
      _, ret_sp = CI.update([(int(i * DT_CTRL * 1e9), [(0x35F, bytes.fromhex(payload), 2)])])
    assert ret_sp.speedLimit == pytest.approx(expected_ms)

  @pytest.mark.parametrize(("sign_on", "speed_sign"), [
    (1, 120),  # above any real mph posting
    (2, 127),  # all-ones: invalid sentinel
    (3, 50),   # undefined state
    (1, 0),    # displayed-but-zero
  ])
  def test_implausible_frames_read_as_no_limit(self, sign_on, speed_sign):
    from opendbc.can import CANPacker
    packer = CANPacker("mazda_2017")
    msg = packer.make_can_msg("CAM_TRAFFIC_SIGNS", 2, {"SPEED_SIGN_ON": sign_on, "SPEED_SIGN": speed_sign})
    CI = _interface()
    ret_sp = None
    for i in range(2):
      _, ret_sp = CI.update([(int(i * DT_CTRL * 1e9), [(msg[0], msg[1], msg[2])])])
    assert ret_sp.speedLimit == 0.0


class TestCancelUnderBraking:
  """The availability brake-hold exists for brake-only PEDALS samples that arrive with both
  bits low mid-press. A wheel CANCEL turns the MRCC main state off for real and must land
  even with the brake down (route 7f9e3ff336 t+484-488: cancel mashed under braking was
  swallowed until the brake released 4 s later)."""

  def _armed_and_silent(self, CI):
    # get past the two-master guard with the main armed so availability starts True
    from opendbc.can import CANPacker
    packer = CANPacker("mazda_2017")
    guard = CarControllerParams.STOCK_RADAR_GUARD_T + 0.5
    for i in range(int(guard / DT_CTRL)):
      msgs = [packer.make_can_msg("PEDALS", 0, {"ACC_OFF": 1})]
      ret, _ = CI.update([(int(i * DT_CTRL * 1e9), [(m[0], m[1], m[2]) for m in msgs])])
    assert ret.cruiseState.available
    return packer, int(guard / DT_CTRL)

  def _feed(self, CI, packer, n0, seconds, brake, cancel):
    ret = None
    frames = int(seconds / DT_CTRL)
    for i in range(n0, n0 + frames):
      msgs = [packer.make_can_msg("PEDALS", 0, {"ACC_OFF": 0, "BRAKE_ON": int(brake)}),
              packer.make_can_msg("CRZ_BTNS", 0, {"CAN_OFF": int(cancel)})]
      ret, _ = CI.update([(int(i * DT_CTRL * 1e9), [(m[0], m[1], m[2]) for m in msgs])])
    return ret, n0 + frames

  def test_brake_only_dropout_is_held(self):
    CI = _interface()
    packer, n = self._armed_and_silent(CI)
    ret, n = self._feed(CI, packer, n, 1.0, brake=True, cancel=False)
    assert ret.cruiseState.available

  def test_cancel_lands_through_the_brake(self):
    CI = _interface()
    packer, n = self._armed_and_silent(CI)
    ret, n = self._feed(CI, packer, n, 0.3, brake=True, cancel=True)
    assert not ret.cruiseState.available

  def test_cancel_context_outlives_the_press(self):
    # the PEDALS reaction can trail the button: press-and-release while still armed, then the
    # bits drop only after the button is back up -- the context memory has to carry it
    CI = _interface()
    packer, n = self._armed_and_silent(CI)
    ret = None
    for i in range(n, n + 5):  # cancel pressed, PEDALS not yet reacting
      msgs = [packer.make_can_msg("PEDALS", 0, {"ACC_OFF": 1}),
              packer.make_can_msg("CRZ_BTNS", 0, {"CAN_OFF": 1})]
      ret, _ = CI.update([(int(i * DT_CTRL * 1e9), [(m[0], m[1], m[2]) for m in msgs])])
    assert ret.cruiseState.available
    ret, n = self._feed(CI, packer, n + 5, 0.2, brake=True, cancel=False)
    assert not ret.cruiseState.available


class TestCruiseStandstill:
  """PEDALS.STANDSTILL is the PCM's wheel-speed "stopped" bit, not a stock-ACC hold state.

  LongControl's starting_condition is `not should_stop and not cruise_standstill and not
  brake_pressed`, so reporting it under openpilot longitudinal deadlocks every stop: long
  control holds LongCtrlState.stopping (and with it stopAccel) until the car moves, and the
  car cannot move until long control leaves stopping. Both engaged stops on route
  000000fa--6b21bd7e7e (2026-08-25) sat pinned at -1.03 m/s2 through a departing lead, with
  the plan asking for +1.3, until the driver used the gas pedal. Nothing downstream ever ran:
  no RESUME_UNLATCHING pulse and no RES press, since both key off actuators.accel > 0.
  """

  def _standstill(self, alpha_long):
    from opendbc.can import CANPacker
    packer = CANPacker("mazda_2017")
    CI = _interface(alpha_long)
    ret = None
    for i in range(2):  # CANParser registers a message lazily, so the first frame only arms it
      msg = packer.make_can_msg("PEDALS", 0, {"STANDSTILL": 1})
      ret, _ = CI.update([(int(i * DT_CTRL * 1e9), [(msg[0], msg[1], msg[2])])])
    return ret.cruiseState.standstill

  def test_not_reported_under_openpilot_longitudinal(self):
    # the stock MRCC is not in the loop at all here: its radar is silenced and we synthesize
    # its frames, so there is no stock standstill state to report
    assert not self._standstill(alpha_long=True)

  def test_still_reported_with_stock_longitudinal(self):
    # stock long still needs it: it is what drives CC.cruiseControl.resume in controlsd
    assert self._standstill(alpha_long=False)


class TestMazdaTorqueInterceptorState:
  def test_feedback_parser_is_ti_only_on_bus_one_at_50_hz(self):
    assert Bus.body not in _interface(ti=False).can_parsers

    parser = _interface(ti=True).can_parsers[Bus.body]
    assert parser.bus == 1
    assert parser.message_states[0x24A].frequency == 50

  @pytest.mark.parametrize(("overrides", "healthy"), [
    ({}, True),
    ({"VERSION_NUMBER": 16}, True),  # current firmware, captured healthy frame 7f7f100300000030
    ({"VERSION_NUMBER": 0}, False),
    ({"VERSION_NUMBER": 2}, False),
    ({"VERSION_NUMBER": 17}, False),
    ({"STATE": 0}, False),
    ({"STATE": 1}, False),
    ({"STATE": 2}, False),
    ({"VIOL": 1}, False),
    ({"ERROR": 1}, False),
    ({"RAMP_DOWN": 1}, False),
  ])
  def test_feedback_health_controls_temporary_fault(self, overrides, healthy):
    CI = _interface(ti=True)
    ret = _feed_ti(CI, speed_kph=50, **overrides)  # fault only faults at speed
    assert CI.can_parsers[Bus.body].can_valid
    assert CI.CS.ti_lkas_allowed is healthy
    assert ret.steerFaultTemporary is not healthy

  def test_low_speed_ti_dropout_is_not_a_fault(self):
    # captured standstill pattern: TI self-protects to OFF+VIOL=0x11 at stops
    # and while creeping; that is normal, not a steering fault
    CI = _interface(ti=True)
    for speed_kph in (0, 5, 14):  # 14.6 kph dropout observed in rlogs
      ret = _feed_ti(CI, speed_kph=speed_kph, STATE=1, VIOL=0x11)
      assert not CI.CS.ti_lkas_allowed
      assert not ret.steerFaultTemporary

  def test_missing_stale_and_wrong_bus_feedback_are_unhealthy(self):
    CI = _interface(ti=True)
    ret, _ = CI.update([])
    assert not CI.CS.ti_lkas_allowed
    assert not ret.steerFaultTemporary  # not moving: no fault

    ret = _feed_ti(CI, speed_kph=50)  # healthy at speed
    assert CI.CS.ti_lkas_allowed
    assert not ret.steerFaultTemporary

    # feedback only ever arriving on the wrong bus: never healthy, faults at speed
    CI2 = _interface(ti=True)
    fb = _ti_feedback()
    ws = CANPacker("mazda_2017").make_can_msg("WHEEL_SPEEDS", 0, {"FL": 50, "FR": 50, "RL": 50, "RR": 50})
    for i in range(1, 7):
      ret, _ = CI2.update([(i * 20_000_000, [(fb[0], fb[1], 0), ws])])
    assert not CI2.CS.ti_lkas_allowed
    assert ret.steerFaultTemporary

    # stale: healthy stream stops while at speed
    for i in range(5):
      ret, _ = CI.update([((500 + i * 100) * 1_000_000, [])])
    assert not CI.CS.ti_lkas_allowed
    assert ret.steerFaultTemporary

  def test_one_healthy_frame_recovers_after_protocol_error(self):
    CI = _interface(ti=True)
    ret = _feed_ti(CI, speed_kph=50, ERROR=1)
    assert not CI.CS.ti_lkas_allowed
    assert ret.steerFaultTemporary
    ret = _feed_ti(CI, speed_kph=50)
    assert CI.CS.ti_lkas_allowed
    assert not ret.steerFaultTemporary

  @pytest.mark.parametrize(("torque", "pressed"), [(6, False), (7, True), (-7, True)])
  def test_ti_torque_and_pressed_threshold(self, torque, pressed):
    ret = _feed_ti(_interface(ti=True), frames=6, torque=torque)
    assert ret.steeringTorque == torque
    assert ret.steeringPressed is pressed

  def test_ti_suppresses_stock_faults_while_health_drives_temporary_fault(self):
    stock = _interface(ti=False)
    ti = _interface(ti=True)
    stock.update([])
    ti.update([])

    packer = CANPacker("mazda_2017")
    cam_fault = packer.make_can_msg("CAM_LKAS", 2, {"ERR_BIT_1": 1})
    # upstream freshness gate: invalidLkasSetting is only judged on a fresh CAM_LANEINFO
    laneinfo = packer.make_can_msg("CAM_LANEINFO", 2, {"LANE_LINES": 0})
    stock_ret, _ = stock.update([(20_000_000, [cam_fault, laneinfo])])
    ti_ret, _ = ti.update([(20_000_000, [cam_fault, laneinfo, _ti_feedback()])])

    assert stock_ret.invalidLkasSetting
    assert stock_ret.steerFaultPermanent
    assert not stock_ret.steerFaultTemporary
    assert not ti_ret.invalidLkasSetting
    assert not ti_ret.steerFaultPermanent
    assert not ti_ret.steerFaultTemporary
    assert ti.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR

  def test_ti_ready_mirrored_to_carstate_sp(self):
    CI = _interface(ti=True)
    _, ret_sp = CI.update([])
    assert not ret_sp.torqueInterceptorReady

    _, ret_sp = CI.update([(20_000_000, [_ti_feedback()])])
    assert CI.CS.ti_lkas_allowed
    assert ret_sp.torqueInterceptorReady

  def test_ti_not_ready_field_capture(self):
    # Real frame captured from a TI reporting STATE=OFF, VERSION_NUMBER=16, VIOL=0x12
    CI = _interface(ti=True)
    _, ret_sp = CI.update([(20_000_000, [(0x24A, bytes.fromhex("7f7f100112000030"), 1)])])
    assert not CI.CS.ti_lkas_allowed
    assert not ret_sp.torqueInterceptorReady
