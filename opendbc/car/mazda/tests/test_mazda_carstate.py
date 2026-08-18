import pytest

from opendbc.car import DT_CTRL, gen_empty_fingerprint
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, CarControllerParams

CAM_LANEINFO = 0x440

# Real CAM_LANEINFO prefixes, captured on two CX-5 2022s running the same FSC firmware
# (GSH7-67XK2-U). Only byte 1 differs: bit 5 is BIT2, bit 6 is NO_ERR_BIT.
BOOTING = bytes([0x42, 0b01000001, 0, 0, 0, 0, 0, 0])       # NO_ERR_BIT set: still booting
SETTLED = bytes([0x42, 0b00000001, 0, 0, 0, 0, 0, 0])       # markers clear: settled
BIT2_LATCHED = bytes([0x41, 0b00100001, 0, 0, 0, 0, 0, 0])  # BIT2 stuck high for a whole cycle
FAULTED = bytes([0x42, 0b00000001, 0, 0, 0, 0x01, 0, 0])    # ERR_BIT (bit 40) set


def _interface(alpha_long=True, car=CAR.MAZDA_CX5_2022):
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(car, fingerprint, [], alpha_long=alpha_long,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, car, fingerprint, [],
                                     alpha_long=alpha_long, is_release_sp=False, docs=False)
  return CarInterface(CP, CP_SP)


def test_invalid_lkas_gate_only_applies_to_stock_gen1_eps():
  # Stock LKAS off (no CAM_LANEINFO): the stock GEN1 EPS keeps the engagement
  # gate; 2022-EPS cars are exempt (we present LKAS-on to the EPS ourselves).
  gen1_ret, _ = _interface(car=CAR.MAZDA_CX5).update([])
  cx5_2022_ret, _ = _interface().update([])

  assert gen1_ret.invalidLkasSetting
  assert not cx5_2022_ret.invalidLkasSetting


def _feed(CI, payload, seconds):
  frames = int(seconds / DT_CTRL)
  for i in range(frames):
    CI.update([(int(i * DT_CTRL * 1e9), [(CAM_LANEINFO, payload, 2)])])
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

  def test_gate_starts_closed_before_any_camera_frame(self):
    # the parser reads all-zero before the first frame, which would otherwise look settled
    CI = _interface()
    for i in range(int(CarControllerParams.FSC_SETTLE_T * 2 / DT_CTRL)):
      CI.update([(int(i * DT_CTRL * 1e9), [])])
    assert not CI.CS.fsc_settled


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

  def _feed_guard(self, CI, seconds, radar_alive, start_frame=0):
    from opendbc.can import CANPacker
    from opendbc.car.mazda import mazdacan
    packer = CANPacker("mazda_2017")
    ret = None
    frames = int(seconds / DT_CTRL)
    for i in range(start_frame, start_frame + frames):
      msgs = [packer.make_can_msg("PEDALS", 0, {"ACC_OFF": 1})]
      if radar_alive:
        msgs.append(mazdacan.create_acc_command(packer, 0, i, 0., False, True,
                                                stopping=False, resume_unlatching=False))
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
