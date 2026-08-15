import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, LKAS_LIMITS, STEER_TO_ZERO_EPS_FW, MazdaFlags, MazdaSafetyFlags
from opendbc.sunnypilot.car.interfaces import setup_interfaces

Ecu = structs.CarParams.Ecu

# The steer-to-zero EPS a swap donates, and a stock pre-2022 CX-5 EPS for contrast
SWAPPED_EPS_FW = sorted(STEER_TO_ZERO_EPS_FW)[0]
STOCK_CX5_EPS_FW = b'K319-3210X-A-00' + b'\x00' * 9


def _eps_fw(version: bytes) -> list[structs.CarParams.CarFw]:
  fw = structs.CarParams.CarFw()
  fw.ecu = Ecu.eps
  fw.address = 0x730
  fw.subAddress = 0
  fw.fwVersion = version
  return [fw]


def _params(candidate, car_fw=None, alpha_long=False):
  return CarInterface.get_params(candidate, {0: {}, 1: {}, 2: {}}, car_fw or [],
                                 alpha_long, is_release=False, docs=False)


def _setup_ti(CP, enabled=True):
  setup_interfaces(CarInterface, CP, structs.CarParamsSP(), [{"TorqueInterceptorEnabled": enabled}])


class TestMazdaEpsSwap:
  """A 2022+ CX-5 EPS swapped into an older Mazda brings the EPS-derived behaviour with it.

  Pre-2022 Mazdas are dashcam only because their EPS locks steering out after ~5 s hands-off
  and below 45 kph. That lockout lives in the EPS, so the swap lifts it. Everything keyed on
  the radar, camera or vehicle dynamics must stay keyed on the model.
  """

  def test_stock_older_mazda_is_dashcam_only(self):
    CP = _params(CAR.MAZDA_CX5, _eps_fw(STOCK_CX5_EPS_FW))
    assert CP.dashcamOnly
    assert CP.minSteerSpeed == pytest.approx(LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS)
    assert CP.steerActuatorDelay == pytest.approx(0.1)
    assert CP.flags & MazdaFlags.GEN1
    assert not CP.flags & MazdaFlags.STEER_TO_ZERO

  def test_swapped_eps_lifts_dashcam_and_the_speed_floor(self):
    CP = _params(CAR.MAZDA_CX5, _eps_fw(SWAPPED_EPS_FW))
    assert not CP.dashcamOnly
    assert CP.minSteerSpeed == 0
    assert CP.steerActuatorDelay == pytest.approx(0.14)
    assert CP.flags & MazdaFlags.STEER_TO_ZERO

  def test_swapped_eps_does_not_unlock_longitudinal(self):
    # the radar and camera are not part of an EPS swap, and this car keeps its own pre-2022 pair
    CP = _params(CAR.MAZDA_CX5, _eps_fw(SWAPPED_EPS_FW), alpha_long=True)
    assert not CP.alphaLongitudinalAvailable
    assert not CP.openpilotLongitudinalControl

  def test_swapped_eps_keeps_the_real_vehicle_specs(self):
    # the whole point of fixing detection is that the user no longer forces MAZDA_CX5_2022 and
    # inherits its mass, steer ratio and tire stiffness
    swapped = _params(CAR.MAZDA_CX5, _eps_fw(SWAPPED_EPS_FW))
    cx5_2022 = _params(CAR.MAZDA_CX5_2022)
    assert swapped.mass != cx5_2022.mass
    assert swapped.steerRatio != cx5_2022.steerRatio
    assert swapped.tireStiffnessFactor != cx5_2022.tireStiffnessFactor

  def test_supported_platforms_are_unchanged(self):
    cx5_2022 = _params(CAR.MAZDA_CX5_2022)
    assert not cx5_2022.dashcamOnly
    assert cx5_2022.minSteerSpeed == 0
    assert cx5_2022.steerActuatorDelay == pytest.approx(0.14)
    assert cx5_2022.alphaLongitudinalAvailable

    # the CX-9 2021 is supported without the CX-5 EPS, so it keeps the 45 kph floor
    cx9_2021 = _params(CAR.MAZDA_CX9_2021)
    assert not cx9_2021.dashcamOnly
    assert cx9_2021.minSteerSpeed == pytest.approx(LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS)
    assert cx9_2021.steerActuatorDelay == pytest.approx(0.1)

  def test_docs_are_generated_without_firmware(self):
    # car_fw is empty when building CARS.md, so the docs must keep advertising dashcam mode
    for candidate in (CAR.MAZDA_CX5, CAR.MAZDA_CX9, CAR.MAZDA_3, CAR.MAZDA_6):
      CP = CarInterface.get_params(candidate, {0: {}, 1: {}, 2: {}}, [], False,
                                   is_release=False, docs=True)
      assert CP.dashcamOnly, candidate


class TestMazdaTorqueInterceptorParams:
  def test_flag_values_are_stable(self):
    assert MazdaFlags.GEN1.value == 1
    assert MazdaFlags.STEER_TO_ZERO.value == 2
    assert MazdaFlags.TORQUE_INTERCEPTOR.value == 4
    assert MazdaSafetyFlags.LONG.value == 1
    assert MazdaSafetyFlags.TORQUE_INTERCEPTOR.value == 2

  def test_disabled_param_keeps_stock_params(self):
    CP = _params(CAR.MAZDA_CX5, _eps_fw(STOCK_CX5_EPS_FW))
    before = (CP.flags, CP.safetyConfigs[0].safetyParam, CP.minSteerSpeed, CP.steerAtStandstill)
    _setup_ti(CP, enabled=False)
    assert (CP.flags, CP.safetyConfigs[0].safetyParam, CP.minSteerSpeed, CP.steerAtStandstill) == before

  def test_enabled_param_composes_flags_and_long_safety(self):
    CP = _params(CAR.MAZDA_CX5_2022, alpha_long=True)
    _setup_ti(CP)
    assert CP.flags & MazdaFlags.GEN1
    assert CP.flags & MazdaFlags.STEER_TO_ZERO
    assert CP.flags & MazdaFlags.TORQUE_INTERCEPTOR
    assert CP.safetyConfigs[0].safetyParam == MazdaSafetyFlags.LONG | MazdaSafetyFlags.TORQUE_INTERCEPTOR
    assert CP.minSteerSpeed == 0
    assert CP.steerAtStandstill

  def test_ti_zero_speed_does_not_claim_steer_to_zero_eps(self):
    CP = _params(CAR.MAZDA_CX5, _eps_fw(STOCK_CX5_EPS_FW))
    _setup_ti(CP)
    assert CP.flags & MazdaFlags.TORQUE_INTERCEPTOR
    assert not CP.flags & MazdaFlags.STEER_TO_ZERO
    assert not CP.dashcamOnly
    assert CP.minSteerSpeed == 0

  def test_non_gen1_configuration_is_rejected(self):
    CP = _params(CAR.MAZDA_CX5)
    CP.flags &= ~MazdaFlags.GEN1.value
    with pytest.raises(ValueError, match="Mazda GEN1"):
      _setup_ti(CP)


class TestMazdaTorqueInterceptorDbc:
  @pytest.mark.parametrize(("torque", "expected"), [
    (-600, "05a805a8c461ce60"),
    (0, "08000800c461ce60"),
    (600, "0a580a58c461ce60"),
  ])
  def test_command_layout_round_trips(self, torque, expected):
    packer = CANPacker("mazda_2017")
    msg = packer.make_can_msg("CAM_LKAS2", 1, {
      "LKAS_REQUEST": torque,
      "CHKSUM": torque,
      "KEY": 0xC461CE60,
    })
    assert msg == (0x249, bytes.fromhex(expected), 1)

    parser = CANParser("mazda_2017", [("CAM_LKAS2", 0)], 1)
    parser.update([(1, [msg])])
    assert parser.vl["CAM_LKAS2"] == {
      "LKAS_REQUEST": torque,
      "CHKSUM": torque,
      "KEY": 0xC461CE60,
    }
