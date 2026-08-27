#!/usr/bin/env python3
from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.radar_interface import RadarInterface
from opendbc.car.mazda.values import CAR, DBC, LKAS_LIMITS, STEER_TO_ZERO_EPS_FW, MazdaFlags, MazdaSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]

    ret.radarUnavailable = Bus.radar not in DBC[candidate]

    # 2022+ CX-5 EPS can steer to zero and has no hands-off lockout. Detected by EPS firmware
    # rather than by model, so an EPS swapped into an older Mazda is recognized as what it is.
    steer_to_zero = candidate == CAR.MAZDA_CX5_2022 or \
      any(fw.ecu == 'eps' and fw.fwVersion in STEER_TO_ZERO_EPS_FW for fw in car_fw)
    if steer_to_zero:
      ret.flags |= MazdaFlags.STEER_TO_ZERO.value
    else:
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    # CX-9 2021 verified against route 00000004--97e4328f4f: same message set at the same
    # rates, CRZ_INFO checksum holds on all 54k stock frames, radar UDS at 0x764, and the
    # same FSC camera firmware (GSH7-67XK2-U) as the CX-5 2022 this was developed on.
    ret.alphaLongitudinalAvailable = candidate in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021)
    ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.LONG.value
      # engagement stays with the car: the driver SETs on the wheel, the body ECU raises
      # PEDALS.ACC_ACTIVE, and the dash-owned CRZ_EVENTS setpoint survives the radar teardown
      ret.pcmCruise = True
      ret.radarUnavailable = True
      ret.stopAccel = -1.024  # stock MRCC holds raw -1024 at a stop; the plan parks here and we send it as-is
      ret.longitudinalActuatorDelay = 0.36  # measured ~0.3 s dead time + ~0.3 s first-order lag

    # Older Mazdas are dashcam only for one reason: their EPS locks steering out after ~5 s of
    # hands-off and below 45 kph. That is a property of the EPS, not of the car, so a car with
    # the 2022+ EPS swapped in is controllable and lifts with it. Longitudinal stays keyed on the
    # model above: the radar and camera are not part of an EPS swap.
    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021) and not steer_to_zero

    ret.enableBsm = 0x477 in fingerprint[0]

    # command-to-torque lag is EPS firmware, so it follows the EPS. lagd learns the rest
    # (0.338 total on a CX-5 2022; initial = this + 0.2)
    ret.steerActuatorDelay = 0.14 if steer_to_zero else 0.1
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate == CAR.MAZDA_CX8_2022:
      # Owner-validated StarPilot generic-path gains.
      # Fork controller defaults: kp 0.8 / ki 0.15 (latcontrol_torque.py module constants)
      # Old sunnypilot defaults:  kp 1.0 / ki 0.3
      ret.lateralTuning.torque.kp = 0.6
      ret.lateralTuning.torque.ki = 0.35

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    ret.intelligentCruiseButtonManagementAvailable = True

    return ret
