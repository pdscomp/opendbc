from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, DT_CTRL, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_FACTOR = 1         # from dbc
  STEER_STEP = 1  # 100 Hz

  ACCEL_MAX = 2.0   # m/s2
  ACCEL_MIN = -3.5  # m/s2

  # Longitudinal message rates, 100 Hz frames
  LONG_STEP = 2        # CRZ_INFO/CRZ_CTRL at 50 Hz, matching stock
  RADAR_STEP = 10      # radar static + track frames at 10 Hz
  RADAR_UDS_STEP = 50  # radar UDS traffic at 2 Hz: session control or tester present

  # Radar session timing, seconds. The FSC's radar-presence check faulted when the radar
  # went quiet 1.9 s after the camera's boot settle and passed from 5.8 s
  # (docs/mazda-alpha-long-setup-teardown.md), hence the 10 s settle requirement.
  FSC_SETTLE_T = 10.0         # observed-settled time before the teardown may start
  STOCK_RADAR_ALIVE_T = 0.05  # stock CRZ_INFO runs at 50 Hz; silent this long = torn down
  STOCK_RADAR_GUARD_T = 1.0   # two-master guard: block engagement until silent this long

  RESUME_UNLATCH_T = 0.20      # RESUME_UNLATCHING pulse width at the release

  CANCEL_CONTEXT_T = 0.5       # a wheel CANCEL keeps availability drops landing this long after release

  # A marginal vision lead flickers leadVisible faster than the camera can be shown a track
  # appearing and vanishing (route 6bb2dc61c4 t+400: 6 toggles in 1.4 s on a 120 m lead), so the
  # advertised lead only follows a state that has held steady, the way Hyundai debounces its
  # lead bit for 50 frames
  LEAD_DEBOUNCE_T = 0.5

  # Stock relaxes its standstill command the instant the body ECU takes the hold over, not on any
  # schedule: across 13 stock holds >= 4.5 s the relax and GEAR.BRAKE_HOLD agreed to within
  # +-0.02 s in all 9 where both were visible, and the latch itself landed anywhere from 0.01 s
  # to 7.6 s after standstill. The command through the hold is the plan's own, which parks at
  # CP.stopAccel; this is only the relaxed value we send once the car has the brakes.
  ACCEL_HOLD_LATCHED = -0.001  # m/s2

  # Command slew limits, m/s3, on the plan-following command only. Asymmetric on purpose: the
  # windup limit is what keeps the command from dumping the brake in one frame (the driver-felt
  # problem), while a tight winddown limit would delay real braking for no measured benefit.
  # 4.0 sits above the p99 of the plan's own up-slew on the reporter's route (p99 +3.2, p99.9
  # +6.3, max +34 m/s3), so it only clips state-transition steps. Toyota uses 4.0 both ways.
  ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL     # m/s2 per frame
  ACCEL_WINDDOWN_LIMIT = -10.0 * DT_CTRL  # m/s2 per frame, clips only the p99.9+ steps

  def __init__(self, CP):
    # Gate the higher-authority steering tune on the CX-5 2022+ EPS, not the car model, so the
    # CX-9 that shares this EPS and CX-5-EPS swaps keep it. TI also lowers minSteerSpeed, so
    # the explicit EPS capability flag is the only safe identity for this tune.
    if CP.flags & MazdaFlags.STEER_TO_ZERO:
      self.STEER_MAX = 1200        # theoretical max_steer 2047; EPS clips above ceiling per speed
      # 1200 below 32 mph for full low-speed authority and feedforward overshoot.
      # 800 above for smoother highway steering with 22% PID headroom above EPS ceiling (620).
      self.STEER_MAX_LOOKUP = ([0., 14.2, 14.5], [1200, 1200, 800])
      # EPS hardware rate limit: 12 units/frame at all speeds (4-unit quantization, max 3 steps).
      self.STEER_DELTA_UP = 12
      self.STEER_DELTA_DOWN = 25
      self.STEER_DRIVER_MULTIPLIER = 15   # weight driver torque (tuned for the CX-5 EPS; upstream stock is 1)
    else:
      self.STEER_MAX = 800         # theoretical max_steer 2047
      self.STEER_DELTA_UP = 10
      self.STEER_DELTA_DOWN = 25
      self.STEER_DRIVER_MULTIPLIER = 1    # upstream stock


@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


@dataclass(frozen=True, kw_only=True)
class MazdaCX5_2022CarSpecs(CarSpecs):
  tireStiffnessFactor: float = 1.0


class MazdaFlags(IntFlag):
  # Static flags
  # Gen 1 hardware: same CAN messages and same camera
  GEN1 = 1
  STEER_TO_ZERO = 2
  TORQUE_INTERCEPTOR = 4


class MazdaSafetyFlags(IntFlag):
  LONG = 1
  TORQUE_INTERCEPTOR = 2


class TorqueInterceptorState(IntEnum):
  DISCOVER = 0
  OFF = 1
  DRIVER_OVER = 2
  RUN = 3


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mazda_2017', Bus.radar: 'mazda_2017'})
  flags: int = MazdaFlags.GEN1


class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5)
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=2.93, steerRatio=17.6)
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0)
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5)
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video="https://youtu.be/dA3duO4a0O4")],
    MazdaCarSpecs(mass=4409 * CV.LB_TO_KG, wheelbase=2.93, steerRatio=17.6)
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-25")],
    MazdaCX5_2022CarSpecs(mass=3728 * CV.LB_TO_KG, wheelbase=2.698, steerRatio=18.1),  # 15.5 is factory spec; 18.1 from paramsd learner (2.9M samples)
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph


# EPS firmware versions with steer-to-zero capability (2022+ CX-5 EPS). Matched against
# car_fw rather than the fingerprinted platform so the same EPS swapped into another Mazda
# keeps full-speed steering. Keep in sync with the CAR.MAZDA_CX5_2022 (Ecu.eps, 0x730) block
# in fingerprints.py.
STEER_TO_ZERO_EPS_FW = {
  b'KBST-3210X-A-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  b'KSD5-3210X-C-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
}


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
