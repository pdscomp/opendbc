from dataclasses import dataclass, field
from enum import IntEnum, IntFlag, StrEnum

from opendbc.car import Bus, CarSpecs, DbcDict, DT_CTRL, PlatformConfig, Platforms
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.vin import Vin, is_valid_vin

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
  FSC_SETTLE_T = 10.0          # observed-settled time before the teardown may start
  STOCK_RADAR_ALIVE_T = 0.05   # stock CRZ_INFO runs at 50 Hz; silent this long = torn down
  STOCK_RADAR_GUARD_T = 1.0    # two-master guard: block engagement until silent this long
  RADAR_SESSION_LIMIT_T = 10.0  # per-episode UDS budget: a silent radar gives up here
  # CAM_LANEINFO is a ~2 Hz message (longest period measured 0.563 s across 26+ segments on
  # two cars), so freshness has to be judged against that cadence: a window shorter than one
  # period reads every inter-frame gap as a dropout, zeroes the settle timer each time, and
  # the teardown gate never opens. The window keeps 2.7x margin over the longest observed
  # period and still catches a genuine camera dropout.
  CAM_LANEINFO_PERIOD_T = 0.563
  CAM_LANEINFO_FRESH_T = 1.5

  # RESUME_UNLATCHING pulse width at the release; stock latched releases pulse 0.22-0.38 s,
  # this sits mid-distribution
  RESUME_UNLATCH_T = 0.26

  CANCEL_CONTEXT_T = 0.5       # a wheel CANCEL keeps availability drops landing this long after release

  # The plan flapping across zero at a held standstill (a lead inches forward and stops) used
  # to fire a fresh RESUME_UNLATCHING pulse per flap and re-assert the stop bits mid-pulse, a
  # combination stock never emits (stock pulses exactly once per release, stop bits already
  # dropped). The plan must ask to move this long before the hold releases; stock's releases
  # lag the lead's departure by at least this much (all 23 latched releases show the lead
  # already opening at >= +0.31 m/s at the pulse, ~0.2 s into a typical drive-off).
  RELEASE_DEBOUNCE_T = 0.2

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

  # ACCEL_CMD ceiling while a body-latched release's RESUME_UNLATCHING pulse plays: stock's
  # latched releases peak at +0.24-0.25 m/s2 (raw +182/+195) in the pulse tail. Non-latched
  # pulses are capped at zero instead -- stock's are still <= -0.27 m/s2 when the pulse ends,
  # and both observed SCBS latches (routes 000000fe, 00000100) fired at a zero-cross inside a
  # non-latched pulse.
  ACCEL_RESUME_PULSE_MAX = 0.25  # m/s2, latched releases only

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


class TorqueInterceptorControllerParams:
  STEER_MAX = 600
  STEER_DELTA_UP = 6
  STEER_DELTA_DOWN = 15
  STEER_DRIVER_ALLOWANCE = 15
  STEER_DRIVER_MULTIPLIER = 40
  STEER_DRIVER_FACTOR = 1
  STEER_MAX_RT_DELTA = 192
  STEER_RT_INTERVAL_NS = 250_000_000
  # below this speed the TI gets zero torque: kills standstill wheel wiggle and the
  # TI's own ~30 s self-protection latch, which trips even at low command when stopped
  STANDSTILL_ZERO_SPEED = 1.0  # m/s (~2.2 mph)

  def __init__(self, CP=None):
    if CP is None:
      return
    # 12/frame up-rate: proven on the 2022+ EPS generation (KSD5 via STZ tune). Extended to the
    # CX-8 (K19G, same generation): route 8762fabfa41efd82_00000006--712dfc7c83 showed the base
    # 6/frame limiter saturating on 9.5-22.6% of command frames. Validate on-device: watch for
    # EPS steer faults on hard transitions.
    if CP.flags & MazdaFlags.STEER_TO_ZERO or CP.carFingerprint == "MAZDA_CX8_2022":
      self.STEER_DELTA_UP = 12
    if CP.flags & MazdaFlags.STEER_TO_ZERO:
      # The magnitude envelope does NOT port: the TI's DAC output stage self-protects above
      # ~600 (STATE=OFF + VIOL=0x11, latched ~30 s). First-drive evidence 2026-08-22: 9 cutouts,
      # every at-speed entry preceded by |cmd| >600 in the prior 3 s; none without.
      # STEER_MAX stays 600.
      self.STEER_DELTA_DOWN = 25
      self.STEER_MAX_RT_DELTA = 384


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


class WMI(StrEnum):
  JAPAN_PASSENGER = "JM1"   # Japan-built passenger cars
  JAPAN_CROSSOVER = "JM3"   # Japan-built crossovers
  MEXICO_PASSENGER = "3MZ"  # Mazda de Mexico (Mazda 3)


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mazda_2017', Bus.radar: 'mazda_2017'})
  flags: int = MazdaFlags.GEN1
  wmis: set[WMI] = field(default_factory=set)
  chassis_codes: set[str] = field(default_factory=set)
  years: set[str] = field(default_factory=set)


class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5),
    wmis={WMI.JAPAN_CROSSOVER}, chassis_codes={'KF'}, years={'H', 'J', 'K', 'L', 'M'},  # 2017-21
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=2.93, steerRatio=17.6),
    wmis={WMI.JAPAN_CROSSOVER}, chassis_codes={'TC'}, years={'G', 'H', 'J', 'K', 'L'},  # 2016-20
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0),
    wmis={WMI.JAPAN_PASSENGER, WMI.MEXICO_PASSENGER}, chassis_codes={'BN'}, years={'H', 'J'},  # 2017-18
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5),
    wmis={WMI.JAPAN_PASSENGER}, chassis_codes={'GL'}, years={'H', 'J', 'K', 'L', 'M'},  # 2017-21
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video="https://youtu.be/dA3duO4a0O4")],
    MazdaCarSpecs(mass=4409 * CV.LB_TO_KG, wheelbase=2.93, steerRatio=17.6),
    wmis={WMI.JAPAN_CROSSOVER}, chassis_codes={'TC'}, years={'M', 'N', 'P'},  # 2021-23
  )
  # KG-platform twin of the CX-9. Mass (1850 kg kerb) and wheelbase owner-verified; steerRatio 17.6
  # confirmed via route analysis. Overall dimensions 4900x1840x1730 (no CarParams fields for these).
  MAZDA_CX8_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-8 2022-24")],
    MazdaCarSpecs(mass=1850, wheelbase=2.93, steerRatio=17.6)
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-25")],
    MazdaCX5_2022CarSpecs(mass=3728 * CV.LB_TO_KG, wheelbase=2.698, steerRatio=18.1),  # 15.5 is factory spec; 18.1 from paramsd learner (2.9M samples)
    wmis={WMI.JAPAN_CROSSOVER}, chassis_codes={'KF'}, years={'N', 'P', 'R', 'S'},  # 2022-25
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


def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions) -> set[str]:
  # A donor EPS (steer-to-zero swaps) breaks every exact FW match; the VIN names
  # the chassis through any ECU swap. Runs only after exact and fuzzy FW fail.
  # Model line is VIN positions 4-5, model year code is position 10.
  if is_valid_vin(vin):
    vin_obj = Vin(vin)
    chassis_code = vin_obj.vds[0:2]
    year = vin_obj.vis[0]

    candidates = set()
    for platform in CAR:
      platform_config = platform.config
      if vin_obj.wmi in platform_config.wmis and chassis_code in platform_config.chassis_codes and year in platform_config.years:
        candidates.add(platform)

    if len(candidates) == 1:
      carlog.error(f"Fingerprinted {next(iter(candidates))} by VIN")
      return {str(c) for c in candidates}

    # a known Mazda WMI that names no platform identified an unsupported model
    # (BP, DM, KE, out-of-range years): never second-guess it with the engine.
    # WMIs outside the table (e.g. 7MM, CX-50) keep the fallback and its
    # collision risk; pinned by test.
    if vin_obj.wmi in {wmi for platform in CAR for wmi in platform.config.wmis}:
      return set()

  # Oceania VINs encode no model year and never decode; engine firmware is
  # unique per platform (asserted by test), so it names the chassis instead.
  # A lone responding address is not a car to name.
  if len(live_fw_versions) < 2:
    return set()

  engine_fw = live_fw_versions.get((0x7e0, None), set())
  candidates = set()
  for platform, ecus in offline_fw_versions.items():
    if engine_fw & set(ecus.get((Ecu.engine, 0x7e0, None), [])):
      candidates.add(platform)

  if len(candidates) == 1:
    carlog.error(f"Fingerprinted {next(iter(candidates))} by engine firmware")
  return {str(c) for c in candidates}


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[A-Z0-9-]{11,16}\x00{8,13}",
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

DBC = CAR.create_dbc_map()
