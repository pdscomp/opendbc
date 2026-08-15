from opendbc.car.can_definitions import CanData
from opendbc.car.mazda.values import Buttons, MazdaFlags

# Radar frames the body ECU expects to keep receiving for stop-and-go to work. Byte-exact
# captures from a 0x764 radar with no objects in view; only the counter nibble in the last
# byte changes. 0x364 carries the lead we are following, if any.
RADAR_STATIC_MSG = (0x499, bytes.fromhex("0008c00000000000"))
RADAR_TRACK_MSGS = {
  0x361: bytes.fromhex("fff7fefe1fc00080"),
  0x362: bytes.fromhex("fff7fefe1fc78c80"),
  0x363: bytes.fromhex("fff7fefe1fc00000"),
  0x364: bytes.fromhex("fff7fefe1fc00000"),
  0x365: bytes.fromhex("fff7fe7ffbff3fc0"),
  0x366: bytes.fromhex("fff7fe7ffbff3fc0"),
}
LEAD_TRACK_ADDR = 0x364
# An occupied track slot, captured from a 0x764 radar holding a stopped lead at 10.25 m.
# create_lead_track only rewrites DIST_OBJ and RELV_OBJ; the rest is the radar's track-valid
# pattern, which is not understood well enough to synthesize.
LEAD_TRACK_TEMPLATE = bytes.fromhex("0a4000001dc00000")
LEAD_TRACK_DIST = 10.25   # m, the template's own range
DIST_OBJ_SCALE = 0.0625   # m per bit, DIST_OBJ and RELV_OBJ share it


def crz_info_checksum(dat: bytes) -> int:
  # Inverted sum of the first seven bytes; the radar leaves the STOPPING bit out of the
  # sum. Verified against 1.94M stock frames, including every stop-bit frame.
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04)) & 0xFF)) & 0xFF


def create_acc_command(packer, bus, counter, accel, long_active, acc_available, stopping, resume_unlatching):
  # CRZ_INFO stands in for the disabled radar's accel command frame. While MRCC is armed
  # but not engaged, stock advertises ACC_SET_ALLOWED with a zero command so the dash
  # accepts SET; with the main switch off it broadcasts a static standby pattern with the
  # command field pegged high.
  values = {
    "STATUS": 1,
    "STATIC_1": 0x7ff,
    "CTR1": counter % 16,
  }
  if long_active or acc_available:
    values.update({
      "ACCEL_CMD": accel,
      "ACC_ACTIVE": int(long_active),
      "ACC_SET_ALLOWED": 1,
      "NEW_SIGNAL_7": 1,
      "STOPPING": int(stopping),
      "STOPPING_2": int(stopping),
      "RESUME_UNLATCHING": int(resume_unlatching),
    })
  else:
    values["ACCEL_CMD"] = 4.094  # standby pattern, raw 8190

  dat = packer.make_can_msg("CRZ_INFO", bus, values)[1]
  values["CHKSUM"] = crz_info_checksum(dat)
  return packer.make_can_msg("CRZ_INFO", bus, values)


def create_crz_ctrl(packer, bus, long_active, acc_available, gap_setting, radar_has_lead, stop_go_phase, acc_active_2):
  # CRZ_CTRL stands in for the disabled radar's cruise-state frame. stop_go_phase mirrors
  # stock's stop-and-go progression through RADAR_LEAD_RELATIVE_DISTANCE (see the DBC
  # comment); gap_setting mirrors the driver's distance setting on the dash.
  values = {
    "MSG_1_INV": 1,
    "MSG_1_INV_COPY": 1,
    "NEW_SIGNAL_8": 1,
    "CRZ_ACTIVE": int(long_active),
    "CRZ_AVAILABLE": int(long_active or acc_available),
    "DISTANCE_SETTING": gap_setting,
    "RADAR_HAS_LEAD": int(radar_has_lead),
    "RADAR_LEAD_RELATIVE_DISTANCE": stop_go_phase,
    "ACC_ACTIVE_2": int(acc_active_2),
  }
  return packer.make_can_msg("CRZ_CTRL", bus, values)


def create_lead_track(d_rel: float, v_rel: float) -> bytes:
  """Place the lead we are following on the track slot the camera reads.

  A stock radar re-measures every track every 100 ms, so its range and range rate move with
  the lead even at a standstill. Repeating one frozen frame instead makes the camera latch an
  SCBS fault the moment a standstill hold releases: it is told an object sits at a fixed range
  with zero closing speed while the car is commanded to drive off, which its own view of the
  lead pulling away contradicts. RELV_OBJ carries the same sign as vRel, positive opening.
  """
  dist = round(min(max(d_rel, 0.), 255.875) / DIST_OBJ_SCALE)
  relv = round(min(max(v_rel, -64.), 63.9375) / DIST_OBJ_SCALE) & 0x7ff
  dat = bytearray(LEAD_TRACK_TEMPLATE)
  dat[0] = dist >> 4
  dat[1] = ((dist & 0xf) << 4) | (dat[1] & 0x0f)
  dat[3] = relv >> 3
  dat[4] = ((relv & 0x7) << 5) | (dat[4] & 0x1f)
  return bytes(dat)


def create_radar_frames(bus, counter, lead):
  """lead is the (dRel, vRel) of the object to advertise on 0x364, or None for an empty slot."""
  frames = [CanData(RADAR_STATIC_MSG[0], RADAR_STATIC_MSG[1], bus)]
  for addr, dat in RADAR_TRACK_MSGS.items():
    if lead is not None and addr == LEAD_TRACK_ADDR:
      dat = create_lead_track(*lead)
    frames.append(CanData(addr, dat[:7] + bytes([(dat[7] & 0xf0) | (counter % 16)]), bus))
  return frames


def create_steering_control(packer, CP, frame, apply_torque, lkas):

  tmp = apply_torque + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  er1 = int(lkas["ERR_BIT_1"])
  lnv = 0
  ldw = 0
  er2 = int(lkas["ERR_BIT_2"])

  # Some older models do have these, newer models don't.
  # Either way, they all work just fine if set to zero.
  steering_angle = 0
  b2 = 0

  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2

  ctr = frame % 16
  # bytes:     [    1  ] [ 2 ] [             3               ]  [           4         ]
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)

  # bytes      [ 5 ] [ 6 ] [    7   ]
  csum = csum - ahi - amd - alo - b2

  if ahi == 1:
    csum = csum + 15

  if csum < 0:
    if csum < -256:
      csum = csum + 512
    else:
      csum = csum + 256

  csum = csum % 256

  values = {}
  if CP.flags & MazdaFlags.GEN1:
    values = {
      "LKAS_REQUEST": apply_torque,
      "CTR": ctr,
      "ERR_BIT_1": er1,
      "LINE_NOT_VISIBLE": lnv,
      "LDW": ldw,
      "BIT_1": b1,
      "ERR_BIT_2": er2,
      "STEERING_ANGLE": steering_angle,
      "ANGLE_ENABLED": b2,
      "CHKSUM": csum
    }

  return packer.make_can_msg("CAM_LKAS", 0, values)


def create_ti_steering_control(packer, apply_torque):
  return packer.make_can_msg("CAM_LKAS2", 1, {
    "LKAS_REQUEST": apply_torque,
    "CHKSUM": apply_torque,
    "KEY": 0xC461CE60,
  })


def create_alert_command(packer, cam_msg: dict, ldw: bool, steer_required: bool):
  values = {s: cam_msg[s] for s in [
    "LINE_VISIBLE",
    "LINE_NOT_VISIBLE",
    "LANE_LINES",
    "BIT1",
    "BIT2",
    "BIT3",
    "NO_ERR_BIT",
    "S1",
    "S1_HBEAM",
  ]}
  values.update({
    # TODO: what's the difference between all these? do we need to send all?
    "HANDS_WARN_3_BITS": 0b111 if steer_required else 0,
    "HANDS_ON_STEER_WARN": steer_required,
    "HANDS_ON_STEER_WARN_2": steer_required,

    # TODO: right lane works, left doesn't
    # TODO: need to do something about L/R
    "LDW_WARN_LL": 0,
    "LDW_WARN_RL": 0,
  })
  return packer.make_can_msg("CAM_LANEINFO", 0, values)


def create_button_cmd(packer, CP, counter, button):

  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)
  inc = int(button == Buttons.SET_PLUS)
  dec = int(button == Buttons.SET_MINUS)

  if CP.flags & MazdaFlags.GEN1:
    values = {
      "CAN_OFF": can,
      "CAN_OFF_INV": (can + 1) % 2,

      "SET_P": inc,
      "SET_P_INV": (inc + 1) % 2,

      "RES": res,
      "RES_INV": (res + 1) % 2,

      "SET_M": dec,
      "SET_M_INV": (dec + 1) % 2,

      "DISTANCE_LESS": 0,
      "DISTANCE_LESS_INV": 1,

      "DISTANCE_MORE": 0,
      "DISTANCE_MORE_INV": 1,

      "MODE_X": 0,
      "MODE_X_INV": 1,

      "MODE_Y": 0,
      "MODE_Y_INV": 1,

      "BIT1": 1,
      "BIT2": 1,
      "BIT3": 1,
      "CTR": (counter + 1) % 16,
    }

    return packer.make_can_msg("CRZ_BTNS", 0, values)
