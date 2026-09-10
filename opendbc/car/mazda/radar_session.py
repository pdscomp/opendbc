"""Mazda radar diagnostic-session ownership, independent of the acceleration controller."""
from enum import StrEnum

from opendbc.car import DT_CTRL, make_tester_present_msg, uds
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.mazda.values import CarControllerParams

RADAR_ADDR = 0x764
RADAR_BUS = 0
RADAR_SESSION_LIMIT_FRAMES = int(CarControllerParams.RADAR_SESSION_LIMIT_T / DT_CTRL)
# Require more than one fresh stock frame before allowing a process restart.
RADAR_RESTORE_FRAMES = 2 * round(CarControllerParams.STOCK_RADAR_ALIVE_T / DT_CTRL)


def create_radar_session_msg(session_type: int) -> CanData:
  return CanData(RADAR_ADDR, bytes([2, uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, session_type, 0, 0, 0, 0, 0]), RADAR_BUS)


class RadarSessionState(StrEnum):
  STOCK = "stock"
  SILENCING = "silencing"
  SILENCED = "silenced"
  HANDBACK = "handback"


class RadarSessionManager:
  """Keep request scheduling, radar ownership and restoration results in one place.

  Silence is evidence only while the vehicle bus is healthy. A requested teardown
  uses the short traffic window; adoption without a request uses the longer guard.
  A restoration timeout is a failure, never proof that stock control recovered.
  """

  def __init__(self):
    self.state = RadarSessionState.STOCK
    self.state_frames = 0
    self.silencing_failed = False
    self.handback_completed = False
    self.handback_failed = False
    self.programming_sent = False
    self.programming_confirmed = False
    self.default_sent = False
    self.default_confirmed = False
    self.stock_frames = 0
    self.replacement_active = False
    self.diagnostic_message: CanData | None = None

  def _transition(self, state: RadarSessionState, reason: str) -> None:
    if state != self.state:
      carlog.info({"event": "mazdaRadarSession", "from": self.state, "to": state, "reason": reason,
                   "programmingConfirmed": self.programming_confirmed, "defaultConfirmed": self.default_confirmed})
      self.state = state
      self.state_frames = 0
      if state == RadarSessionState.SILENCING:
        self.programming_sent = self.programming_confirmed = False
      elif state == RadarSessionState.HANDBACK:
        self.default_sent = self.default_confirmed = False
        self.stock_frames = 0

  def update(self, gate_passed: bool, stock_radar_alive: bool, handback: bool,
             standstill: bool, session_refused: bool, stock_radar_gone: bool, *,
             bus_healthy: bool = True, session_response: int = 0, frame: int = 0) -> RadarSessionState:
    # the controller's frame is the one clock for every cadence, the UDS schedule included
    self.frame = frame
    self.diagnostic_message = None
    self.state_frames += 1
    self.stock_frames = self.stock_frames + 1 if bus_healthy and stock_radar_alive else 0
    if self.state == RadarSessionState.SILENCING and self.programming_sent and session_response == uds.SESSION_TYPE.PROGRAMMING:
      self.programming_confirmed = True
    if self.state == RadarSessionState.HANDBACK and self.default_sent and session_response == uds.SESSION_TYPE.DEFAULT:
      self.default_confirmed = True

    if handback:
      if self.state in (RadarSessionState.SILENCING, RadarSessionState.SILENCED):
        self._transition(RadarSessionState.HANDBACK, "requested")
      elif self.state == RadarSessionState.STOCK and not self.handback_completed:
        if not self.programming_sent and bus_healthy and stock_radar_alive:
          self.handback_completed = self.stock_frames >= RADAR_RESTORE_FRAMES
        else:
          self._transition(RadarSessionState.HANDBACK, "restore uncertain ownership")

    # A withdrawn request does not leave HANDBACK: the in-flight default request completes
    # before another programming request, so a toggle reversal never alternates sessions.
    if self.state == RadarSessionState.HANDBACK:
      # A response alone cannot prove that periodic radar traffic has resumed.
      if self.default_sent and self.stock_frames >= RADAR_RESTORE_FRAMES:
        self.handback_completed = True
        self.handback_failed = False
        self._transition(RadarSessionState.STOCK, "stock traffic restored")
        self._update_replacement(stock_radar_alive, bus_healthy, stock_radar_gone)
        return self.state
      elif self.state_frames >= RADAR_SESSION_LIMIT_FRAMES and not self.handback_failed:
        self.handback_failed = True
        carlog.error({"event": "mazdaRadarRestoreFailed", "reason": "stock traffic did not recover"})
      # After the bounded request budget, stop diagnostics but continue neutral replacement
      # traffic while quiet. A late stock recovery can still complete the handback.
      if not self.handback_failed and (not self.default_sent or not stock_radar_alive) and \
         self.frame % CarControllerParams.RADAR_UDS_STEP == 0:
        self.diagnostic_message = create_radar_session_msg(uds.SESSION_TYPE.DEFAULT)
        self.default_sent = True
      self._update_replacement(stock_radar_alive, bus_healthy, stock_radar_gone)
      return self.state

    if handback or self.handback_completed:
      self._update_replacement(stock_radar_alive, bus_healthy, stock_radar_gone)
      return self.state

    if self.state == RadarSessionState.SILENCED and stock_radar_alive:
      self._transition(RadarSessionState.STOCK, "stock radar returned")

    if self.state == RadarSessionState.STOCK and gate_passed and bus_healthy and not self.silencing_failed:
      if stock_radar_gone:
        self._transition(RadarSessionState.SILENCED, "adopt quiet radar on live bus")
      elif standstill and stock_radar_alive:
        self._transition(RadarSessionState.SILENCING, "parked takeover")

    if self.state == RadarSessionState.SILENCING:
      if session_refused:
        self.silencing_failed = True
        self._transition(RadarSessionState.HANDBACK, "programming refused")
      elif not bus_healthy or not gate_passed or not standstill:
        # A request may already have been queued: undo it instead of abandoning it.
        self._transition(RadarSessionState.HANDBACK if self.programming_sent else RadarSessionState.STOCK,
                         "takeover prerequisites lost")
      elif self.programming_sent and not stock_radar_alive:
        self._transition(RadarSessionState.SILENCED, "requested radar silence")
      elif self.state_frames >= RADAR_SESSION_LIMIT_FRAMES:
        self.silencing_failed = True
        self._transition(RadarSessionState.HANDBACK, "programming timed out")
      elif self.frame % CarControllerParams.RADAR_UDS_STEP == 0:
        self.diagnostic_message = create_radar_session_msg(uds.SESSION_TYPE.PROGRAMMING)
        self.programming_sent = True

    if self.state == RadarSessionState.SILENCED and self.frame % CarControllerParams.RADAR_UDS_STEP == 0:
      self.diagnostic_message = make_tester_present_msg(RADAR_ADDR, RADAR_BUS, suppress_response=True)
    self._update_replacement(stock_radar_alive, bus_healthy, stock_radar_gone)
    return self.state

  def _update_replacement(self, stock_radar_alive: bool, bus_healthy: bool, stock_radar_gone: bool) -> None:
    # Synthetic radar frames while the stock radar is quiet and ours: held through a bus
    # blip in HANDBACK, since a gap for the camera is worse than a late stop.
    if stock_radar_alive or self.state == RadarSessionState.STOCK:
      self.replacement_active = False
    elif self.state == RadarSessionState.SILENCED or \
         (self.state == RadarSessionState.HANDBACK and bus_healthy and (self.programming_sent or stock_radar_gone)):
      self.replacement_active = True
