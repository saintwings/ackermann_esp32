"""
Per-robot file logger.

Creates two daily-rotating files under logs/<robot_id>/:
  telemetry_YYYYMMDD.jsonl  — one JSON line per telemetry sample (append-safe)
  comm_YYYYMMDD.log         — human-readable connect / command / ack events

Usage:
    logger = RobotLogger("ladybug_001")
    logger.log_telemetry(telemetry_message_dict)
    logger.log_comm("CONNECT", "name=ladybug type=ackermann")
"""

import json
import os
from datetime import datetime, date, timezone

LOGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")


class RobotLogger:
    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self._dir = os.path.join(LOGS_DIR, robot_id)
        os.makedirs(self._dir, exist_ok=True)
        self._current_date: date | None = None
        self._telem_fh = None
        self._comm_fh = None
        self._rotate()

    # ── public API ────────────────────────────────────────────────────────────

    def log_telemetry(self, message: dict) -> None:
        """Append one telemetry sample as a JSON line."""
        self._rotate()
        record = {"ts": self._ts(), "payload": message.get("payload", {})}
        self._telem_fh.write(json.dumps(record, separators=(",", ":")) + "\n")

    def log_comm(self, event: str, detail: str = "") -> None:
        """Append one communication event line."""
        self._rotate()
        line = f"[{self._ts()}] {event}"
        if detail:
            line += f"  {detail}"
        self._comm_fh.write(line + "\n")

    def close(self) -> None:
        for fh in (self._telem_fh, self._comm_fh):
            if fh:
                fh.close()
        self._telem_fh = self._comm_fh = None

    # ── internal ──────────────────────────────────────────────────────────────

    @staticmethod
    def _ts() -> str:
        return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"

    def _rotate(self) -> None:
        today = date.today()
        if today == self._current_date:
            return
        self._current_date = today
        date_str = today.strftime("%Y%m%d")

        if self._telem_fh:
            self._telem_fh.close()
        if self._comm_fh:
            self._comm_fh.close()

        # line-buffered so data lands on disk without explicit flush
        self._telem_fh = open(
            os.path.join(self._dir, f"telemetry_{date_str}.jsonl"), "a", buffering=1
        )
        self._comm_fh = open(
            os.path.join(self._dir, f"comm_{date_str}.log"), "a", buffering=1
        )
