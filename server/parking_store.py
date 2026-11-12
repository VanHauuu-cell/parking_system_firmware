"""Durable approval history shared by the ENTRY and EXIT applications."""

from contextlib import closing
from pathlib import Path
import re
import sqlite3
import time


MIN_TIMESTAMP_MS = 1577836800000
MAX_TIMESTAMP_MS = 4102444800000
EVENT_FIELDS = (
    "event_id", "approval_id", "camera_id", "request_id", "kind", "uid", "plate",
    "approved_at_ms",
)
LINK_FIELDS = ("camera_id", "request_id", "uid", "plate", "approved_at_ms")


def _validate_event(event, expected_camera_id):
    if type(expected_camera_id) is not int or expected_camera_id not in (1, 2):
        raise ValueError("Expected camera ID must be 1 or 2")
    if not isinstance(event, dict):
        raise ValueError("Access event must be a JSON object")
    if any(name not in event for name in EVENT_FIELDS):
        raise ValueError("Access event is missing fields")
    data = {name: event[name] for name in EVENT_FIELDS}
    if type(data["camera_id"]) is not int or data["camera_id"] != expected_camera_id:
        raise ValueError("Access event came from the wrong camera")
    if (type(data["request_id"]) is not int or
            not 1 <= data["request_id"] <= 0xFFFFFFFF):
        raise ValueError("Invalid access request ID")
    if (type(data["approved_at_ms"]) is not int or
            not MIN_TIMESTAMP_MS <= data["approved_at_ms"] <= MAX_TIMESTAMP_MS):
        raise ValueError("Invalid approval timestamp")
    for name in ("event_id", "approval_id"):
        if (not isinstance(data[name], str) or
                re.fullmatch(r"[0-9A-F]{16}", data[name]) is None):
            raise ValueError(f"Invalid {name}")
    if (not isinstance(data["uid"], str) or
            re.fullmatch(r"[0-9A-F]{8}", data["uid"]) is None):
        raise ValueError("Invalid UID")
    if (not isinstance(data["plate"], str) or
            re.fullmatch(r"[A-Z0-9]{1,19}", data["plate"]) is None):
        raise ValueError("Invalid plate")
    if data["kind"] not in ("APPROVED", "CANCELLED"):
        raise ValueError("Invalid access event kind")
    if ((data["kind"] == "APPROVED") !=
            (data["event_id"] == data["approval_id"])):
        raise ValueError("Invalid approval reference")
    return data


def _result(event):
    return {
        **{name: event[name] for name in EVENT_FIELDS},
        "status": "CANCELLED",
        "entry_at_ms": None,
        "exit_at_ms": None,
        "duration_seconds": None,
        "duration_hours": None,
    }


def _project(events):
    """Rebuild one UID/plate history so late arrivals and cancellations reconcile."""
    cancelled = {event["approval_id"] for event in events if event["kind"] == "CANCELLED"}
    results = {event["event_id"]: _result(event) for event in events}
    visits = []
    active = None
    extra_entries = []
    for event in events:
        result = results[event["event_id"]]
        if event["kind"] == "CANCELLED" or event["event_id"] in cancelled:
            if event["camera_id"] == 1:
                result["entry_at_ms"] = event["approved_at_ms"]
            else:
                result["exit_at_ms"] = event["approved_at_ms"]
            continue
        if event["camera_id"] == 1:
            if active is None:
                active = {
                    "entry_event_id": event["event_id"],
                    "exit_event_id": None,
                    "uid": event["uid"],
                    "plate": event["plate"],
                    "entry_at_ms": event["approved_at_ms"],
                    "exit_at_ms": None,
                    "duration_seconds": None,
                }
                visits.append(active)
                result["status"] = "ENTRY_APPROVED"
                result["entry_at_ms"] = active["entry_at_ms"]
                extra_entries = []
            else:
                # Keep the first unclosed admission, while retaining the extra audit event.
                result["status"] = "EXTRA_ENTRY"
                result["entry_at_ms"] = active["entry_at_ms"]
                extra_entries.append(event["event_id"])
            continue
        result["exit_at_ms"] = event["approved_at_ms"]
        if active is None:
            result["status"] = "MISSING_ENTRY"
            continue
        active["exit_event_id"] = event["event_id"]
        active["exit_at_ms"] = event["approved_at_ms"]
        duration = (active["exit_at_ms"] - active["entry_at_ms"]) / 1000.0
        active["duration_seconds"] = duration
        result["status"] = "EXIT_APPROVED"
        linked_results = [result, results[active["entry_event_id"]]]
        linked_results.extend(results[event_id] for event_id in extra_entries)
        for linked in linked_results:
            linked.update(
                entry_at_ms=active["entry_at_ms"],
                exit_at_ms=active["exit_at_ms"],
                duration_seconds=duration,
                duration_hours=duration / 3600.0,
            )
        active = None
        extra_entries = []

    # A cancelled EXIT reopens its prior visit. Show that visit on the cancel result.
    for event in events:
        if event["kind"] != "CANCELLED" or event["camera_id"] != 2:
            continue
        previous = [visit for visit in visits if visit["entry_at_ms"] <= event["approved_at_ms"]]
        if previous:
            visit = previous[-1]
            results[event["event_id"]].update(
                entry_at_ms=visit["entry_at_ms"],
                exit_at_ms=visit["exit_at_ms"],
                duration_seconds=visit["duration_seconds"],
                duration_hours=(None if visit["duration_seconds"] is None
                                else visit["duration_seconds"] / 3600.0),
            )
    return results, visits


class ParkingStore:
    def __init__(self, db_path):
        self.path = Path(db_path).resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with closing(self._connect()) as connection:
            # A second process may create the database at the same time.
            for attempt in range(3):
                try:
                    connection.execute("PRAGMA journal_mode=WAL")
                    break
                except sqlite3.OperationalError as error:
                    if "locked" not in str(error).lower() or attempt == 2:
                        raise
                    time.sleep(0.1 * (attempt + 1))
            with connection:
                connection.execute("BEGIN IMMEDIATE")
                connection.execute("""
                    CREATE TABLE IF NOT EXISTS access_events (
                        event_id TEXT PRIMARY KEY,
                        approval_id TEXT NOT NULL,
                        camera_id INTEGER NOT NULL CHECK(camera_id IN (1, 2)),
                        request_id INTEGER NOT NULL,
                        kind TEXT NOT NULL CHECK(kind IN ('APPROVED', 'CANCELLED')),
                        uid TEXT NOT NULL,
                        plate TEXT NOT NULL,
                        approved_at_ms INTEGER NOT NULL,
                        received_at_ms INTEGER NOT NULL
                    )
                """)
                connection.execute("""
                    CREATE INDEX IF NOT EXISTS access_events_vehicle
                    ON access_events(uid, plate, approved_at_ms, camera_id, event_id)
                """)
                connection.execute("""
                    CREATE INDEX IF NOT EXISTS access_events_approval
                    ON access_events(approval_id)
                """)
                connection.execute("""
                    CREATE TABLE IF NOT EXISTS parking_visits (
                        entry_event_id TEXT PRIMARY KEY REFERENCES access_events(event_id),
                        exit_event_id TEXT UNIQUE REFERENCES access_events(event_id),
                        uid TEXT NOT NULL,
                        plate TEXT NOT NULL,
                        entry_at_ms INTEGER NOT NULL,
                        exit_at_ms INTEGER,
                        duration_seconds REAL
                    )
                """)

    def _connect(self):
        connection = sqlite3.connect(self.path, timeout=10, isolation_level=None)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA busy_timeout=10000")
        connection.execute("PRAGMA foreign_keys=ON")
        connection.execute("PRAGMA synchronous=FULL")
        return connection

    @staticmethod
    def _validate_reference(connection, event):
        if event["kind"] == "CANCELLED":
            parent = connection.execute(
                "SELECT * FROM access_events WHERE event_id=?", (event["approval_id"],)
            ).fetchone()
            if parent is not None and (parent["kind"] != "APPROVED" or
                    any(parent[name] != event[name] for name in LINK_FIELDS)):
                raise ValueError("Cancellation does not match its approval")
        children = connection.execute(
            "SELECT * FROM access_events WHERE approval_id=? AND kind='CANCELLED'",
            (event["approval_id"],),
        ).fetchall()
        if any(any(child[name] != event[name] for name in LINK_FIELDS) for child in children):
            raise ValueError("Conflicting approval reference")

    @staticmethod
    def _vehicle_events(connection, uid, plate):
        return connection.execute(
            """SELECT * FROM access_events WHERE uid=? AND plate=?
               ORDER BY approved_at_ms, camera_id, event_id""", (uid, plate)
        ).fetchall()

    def record_event(self, event, expected_camera_id):
        event = _validate_event(event, expected_camera_id)
        with closing(self._connect()) as connection:
            with connection:
                connection.execute("BEGIN IMMEDIATE")
                existing = connection.execute(
                    "SELECT * FROM access_events WHERE event_id=?", (event["event_id"],)
                ).fetchone()
                if existing is not None:
                    if any(existing[name] != event[name] for name in EVENT_FIELDS):
                        raise ValueError("Event ID was reused with a different payload")
                else:
                    self._validate_reference(connection, event)
                    connection.execute(
                        """INSERT INTO access_events (
                               event_id, approval_id, camera_id, request_id, kind, uid,
                               plate, approved_at_ms, received_at_ms
                           ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                        tuple(event[name] for name in EVENT_FIELDS) + (int(time.time() * 1000),),
                    )
                events = self._vehicle_events(connection, event["uid"], event["plate"])
                results, visits = _project(events)
                connection.execute(
                    "DELETE FROM parking_visits WHERE uid=? AND plate=?",
                    (event["uid"], event["plate"]),
                )
                connection.executemany(
                    """INSERT INTO parking_visits (
                           entry_event_id, exit_event_id, uid, plate, entry_at_ms,
                           exit_at_ms, duration_seconds
                       ) VALUES (:entry_event_id, :exit_event_id, :uid, :plate,
                                 :entry_at_ms, :exit_at_ms, :duration_seconds)""", visits
                )
            # SQLite commit has completed before the caller can acknowledge the CAM.
            return results[event["event_id"]]

    def latest_for_camera(self, camera_id):
        if type(camera_id) is not int or camera_id not in (1, 2):
            raise ValueError("Camera ID must be 1 or 2")
        with closing(self._connect()) as connection:
            with connection:
                connection.execute("BEGIN")
                latest = connection.execute(
                    "SELECT * FROM access_events WHERE camera_id=? ORDER BY rowid DESC LIMIT 1",
                    (camera_id,),
                ).fetchone()
                if latest is None:
                    return None
                events = self._vehicle_events(connection, latest["uid"], latest["plate"])
                results, _ = _project(events)
                return results[latest["event_id"]]

    def close(self):
        """Connections close after each operation, including failed operations."""
