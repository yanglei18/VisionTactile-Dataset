"""Private SQLite intent ledger. External recorder calls never run in a transaction."""
from __future__ import annotations

from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import stat
import threading
import uuid


ACTIVE_TRIAL_STATES = ("STARTING", "RECORDING", "STOPPING", "UNKNOWN")


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def bounded_text(value, maximum=2048):
    return str(value).encode("utf-8")[:maximum].decode("utf-8", errors="ignore")


def fingerprint(command, params):
    payload = json.dumps(dict(command=command, params=params), sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode()).hexdigest(), payload


def private_directory(path):
    path = Path(path).expanduser().absolute()
    missing, ancestor = [], path
    while not ancestor.exists() and not ancestor.is_symlink():
        missing.append(ancestor)
        ancestor = ancestor.parent
    for item in reversed(missing):
        item.mkdir(mode=0o700)
    info = path.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise ValueError(f"State directory must be owned, private (0700), and not a symlink: {path}")
    return path


class IntentStore:
    def __init__(self, directory):
        self.directory = private_directory(directory)
        self.manifests = private_directory(self.directory / "trials")
        self._mutex = threading.RLock()
        self._artifact_mutex = threading.RLock()
        self._lock_file = None
        self._db = None
        try:
            fd = os.open(self.directory / "manager.lock", os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
            self._lock_file = os.fdopen(fd, "r+")
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            database = self.directory / "capture.sqlite3"
            if database.is_symlink():
                raise ValueError("Refusing symlink database")
            self._db = sqlite3.connect(database, isolation_level=None, check_same_thread=False, timeout=2.)
            os.chmod(database, 0o600)
            self._db.row_factory = sqlite3.Row
            version = self._db.execute("PRAGMA user_version").fetchone()[0]
            if version not in (0, 1, 2):
                raise ValueError(f"Unsupported management database schema version {version}; refusing to downgrade")
            self._db.executescript("""
                PRAGMA journal_mode=WAL;
                PRAGMA synchronous=FULL;
                PRAGMA foreign_keys=ON;
                BEGIN IMMEDIATE;
                CREATE TABLE IF NOT EXISTS batches (
                    id TEXT PRIMARY KEY, label TEXT NOT NULL, created_at TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS trials (
                    id TEXT PRIMARY KEY, batch_id TEXT NOT NULL REFERENCES batches(id), label TEXT NOT NULL,
                    state TEXT NOT NULL, session_id TEXT NOT NULL DEFAULT '', output_path TEXT NOT NULL DEFAULT '',
                    duration_s REAL NOT NULL, elapsed_s REAL NOT NULL DEFAULT 0,
                    quality TEXT NOT NULL DEFAULT 'NOT_ASSESSED', detail TEXT NOT NULL DEFAULT '',
                    created_at TEXT NOT NULL, controller_instance_id TEXT NOT NULL, start_request_id TEXT UNIQUE NOT NULL);
                CREATE TABLE IF NOT EXISTS operations (
                    request_id TEXT PRIMARY KEY, fingerprint TEXT NOT NULL, payload TEXT NOT NULL,
                    command TEXT NOT NULL, state TEXT NOT NULL, code TEXT NOT NULL, detail TEXT NOT NULL,
                    trial_id TEXT NOT NULL DEFAULT '', phase TEXT NOT NULL, created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL);
                CREATE INDEX IF NOT EXISTS trials_created ON trials(created_at);
                CREATE INDEX IF NOT EXISTS operations_updated ON operations(updated_at);
                CREATE TABLE IF NOT EXISTS trial_metadata (
                    trial_id TEXT PRIMARY KEY REFERENCES trials(id), start_json TEXT, finish_json TEXT,
                    start_path TEXT NOT NULL DEFAULT '', finish_path TEXT NOT NULL DEFAULT '');
                CREATE TABLE IF NOT EXISTS alignment_snapshots (
                    trial_id TEXT PRIMARY KEY REFERENCES trials(id), content BLOB NOT NULL,
                    sha256 TEXT NOT NULL, path TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS quality_jobs (
                    trial_id TEXT PRIMARY KEY REFERENCES trials(id), state TEXT NOT NULL,
                    verdict TEXT NOT NULL DEFAULT '', detail TEXT NOT NULL DEFAULT '', attempt INTEGER NOT NULL DEFAULT 0,
                    report_path TEXT NOT NULL DEFAULT '', report_json TEXT, created_at TEXT NOT NULL, updated_at TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS quality_attempts (
                    trial_id TEXT NOT NULL REFERENCES trials(id), attempt INTEGER NOT NULL,
                    state TEXT NOT NULL, report_path TEXT NOT NULL, detail TEXT NOT NULL DEFAULT '',
                    started_at TEXT NOT NULL, finished_at TEXT, PRIMARY KEY(trial_id,attempt));
                CREATE TRIGGER IF NOT EXISTS immutable_start_metadata BEFORE UPDATE OF start_json ON trial_metadata
                    WHEN OLD.start_json IS NOT NULL AND NEW.start_json IS NOT OLD.start_json
                    BEGIN SELECT RAISE(ABORT, 'start metadata is immutable'); END;
                CREATE TRIGGER IF NOT EXISTS immutable_finish_metadata BEFORE UPDATE OF finish_json ON trial_metadata
                    WHEN OLD.finish_json IS NOT NULL AND NEW.finish_json IS NOT OLD.finish_json
                    BEGIN SELECT RAISE(ABORT, 'finish metadata is immutable'); END;
                PRAGMA user_version=2;
                COMMIT;
            """)
        except BaseException:
            self.close()
            raise

    @contextmanager
    def transaction(self):
        with self._mutex:
            self._db.execute("BEGIN IMMEDIATE")
            try:
                yield self._db
                self._db.execute("COMMIT")
            except BaseException:
                self._db.execute("ROLLBACK")
                raise

    @staticmethod
    def receipt(row):
        return {key: row[key] for key in ("request_id", "state", "code", "detail", "trial_id")}

    def operation(self, request_id):
        with self._mutex:
            row = self._db.execute("SELECT * FROM operations WHERE request_id=?", (request_id,)).fetchone()
            return dict(row) if row is not None else None

    @staticmethod
    def insert_operation(db, request_id, command, params, *, state, code, detail, trial_id="", phase="TERMINAL"):
        digest, payload = fingerprint(command, params)
        now = utc_now()
        db.execute("INSERT INTO operations VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                   (request_id, digest, payload, command, state, code, bounded_text(detail), trial_id, phase, now, now))

    def update_operation(self, request_id, *, state, code, detail, phase="TERMINAL"):
        with self.transaction() as db:
            db.execute("UPDATE operations SET state=?,code=?,detail=?,phase=?,updated_at=? WHERE request_id=?",
                       (state, code, bounded_text(detail), phase, utc_now(), request_id))

    def trial(self, trial_id):
        with self._mutex:
            row = self._db.execute("SELECT * FROM trials WHERE id=?", (trial_id,)).fetchone()
            return dict(row) if row else None

    def active_trials(self):
        with self._mutex:
            return [dict(row) for row in self._db.execute(
                "SELECT * FROM trials WHERE state IN ('STARTING','RECORDING','STOPPING','UNKNOWN')")]

    def recover_unresolved(self):
        """On restart: never automatically dispatch a persisted START or STOP."""
        with self.transaction() as db:
            db.execute("UPDATE operations SET state='unknown',code='manager_restarted',"
                       "detail='Manager restarted; reconcile snapshot only, no command automatically retried.',"
                       "phase='UNKNOWN',updated_at=? WHERE phase IN ('INTENT','DISPATCHED')", (utc_now(),))
            db.execute("UPDATE trials SET state='UNKNOWN',detail='Awaiting controller reconciliation after manager restart.' "
                       "WHERE state IN ('STARTING','RECORDING','STOPPING')")
            db.execute("UPDATE quality_jobs SET state='QUEUED',verdict='',detail='Interrupted by manager restart; read-only quality will retry when idle.',updated_at=? "
                       "WHERE state IN ('RUNNING','INTERRUPTED')", (utc_now(),))
            db.execute("UPDATE quality_attempts SET state='INTERRUPTED',detail='Manager restarted before quality receipt.',finished_at=? "
                       "WHERE state='RUNNING'", (utc_now(),))
            db.execute("UPDATE trials SET quality='QUEUED' WHERE id IN (SELECT trial_id FROM quality_jobs WHERE state='QUEUED')")

    def insert_start_metadata(self, db, trial_id, value, alignment_content=None):
        start_path = str(self.manifests/(trial_id+".start.json"))
        finish_path = str(self.manifests/(trial_id+".finish.json"))
        db.execute("INSERT INTO trial_metadata VALUES(?,?,NULL,?,?)",
                   (trial_id, json.dumps(value, ensure_ascii=False, allow_nan=False), start_path, finish_path))
        if alignment_content is not None:
            digest = hashlib.sha256(alignment_content).hexdigest()
            path = str(self.manifests/(trial_id+".alignment.yaml"))
            db.execute("INSERT INTO alignment_snapshots VALUES(?,?,?,?)", (trial_id, alignment_content, digest, path))

    def freeze_finish(self, db, trial_id, value, *, quality_enabled):
        updated = db.execute("UPDATE trial_metadata SET finish_json=? WHERE trial_id=? AND start_json IS NOT NULL AND finish_json IS NULL",
                             (json.dumps(value, ensure_ascii=False, allow_nan=False), trial_id)).rowcount
        if not updated:
            return False
        now = utc_now()
        state = "QUEUED" if quality_enabled else "NOT_ASSESSED"
        detail = "Owned terminal recorder confirmed stopped; quality queued." if quality_enabled else "Quality is disabled in manager configuration."
        db.execute("INSERT OR IGNORE INTO quality_jobs(trial_id,state,detail,created_at,updated_at) VALUES(?,?,?,?,?)",
                   (trial_id, state, detail, now, now))
        db.execute("UPDATE trials SET quality=? WHERE id=?", (state, trial_id))
        return True

    def quality_job(self, trial_id):
        with self._mutex:
            row = self._db.execute("SELECT * FROM quality_jobs WHERE trial_id=?", (trial_id,)).fetchone()
            return dict(row) if row else None

    def details(self, trial_id):
        with self._mutex:
            if self._db.execute("SELECT 1 FROM trials WHERE id=?", (trial_id,)).fetchone() is None:
                return None
            metadata = self._db.execute("SELECT * FROM trial_metadata WHERE trial_id=?", (trial_id,)).fetchone()
            job = self.quality_job(trial_id)
            return dict(trial_id=trial_id,
                metadata=dict(start=json.loads(metadata["start_json"]) if metadata and metadata["start_json"] else None,
                              finish=json.loads(metadata["finish_json"]) if metadata and metadata["finish_json"] else None),
                quality=(dict(state=job["state"], verdict=job["verdict"], detail=job["detail"], attempt=job["attempt"],
                              report_path=job["report_path"], report=json.loads(job["report_json"]) if job["report_json"] else None)
                         if job else dict(state="NOT_ASSESSED", verdict="", detail="No verified finish metadata or quality job.",
                                          attempt=0, report_path="", report=None)),
                artifacts=dict(start_metadata_path=metadata["start_path"] if metadata else "",
                               finish_metadata_path=metadata["finish_path"] if metadata and metadata["finish_json"] else ""))

    def retry_quality(self, db, trial_id):
        now = utc_now()
        db.execute("UPDATE quality_jobs SET state='QUEUED',verdict='',detail='Explicit quality retry queued.',report_json=NULL,updated_at=? WHERE trial_id=?",
                   (now, trial_id))
        db.execute("UPDATE trials SET quality='QUEUED' WHERE id=?", (trial_id,))

    def claim_quality(self):
        with self.transaction() as db:
            row = db.execute("SELECT q.*,t.output_path FROM quality_jobs q JOIN trials t ON t.id=q.trial_id "
                             "WHERE q.state IN ('QUEUED','INTERRUPTED') ORDER BY q.created_at LIMIT 1").fetchone()
            if row is None:
                return None
            job = dict(row)
            attempt = job["attempt"]+1
            path = str(self.manifests/(job["trial_id"]+f".quality.{attempt}.json"))
            now = utc_now()
            db.execute("UPDATE quality_jobs SET state='RUNNING',verdict='',attempt=?,report_path=?,report_json=NULL,detail='Read-only quality subprocess running.',updated_at=? WHERE trial_id=?",
                       (attempt, path, now, job["trial_id"]))
            db.execute("INSERT INTO quality_attempts(trial_id,attempt,state,report_path,started_at) VALUES(?,?,'RUNNING',?,?)",
                       (job["trial_id"], attempt, path, now))
            db.execute("UPDATE trials SET quality='RUNNING' WHERE id=?", (job["trial_id"],))
            job.update(attempt=attempt, report_path=path, state="RUNNING")
            return job

    def finish_quality(self, job, *, state, verdict="", detail="", report=None):
        quality = {"COMPLETED": "WARNINGS" if verdict == "warnings" else "OK"}.get(state, state)
        with self.transaction() as db:
            updated = db.execute("UPDATE quality_jobs SET state=?,verdict=?,detail=?,report_json=?,updated_at=? "
                                 "WHERE trial_id=? AND attempt=? AND state='RUNNING'",
                                 (state, verdict, bounded_text(detail, 4096), json.dumps(report, ensure_ascii=False, allow_nan=False) if report is not None else None,
                                  utc_now(), job["trial_id"], job["attempt"])).rowcount
            if not updated:
                return
            db.execute("UPDATE quality_attempts SET state=?,detail=?,finished_at=? WHERE trial_id=? AND attempt=?",
                       (state, bounded_text(detail, 4096), utc_now(), job["trial_id"], job["attempt"]))
            db.execute("UPDATE trials SET quality=? WHERE id=?", (quality, job["trial_id"]))

    def alignment_snapshot(self, trial_id):
        with self._mutex:
            row = self._db.execute("SELECT * FROM alignment_snapshots WHERE trial_id=?", (trial_id,)).fetchone()
            return dict(row) if row else None

    def view(self):
        with self._mutex:
            batches = [dict(row) for row in self._db.execute("SELECT * FROM batches ORDER BY created_at DESC,rowid DESC LIMIT 200")]
            trials = [self.public_trial(dict(row)) for row in self._db.execute(
                "SELECT * FROM trials ORDER BY created_at DESC,rowid DESC LIMIT 200")]
            row = self._db.execute("SELECT * FROM operations ORDER BY updated_at DESC,rowid DESC LIMIT 1").fetchone()
            return dict(batches=batches, trials=trials, operation=self.receipt(row) if row else None)

    @staticmethod
    def public_trial(row):
        return {key: row[key] for key in ("id", "batch_id", "label", "state", "session_id", "output_path",
                                        "duration_s", "elapsed_s", "quality", "detail", "created_at")}

    def write_manifest(self, trial_id):
        # Artifact I/O never holds the database lock. The separate artifact
        # lock prevents an older manifest write overtaking a newer quality one.
        with self._artifact_mutex:
            self._write_manifest_locked(trial_id)

    def _write_manifest_locked(self, trial_id):
        row = self.trial(trial_id)
        if row is None:
            return
        # Trial IDs are internal UUIDs, not user-controlled paths. Never write
        # into output_path: it is evidence pointing to the controller's bag.
        if len(trial_id) != 32 or any(c not in "0123456789abcdef" for c in trial_id):
            raise ValueError("invalid internal trial id")
        target = self.manifests / (trial_id + ".json")
        temporary = self.manifests / (trial_id + "." + uuid.uuid4().hex + ".tmp")
        details = self.details(trial_id)
        for key in ("start", "finish"):
            value = details["metadata"][key]
            if value is not None:
                content = (json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False)+"\n").encode()
                self._write_immutable(Path(details["artifacts"][key+"_metadata_path"]), content)
        alignment = self.alignment_snapshot(trial_id)
        if alignment is not None:
            self._write_immutable(Path(alignment["path"]), alignment["content"])
        value = dict(schema_version=2, **row, metadata=details["metadata"], quality_details=details["quality"])
        try:
            fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(value, handle, ensure_ascii=False, sort_keys=True, allow_nan=False)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            if target.is_symlink():
                raise ValueError("Refusing symlink manifest")
            os.replace(temporary, target)
            directory_fd = os.open(self.manifests, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if temporary.exists():
                temporary.unlink()

    @staticmethod
    def _write_immutable(target, content):
        if target.exists() or target.is_symlink():
            if target.is_symlink() or not target.is_file() or target.stat().st_size != len(content):
                raise ValueError(f"Frozen artifact differs from the database: {target.name}")
            with target.open("rb") as stream:
                if stream.read(len(content)+1) != content:
                    raise ValueError(f"Frozen artifact differs from the database: {target.name}")
            return
        temporary = target.with_name(target.name+"."+uuid.uuid4().hex+".tmp")
        try:
            fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
            with os.fdopen(fd, "wb") as handle:
                handle.write(content)
                handle.flush()
                os.fsync(handle.fileno())
            os.link(temporary, target)  # Atomic creation, never overwrite frozen evidence.
        finally:
            if temporary.exists():
                temporary.unlink()

    def close(self):
        with self._mutex:
            if self._db is not None:
                self._db.close()
                self._db = None
            if self._lock_file is not None:
                self._lock_file.close()
                self._lock_file = None
