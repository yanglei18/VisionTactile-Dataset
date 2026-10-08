"""Recording-priority read-only quality queue; never owns a recorder or bag."""
import threading

from .metadata import bounded_document


class QualityScheduler:
    def __init__(self, store, runner, *, can_run, event):
        self.store, self.runner, self.can_run, self.event = store, runner, can_run, event
        self._stop = threading.Event()
        self._cancel = threading.Event()
        self._wake = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, name="capture-quality", daemon=True)
        self._thread.start()
        return self

    def interrupt(self):
        self._cancel.set()
        self._wake.set()

    def wake(self):
        self._wake.set()

    def _run(self):
        while not self._stop.is_set():
            if self.can_run():
                self._cancel.clear()
                # Recheck after clearing so a simultaneous START cannot lose
                # its cancellation: the committed active trial also blocks us.
                if self.can_run() and self.run_once():
                    continue
            self._wake.wait(.1)
            self._wake.clear()

    def run_once(self):
        if self._stop.is_set() or not self.can_run():
            return False
        job = self.store.claim_quality()
        if job is None:
            return False
        try:
            self.store.write_manifest(job["trial_id"])
            details = self.store.details(job["trial_id"])
            finish = details["metadata"]["finish"]
            if finish is None or finish["controller"]["recorder_alive"]:
                raise ValueError("No frozen confirmation that the owned recorder stopped")
            alignment = self.store.alignment_snapshot(job["trial_id"])
            observed = details["metadata"]["start"].get("alignment_config")
            advisories = []
            if observed is not None and alignment is None:
                advisories.append(dict(code="alignment_config_unavailable", detail=
                    "Configured alignment file was unavailable at START; checked actual controller topics only."))
            report = self.runner.run(job["output_path"], job["report_path"],
                expected_topics=list(finish["controller"]["topics"]),
                alignment_config=alignment["path"] if alignment else None,
                advisories=advisories,
                cancel=lambda: self._cancel.is_set() or self._stop.is_set() or not self.can_run())
            if self._cancel.is_set() or self._stop.is_set() or not self.can_run():
                # The child may finish while START arrives. Do not consume a
                # possibly incomplete receipt; a later attempt uses a new path.
                self.store.finish_quality(job, state="INTERRUPTED", detail="Recording or shutdown took priority; quality will resume when idle.")
            else:
                if not isinstance(report, dict) or report.get("status") not in {"ok", "warnings"}:
                    raise ValueError("Quality runner returned an invalid report")
                verdict = report["status"]
                explanation = "Post-record observations only; this is not a sensor-accuracy certification."
                if advisories:
                    explanation = advisories[0]["detail"] + " " + explanation
                preview = bounded_document(dict(report, complete_report_path=job["report_path"]))
                self.store.finish_quality(job, state="COMPLETED", verdict=verdict, detail=explanation, report=preview)
        except Exception as exc:
            interrupted = type(exc).__name__ == "QualityCancelled" or self._cancel.is_set() or self._stop.is_set() or not self.can_run()
            self.store.finish_quality(job, state="INTERRUPTED" if interrupted else "ERROR",
                                      detail=("Quality interrupted; queued for the next idle period. " if interrupted else "Quality execution failed; recording state is unchanged. ")+str(exc))
            self.event("info" if interrupted else "warning", f"Trial {job['trial_id']} quality: {exc}", key="quality-runner")
        finally:
            try:
                self.store.write_manifest(job["trial_id"])
            except (OSError, ValueError) as exc:
                self.event("error", f"Quality manifest unavailable: {exc}", key="quality-manifest")
        return True

    def close(self):
        self._stop.set()
        self.interrupt()
        if self._thread is not None:
            self._thread.join(5.)
            if self._thread.is_alive():
                raise RuntimeError("Read-only quality worker did not cancel within its shutdown bound")
