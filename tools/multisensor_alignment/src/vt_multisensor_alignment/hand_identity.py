"""Bounded provenance for the hand messages actually decoded from a recording."""


class HandStreamIdentity:
    def __init__(self, *, pose):
        self.pose = pose
        self.first = self.last = None
        self.changes = self.unknown = 0

    def observe(self, message):
        keys = ("hand", "source_mode", "session_id", "config_sha256")
        identity = {key: getattr(message, key, None) for key in keys}
        if not all(isinstance(value, str) and 0 < len(value) <= 256 for value in identity.values()):
            self.unknown += 1
            return
        if self.pose:
            identity.update(calibration_version=getattr(message, "calibration_version", None),
                            model_rest_sha256=getattr(message, "model_rest_sha256", None))
            if (type(identity["calibration_version"]) is not int or
                    not isinstance(identity["model_rest_sha256"], str) or not 0 < len(identity["model_rest_sha256"]) <= 128):
                self.unknown += 1
                return
        if self.first is None:
            self.first = identity
        elif self.last != identity:
            self.changes += 1
        self.last = identity

    def document(self):
        return dict(first=self.first, last=self.last, identity_changes=self.changes, unknown_messages=self.unknown)
