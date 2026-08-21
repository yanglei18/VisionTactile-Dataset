"""Typed failures raised by the aligned-dataset reader SDK."""


class DatasetError(Exception):
    """Base class for aligned-dataset reader failures."""


class DatasetFormatError(DatasetError):
    """An alignment document or ROS message violates its schema."""


class IntegrityError(DatasetError):
    """An alignment export does not match its integrity inventory."""


class RejectedDatasetError(DatasetError):
    """The alignment quality verdict is REJECTED."""


class SourceBagMismatchError(DatasetError):
    """The supplied source bag does not match the alignment manifest."""


class MissingMessageError(DatasetError):
    """A non-null message reference cannot be resolved in the source bag."""


class UnsupportedEncodingError(DatasetError):
    """A ROS image uses an encoding not supported by the SDK."""


class DatasetClosedError(DatasetError):
    """An operation requires an open dataset or message resolver."""
