"""Offline alignment and indexed reading of one unified multisensor bag."""

from .dataset import AlignedDataset
from .errors import (
    DatasetClosedError,
    DatasetError,
    DatasetFormatError,
    IntegrityError,
    MissingMessageError,
    RejectedDatasetError,
    SourceBagMismatchError,
    UnsupportedEncodingError,
)
from .model import MessageRef, Transform
from .sdk_model import (
    AdditionalSample,
    AlignedFrame,
    CameraInfoData,
    CameraSample,
    ImageData,
    RegionOfInterestData,
    TrackerPose,
)

__all__ = [
    "AdditionalSample",
    "AlignedDataset",
    "AlignedFrame",
    "CameraInfoData",
    "CameraSample",
    "DatasetClosedError",
    "DatasetError",
    "DatasetFormatError",
    "ImageData",
    "IntegrityError",
    "MessageRef",
    "MissingMessageError",
    "RegionOfInterestData",
    "RejectedDatasetError",
    "SourceBagMismatchError",
    "TrackerPose",
    "Transform",
    "UnsupportedEncodingError",
]
__version__ = "0.3.0"
