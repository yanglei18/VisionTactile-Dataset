from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from .config import BoardConfig
from .model import BoardObservation, CameraIntrinsics
from .transforms import Transform


def _dictionary(config: BoardConfig) -> object:
    dictionary_id = getattr(cv2.aruco, config.dictionary, None)
    if dictionary_id is None or not isinstance(dictionary_id, int):
        raise ValueError(f"unsupported ArUco dictionary: {config.dictionary}")
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def create_board(config: BoardConfig) -> object:
    dictionary = _dictionary(config)
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            config.squares_x,
            config.squares_y,
            config.square_length_m,
            config.marker_length_m,
            dictionary,
        )
    return cv2.aruco.CharucoBoard(
        (config.squares_x, config.squares_y),
        config.square_length_m,
        config.marker_length_m,
        dictionary,
    )


def board_corner_points(board: object) -> np.ndarray:
    if hasattr(board, "getChessboardCorners"):
        points = board.getChessboardCorners()
    else:
        points = board.chessboardCorners
    return np.asarray(points, dtype=np.float64).reshape(-1, 3)


def render_board(config: BoardConfig, output: str | Path, dpi: int = 300) -> Path:
    if type(dpi) is not int or not 72 <= dpi <= 1200:
        raise ValueError("dpi must be an integer within [72, 1200]")
    target = Path(output)
    if target.suffix.lower() != ".png":
        raise ValueError("board output must be a PNG file")
    if not target.parent.is_dir():
        raise FileNotFoundError(f"output directory does not exist: {target.parent}")
    if target.exists():
        raise FileExistsError(f"refusing to overwrite existing board: {target}")
    pixels_per_metre = dpi / 0.0254
    width = round(config.squares_x * config.square_length_m * pixels_per_metre)
    height = round(config.squares_y * config.square_length_m * pixels_per_metre)
    board = create_board(config)
    if hasattr(board, "generateImage"):
        image = board.generateImage((width, height), marginSize=0, borderBits=1)
    else:
        image = board.draw((width, height), marginSize=0, borderBits=1)
    Image.fromarray(image).save(target, format="PNG", dpi=(dpi, dpi))
    return target


def _grayscale(image: np.ndarray, encoding: str) -> np.ndarray:
    value = np.asarray(image)
    if value.dtype != np.uint8:
        raise ValueError("ChArUco detection requires an 8-bit image")
    if value.ndim == 2:
        return value
    if value.ndim != 3 or value.shape[2] not in {3, 4}:
        raise ValueError("image must be mono, RGB, BGR, RGBA, or BGRA")
    conversions = {
        "rgb8": cv2.COLOR_RGB2GRAY,
        "bgr8": cv2.COLOR_BGR2GRAY,
        "rgba8": cv2.COLOR_RGBA2GRAY,
        "bgra8": cv2.COLOR_BGRA2GRAY,
    }
    conversion = conversions.get(encoding.lower())
    if conversion is None:
        raise ValueError(f"unsupported image encoding: {encoding}")
    return cv2.cvtColor(value, conversion)


def detect_board_pose(
    image: np.ndarray,
    *,
    encoding: str,
    intrinsics: CameraIntrinsics,
    config: BoardConfig,
    timestamp_ns: int,
    source_stamp_ns: int,
) -> BoardObservation | None:
    gray = _grayscale(image, encoding)
    if (gray.shape[1], gray.shape[0]) != (intrinsics.width, intrinsics.height):
        raise ValueError(
            "image dimensions do not match CameraInfo: "
            f"image={gray.shape[1]}x{gray.shape[0]} "
            f"camera_info={intrinsics.width}x{intrinsics.height}"
        )
    dictionary = _dictionary(config)
    board = create_board(config)
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        parameters = cv2.aruco.DetectorParameters_create()
    else:
        parameters = cv2.aruco.DetectorParameters()
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
        gray, dictionary, parameters=parameters
    )
    if marker_ids is None or len(marker_ids) < 2:
        return None
    count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        gray,
        board,
        cameraMatrix=intrinsics.camera_matrix,
        distCoeffs=intrinsics.distortion,
    )
    if (
        charuco_ids is None
        or charuco_corners is None
        or int(count) < config.min_corners
    ):
        return None
    success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        charuco_corners,
        charuco_ids,
        board,
        intrinsics.camera_matrix,
        intrinsics.distortion,
        None,
        None,
    )
    if not success:
        return None
    object_points = board_corner_points(board)[
        np.asarray(charuco_ids, dtype=np.int64).reshape(-1)
    ]
    projected, _ = cv2.projectPoints(
        object_points,
        rvec,
        tvec,
        intrinsics.camera_matrix,
        intrinsics.distortion,
    )
    measured = np.asarray(charuco_corners, dtype=np.float64).reshape(-1, 2)
    difference = projected.reshape(-1, 2) - measured
    rms = float(np.sqrt(np.mean(np.sum(difference * difference, axis=1))))
    if not np.isfinite(rms) or rms > config.max_reprojection_rms_px:
        return None
    return BoardObservation(
        timestamp_ns=timestamp_ns,
        camera_from_board=Transform.from_rvec_tvec(rvec, tvec),
        reprojection_rms_px=rms,
        corner_count=int(count),
        source_stamp_ns=source_stamp_ns,
    )
