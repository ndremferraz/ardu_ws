#!/usr/bin/env python3

import argparse

import cv2
import numpy as np


CAMERA_MATRIX = np.array([
    [311.2275116435304, 0.0, 302.7510972126],
    [0.0, 311.65181697045796, 225.589028786722],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

DIST_COEFFS = np.array([
    [-0.04422166943796803,
     0.01770872063406505,
     0.00012155500254070801,
     0.0005324040497094841,
     0.0]
], dtype=np.float64)


def build_detector(dictionary_name):
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

    if hasattr(cv2.aruco, 'DetectorParameters'):
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return dictionary, detector, parameters

    parameters = cv2.aruco.DetectorParameters_create()
    return dictionary, None, parameters


def detect_markers(frame, dictionary, detector, parameters):
    if detector is not None:
        return detector.detectMarkers(frame)
    return cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Basic OpenCV ArUco detector with hardcoded bottom camera calibration.'
    )
    parser.add_argument(
        '--camera',
        type=int,
        default=0,
        help='Camera index to open when --image is not provided.',
    )
    parser.add_argument(
        '--image',
        type=str,
        help='Optional path to a single test image.',
    )
    parser.add_argument(
        '--marker-length',
        type=float,
        default=0.3048,
        help='Marker side length in meters for pose estimation.',
    )
    parser.add_argument(
        '--dict',
        dest='dictionary_name',
        default='DICT_6X6_50',
        help='OpenCV ArUco dictionary name, for example DICT_6X6_50.',
    )
    return parser.parse_args()


def draw_pose(frame, corners, ids, marker_length):
    if ids is None or len(ids) == 0:
        return frame

    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        marker_length,
        CAMERA_MATRIX,
        DIST_COEFFS,
    )

    for marker_id, rvec, tvec, marker_corners in zip(ids.flatten(), rvecs, tvecs, corners):
        if hasattr(cv2, 'drawFrameAxes'):
            cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, marker_length * 0.5)

        center = marker_corners[0].mean(axis=0).astype(int)
        tx, ty, tz = tvec[0]
        label = f'ID {int(marker_id)} x={tx:.2f} y={ty:.2f} z={tz:.2f} m'
        cv2.putText(
            frame,
            label,
            (center[0] - 120, center[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        print(label)

    return frame


def run_on_image(image_path, dictionary, detector, parameters, marker_length):
    frame = cv2.imread(image_path)
    if frame is None:
        raise FileNotFoundError(f'Could not read image: {image_path}')

    corners, ids, _ = detect_markers(frame, dictionary, detector, parameters)
    annotated = draw_pose(frame, corners, ids, marker_length)
    cv2.imshow('ArUco Detector', annotated)
    cv2.waitKey(0)


def run_on_camera(camera_index, dictionary, detector, parameters, marker_length):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f'Could not open camera index {camera_index}')

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('Failed to read frame from camera.')

            corners, ids, _ = detect_markers(frame, dictionary, detector, parameters)
            annotated = draw_pose(frame, corners, ids, marker_length)
            cv2.imshow('ArUco Detector', annotated)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
    finally:
        cap.release()


def main():
    args = parse_args()
    dictionary, detector, parameters = build_detector(args.dictionary_name)

    print('Using camera matrix:')
    print(CAMERA_MATRIX)
    print('Using distortion coefficients:')
    print(DIST_COEFFS)

    if args.image:
        run_on_image(args.image, dictionary, detector, parameters, args.marker_length)
    else:
        run_on_camera(args.camera, dictionary, detector, parameters, args.marker_length)

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
