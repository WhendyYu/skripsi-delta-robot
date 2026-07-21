#!/usr/bin/env python3

import cv2
import json
import yaml
import math
import numpy as np

import rclpy

from rclpy.node import Node

from pupil_apriltags import Detector

from geometry_msgs.msg import Point

from msgs.msg import DetectedObject
from msgs.srv import CaptureWorkspace

from ament_index_python.packages import (
    get_package_share_directory
)

import os

class DetectorNode(Node):

    def __init__(self):

        super().__init__(
            "detector_node"
        )

        # ====================================================
        # PARAMETERS
        # ====================================================

        self.declare_parameter(
            "calibration_path",
            os.path.join(
                get_package_share_directory('odete'),
                'config',
                'camera_calibration.json'
            )
        )

        self.declare_parameter(
            "workspace_path",
            os.path.join(
                get_package_share_directory('odete'),
                'config',
                'workspace.yaml'
            )
        )

        self.declare_parameter(
            "camera_index",
            0
        )

        calibration_path = self.get_parameter(
            "calibration_path"
        ).value

        workspace_path = self.get_parameter(
            "workspace_path"
        ).value

        camera_index = self.get_parameter(
            "camera_index"
        ).value

        # ====================================================
        # LOAD CONFIG
        # ====================================================

        with open(workspace_path, "r") as f:

            workspace = yaml.safe_load(f)

        self.workspace = workspace

        # ====================================================
        # APRILTAG CONFIG
        # ====================================================

        self.TAG_CENTERS = {

            int(k): tuple(v)

            for k, v in workspace[
                "apriltags"
            ].items()
        }

        self.ARENA_SIZE_MM = workspace[
            "arena_size_mm"
        ]

        self.TAG_SIZE_MM = workspace[
            "apriltag"
        ]["tag_size_mm"]

        self.MIN_MARGIN = workspace[
            "apriltag"
        ]["min_margin"]

        # ====================================================
        # BIRD VIEW CONFIG
        # ====================================================

        bird = workspace[
            "bird_view"
        ]

        self.BIRD_SCALE = bird[
            "scale"
        ]

        self.BIRD_PADDING = bird[
            "padding"
        ]

        self.CROP_SIZE = bird[
            "crop_size"
        ]

        self.H_ALPHA = bird[
            "homography_alpha"
        ]

        # ====================================================
        # DETECTION CONFIG
        # ====================================================

        detection = workspace[
            "detection"
        ]

        self.MIN_AREA = detection[
            "min_area"
        ]

        self.MIN_CIRCULARITY = detection[
            "min_circularity"
        ]

        self.color_configs = detection[
            "colors"
        ]

        self.kernel = np.ones(
            (5, 5),
            np.uint8
        )

        # ====================================================
        # QUADRATIC CALIBRATION
        # ====================================================

        self.qx = workspace[
            "quadratic_calibration"
        ]["x"]

        self.qy = workspace[
            "quadratic_calibration"
        ]["y"]

        # ====================================================
        # ROBOT SYNC
        # ====================================================

        sync = workspace[
            "robot_sync"
        ]

        self.tx = sync["tx_mm"]

        self.ty = sync["ty_mm"]

        self.rotation_deg = sync[
            "rotation_deg"
        ]

        self.rotation_rad = math.radians(
            self.rotation_deg
        )

        # ====================================================
        # CAMERA CONFIG
        # ====================================================

        camera = workspace[
            "camera"
        ]

        self.WIDTH = camera[
            "width"
        ]

        self.HEIGHT = camera[
            "height"
        ]

        self.FPS = camera[
            "fps"
        ]

        # ====================================================
        # LOAD CAMERA CALIBRATION
        # ====================================================

        with open(calibration_path, "r") as f:

            calib = json.load(f)

        self.K = np.array(
            calib["camera_matrix"],
            dtype=np.float32
        )

        self.dist = np.array(
            calib["distortion_coefficients"],
            dtype=np.float32
        )

        self.get_logger().info(
            "Calibration loaded"
        )

        # ====================================================
        # APRILTAG DETECTOR
        # ====================================================

        self.detector = Detector(

            families="tag36h11",

            nthreads=4,

            quad_decimate=1.0,

            quad_sigma=0.8,

            refine_edges=1,

            decode_sharpening=0.35,
        )

        # ====================================================
        # CAMERA
        # ====================================================

        self.cap = cv2.VideoCapture(
            camera_index,
            cv2.CAP_V4L2
        )

        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*'MJPG')
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self.WIDTH
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self.HEIGHT
        )

        self.cap.set(
            cv2.CAP_PROP_FPS,
            self.FPS
        )

        self.cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1
        )

        if not self.cap.isOpened():

            raise RuntimeError(
                "Cannot open camera"
            )

        self.get_logger().info(
            "Camera opened"
        )

        # ====================================================
        # BIRD VIEW SIZE
        # ====================================================

        self.BIRD_W = int(
            self.ARENA_SIZE_MM
            * self.BIRD_SCALE
            + 2 * self.BIRD_PADDING
        )

        self.BIRD_H = int(
            self.ARENA_SIZE_MM
            * self.BIRD_SCALE
            + 2 * self.BIRD_PADDING
        )

        # ====================================================
        # HOMOGRAPHY FILTER
        # ====================================================

        self.filtered_H = None

        # ====================================================
        # SERVICE
        # ====================================================

        self.create_service(

            CaptureWorkspace,

            "/capture_workspace",

            self.capture_workspace_callback
        )

        self.get_logger().info(
            "Detector node ready"
        )

    # ========================================================
    # HELPERS
    # ========================================================

    def smooth_h(self, H_new, H_old, alpha):

        if H_old is None:
            return H_new

        return (
            alpha * H_new
            +
            (1.0 - alpha) * H_old
        )

    def reorder_corners(self, corners):

        TL = corners[1]
        TR = corners[0]
        BL = corners[2]
        BR = corners[3]

        return np.array([

            TL,
            TR,
            BR,
            BL

        ], dtype=np.float32)

    def build_world_corners(self, center_xy):

        cx, cy = center_xy

        h = self.TAG_SIZE_MM / 2.0

        TL = [cx - h, cy - h]
        TR = [cx + h, cy - h]
        BR = [cx + h, cy + h]
        BL = [cx - h, cy + h]

        return np.array([

            TL,
            TR,
            BR,
            BL

        ], dtype=np.float32)

    # ========================================================
    # QUADRATIC CORRECTION
    # ========================================================

    def quadratic_correct(self, x_raw, y_raw):

        qx = self.qx
        qy = self.qy

        x_corrected = (

            qx["c0"]
            +
            qx["c1"] * x_raw
            +
            qx["c2"] * y_raw
            +
            qx["c3"] * (x_raw ** 2)
            +
            qx["c4"] * x_raw * y_raw
            +
            qx["c5"] * (y_raw ** 2)
        )

        y_corrected = (

            qy["c0"]
            +
            qy["c1"] * x_raw
            +
            qy["c2"] * y_raw
            +
            qy["c3"] * (x_raw ** 2)
            +
            qy["c4"] * x_raw * y_raw
            +
            qy["c5"] * (y_raw ** 2)
        )

        return x_corrected, y_corrected

    # ========================================================
    # ROBOT SYNC
    # ========================================================

    def sync_to_robot(self, x, y):

        c = math.cos(
            self.rotation_rad
        )

        s = math.sin(
            self.rotation_rad
        )

        xr = (c * x) - (s * y)

        yr = (s * x) + (c * y)

        xr += self.tx
        yr += self.ty

        return xr, yr

    # ========================================================
    # OBJECT DETECTION
    # ========================================================

    def detect_color_objects(
        self,
        hsv,
        color_name,
        lower,
        upper
    ):

        mask = cv2.inRange(
            hsv,
            lower,
            upper
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self.kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            self.kernel
        )

        contours, _ = cv2.findContours(

            mask,

            cv2.RETR_EXTERNAL,

            cv2.CHAIN_APPROX_SIMPLE
        )

        objects = []

        for cnt in contours:

            area = cv2.contourArea(cnt)

            if area < self.MIN_AREA:
                continue

            perimeter = cv2.arcLength(
                cnt,
                True
            )

            if perimeter <= 0:
                continue

            circularity = (

                4
                * np.pi
                * area

            ) / (

                perimeter
                * perimeter
            )

            if circularity < self.MIN_CIRCULARITY:
                continue

            M = cv2.moments(cnt)

            if M["m00"] == 0:
                continue

            cx = int(
                M["m10"] / M["m00"]
            )

            cy = int(
                M["m01"] / M["m00"]
            )

            rect = cv2.minAreaRect(cnt)

            angle = rect[-1]

            objects.append({

                "cx": cx,
                "cy": cy,
                "area": area,
                "angle": angle,
                "color": color_name
            })

        return objects

    # ========================================================
    # CAPTURE CALLBACK
    # ========================================================

    def capture_workspace_callback(
        self,
        request,
        response
    ):

        start_time = self.get_clock().now()

        ret, frame = self.cap.read()

        if not ret:

            response.success = False
            response.capture_time_ms = 0.0

            return response

        # ====================================================
        # UNDISTORT
        # ====================================================

        frame = cv2.undistort(
            frame,
            self.K,
            self.dist
        )

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        tags = self.detector.detect(
            gray
        )

        image_points = []
        world_points = []

        # ====================================================
        # PROCESS TAGS
        # ====================================================

        for tag in tags:

            if tag.decision_margin < self.MIN_MARGIN:
                continue

            tag_id = tag.tag_id

            if tag_id not in self.TAG_CENTERS:
                continue

            corners = tag.corners.astype(
                np.float32
            )

            img_pts = self.reorder_corners(
                corners
            )

            wrld_pts = self.build_world_corners(
                self.TAG_CENTERS[tag_id]
            )

            for i in range(4):

                image_points.append(
                    img_pts[i]
                )

                world_points.append(
                    wrld_pts[i]
                )

        if len(image_points) < 4:

            response.success = False
            response.capture_time_ms = 0.0

            return response

        image_points = np.array(
            image_points,
            dtype=np.float32
        )

        world_points = np.array(
            world_points,
            dtype=np.float32
        )

        dst = np.copy(world_points)

        dst *= self.BIRD_SCALE

        dst[:, 0] += self.BIRD_W // 2
        dst[:, 1] += self.BIRD_H // 2

        H, mask = cv2.findHomography(

            image_points,

            dst,

            cv2.RANSAC,

            3.0
        )

        if H is None:

            response.success = False
            response.capture_time_ms = 0.0

            return response

        self.filtered_H = self.smooth_h(

            H,

            self.filtered_H,

            self.H_ALPHA
        )

        # ====================================================
        # WARP
        # ====================================================

        full_bird = cv2.warpPerspective(

            frame,

            self.filtered_H,

            (
                self.BIRD_W,
                self.BIRD_H
            )
        )

        center_x = self.BIRD_W // 2
        center_y = self.BIRD_H // 2

        xmin = center_x - self.CROP_SIZE
        xmax = center_x + self.CROP_SIZE

        ymin = center_y - self.CROP_SIZE
        ymax = center_y + self.CROP_SIZE

        bird = full_bird[
            ymin:ymax,
            xmin:xmax
        ]

        hsv = cv2.cvtColor(
            bird,
            cv2.COLOR_BGR2HSV
        )

        detected_objects = []

        # ====================================================
        # COLOR DETECTION
        # ====================================================

        for color_name, cfg in self.color_configs.items():

            lower = np.array(
                cfg["lower_hsv"],
                dtype=np.uint8
            )

            upper = np.array(
                cfg["upper_hsv"],
                dtype=np.uint8
            )

            objects = self.detect_color_objects(

                hsv,

                color_name,

                lower,

                upper
            )

            detected_objects.extend(
                objects
            )

        # ====================================================
        # SORT OBJECTS
        # ====================================================

        detected_objects = sorted(

            detected_objects,

            key=lambda o: (
                o["cy"],
                o["cx"]
            )
        )

        # ====================================================
        # BUILD RESPONSE
        # ====================================================

        ros_objects = []

        for obj_id, obj in enumerate(
            detected_objects
        ):

            cx = obj["cx"]
            cy = obj["cy"]

            angle = obj["angle"]

            color = obj["color"]

            # ================================================
            # RAW MM
            # ================================================

            x_raw = (
                cx + xmin - self.BIRD_W // 2
            ) / self.BIRD_SCALE

            y_raw = (
                cy + ymin - self.BIRD_H // 2
            ) / self.BIRD_SCALE

            # ================================================
            # QUADRATIC CORRECTION
            # ================================================

            x_corrected, y_corrected = (
                self.quadratic_correct(
                    x_raw,
                    y_raw
                )
            )

            # ================================================
            # ROBOT SYNC
            # ================================================

            x_robot, y_robot = (
                self.sync_to_robot(
                    x_corrected,
                    y_corrected
                )
            )

            # ================================================
            # BUILD MESSAGE
            # ================================================

            det = DetectedObject()

            det.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            det.position = Point()

            det.position.x = float(
                x_robot
            )

            det.position.y = float(
                y_robot
            )

            det.position.z = 0.0

            det.orientation = float(
                angle
            )

            det.color = color

            det.object_id = int(
                obj_id
            )

            ros_objects.append(det)

        end_time = self.get_clock().now()

        dt = (
            end_time
            -
            start_time
        ).nanoseconds / 1e6

        response.success = True

        response.objects = ros_objects

        response.capture_time_ms = float(
            dt
        )

        return response

    # ========================================================
    # CLEANUP
    # ========================================================

    def destroy_node(self):

        if self.cap is not None:
            self.cap.release()

        super().destroy_node()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = DetectorNode()

    try:

        rclpy.spin(node)

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()