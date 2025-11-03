#!/usr/bin/env python

from __future__ import absolute_import, division, print_function

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, CameraInfo
import numpy as np


class ArucoMarkerPoseDetector(object):
    """Subscribe to a compressed camera feed, estimate ArUco poses, and visualize them."""

    def __init__(self):
        self.bridge = CvBridge()

        image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw/compressed")
        dictionary_name = rospy.get_param("~dictionary", "DICT_4X4_50")
        print_cooldown = rospy.get_param("~print_cooldown", 2.0)
        camera_info_topic = rospy.get_param("~camera_info_topic", "/usb_cam/camera_info")
        self.marker_length = rospy.get_param("~marker_length", 0.05)  # meters

        self.last_print_times = {}
        self.cooldown = rospy.Duration.from_sec(print_cooldown)

        self.dictionary = self._load_dictionary(dictionary_name)
        self.detector = self._build_detector()
        self.camera_matrix = None
        self.dist_coeffs = None
        self._camera_info_logged = False

        queue_size = rospy.get_param("~queue_size", 1)
        rospy.Subscriber(
            image_topic,
            CompressedImage,
            self._image_callback,
            queue_size=queue_size,
            tcp_nodelay=True,
        )
        self.camera_info_sub = rospy.Subscriber(
            camera_info_topic,
            CameraInfo,
            self._camera_info_callback,
            queue_size=1,
        )

        rospy.loginfo(
            "aruco_marker_pose_detector started; listening to %s using %s dictionary",
            image_topic,
            dictionary_name,
        )

    def _load_dictionary(self, dictionary_name):
        aruco_module = cv2.aruco
        dictionary_id = getattr(aruco_module, dictionary_name, None)
        if dictionary_id is None:
            available = [
                attr
                for attr in dir(aruco_module)
                if attr.startswith("DICT_") and attr.isupper()
            ]
            rospy.logwarn(
                "Unknown dictionary '%s'. Falling back to DICT_4X4_50. Available options: %s",
                dictionary_name,
                ", ".join(sorted(available)),
            )
            dictionary_id = aruco_module.DICT_4X4_50
        if hasattr(aruco_module, "getPredefinedDictionary"):
            return aruco_module.getPredefinedDictionary(dictionary_id)
        # OpenCV < 3.2 fallback
        if hasattr(aruco_module, "Dictionary_get"):
            return aruco_module.Dictionary_get(dictionary_id)
        rospy.logerr("cv2.aruco does not expose a dictionary loader; check OpenCV installation.")
        raise AttributeError("Unable to load ArUco dictionary from cv2.aruco")

    def _build_detector(self):
        aruco_module = cv2.aruco

        # Handle both legacy (OpenCV < 4.7) and new detector API
        if hasattr(aruco_module, "DetectorParameters_create"):
            parameters = aruco_module.DetectorParameters_create()
        else:
            parameters = aruco_module.DetectorParameters()

        if hasattr(aruco_module, "ArucoDetector"):
            return aruco_module.ArucoDetector(self.dictionary, parameters)

        # Older API falls back to detectMarkers function
        self._legacy_parameters = parameters
        return None

    def _camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K, dtype=np.float32).reshape((3, 3))
        if len(msg.D) >= 5:
            self.dist_coeffs = np.array(msg.D[:5], dtype=np.float32)
        else:
            self.dist_coeffs = np.zeros(5, dtype=np.float32)
        if self.camera_info_sub is not None:
            self.camera_info_sub.unregister()
            self.camera_info_sub = None
        rospy.loginfo("Camera intrinsics received; pose estimation enabled.")

    def _image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as exc:
            rospy.logerr("Failed to convert image: %s", exc)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self._legacy_parameters
            )

        if ids is not None:
            rvecs = None
            tvecs = None
            if self.camera_matrix is None or self.dist_coeffs is None:
                if not self._camera_info_logged:
                    rospy.logwarn("Camera info not yet received; cannot estimate pose.")
                    self._camera_info_logged = True
            else:
                result = None
                try:
                    result = cv2.aruco.estimatePoseSingleMarkers(
                        corners,
                        self.marker_length,
                        self.camera_matrix,
                        self.dist_coeffs,
                    )
                except AttributeError:
                    pass

                rvecs = None
                tvecs = None
                if result is not None:
                    if isinstance(result, tuple) and len(result) >= 2:
                        rvecs = result[0]
                        tvecs = result[1]

            try:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            except AttributeError:
                # Older OpenCV builds may not expose drawDetectedMarkers
                pass

            now = rospy.Time.now()
            for idx, marker_id in enumerate(ids.flatten()):
                last_print = self.last_print_times.get(marker_id)
                if last_print is None or now - last_print > self.cooldown:
                    rospy.loginfo("Detected ArUco marker ID: %d", marker_id)
                    self.last_print_times[marker_id] = now
                if (
                    self.camera_matrix is not None
                    and self.dist_coeffs is not None
                    and rvecs is not None
                    and tvecs is not None
                ):
                    rvec = rvecs[idx][0]
                    tvec = tvecs[idx][0]
                    cv2.aruco.drawAxis(
                        frame,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        self.marker_length * 0.5,
                    )
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                    yaw_deg = np.degrees(yaw)

                    corner_points = corners[idx].reshape((4, 2))
                    text_origin = tuple(corner_points[0].astype(int))
                    pose_text = "ID {} | x:{:.2f} y:{:.2f} z:{:.2f} m".format(
                        marker_id, tvec[0], tvec[1], tvec[2]
                    )
                    yaw_text = "yaw:{:.1f} deg".format(yaw_deg)

                    cv2.putText(
                        frame,
                        pose_text,
                        text_origin,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        frame,
                        yaw_text,
                        (text_origin[0], text_origin[1] + 18),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )

        cv2.imshow("ArUco Detection", frame)
        cv2.waitKey(1)


def main():
    rospy.init_node("aruco_marker_pose_detector", anonymous=False)
    ArucoMarkerPoseDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
