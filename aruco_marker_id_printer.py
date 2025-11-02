#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class ArucoMarkerIDPrinter:
    """Subscribe to a compressed camera feed and print detected ArUco marker IDs."""

    def __init__(self):
        self.bridge = CvBridge()

        image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw/compressed")
        dictionary_name = rospy.get_param("~dictionary", "DICT_4X4_50")
        print_cooldown = rospy.get_param("~print_cooldown", 2.0)

        self.last_print_times = {}
        self.cooldown = rospy.Duration.from_sec(print_cooldown)

        self.dictionary = self._load_dictionary(dictionary_name)
        self.detector = self._build_detector()

        queue_size = rospy.get_param("~queue_size", 1)
        rospy.Subscriber(
            image_topic,
            CompressedImage,
            self._image_callback,
            queue_size=queue_size,
            tcp_nodelay=True,
        )

        rospy.loginfo(
            "aruco_marker_id_printer started; listening to %s using %s dictionary",
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
        return aruco_module.getPredefinedDictionary(dictionary_id)

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

        if ids is None:
            return

        now = rospy.Time.now()

        for marker_id in ids.flatten():
            last_print = self.last_print_times.get(marker_id)
            if last_print is None or now - last_print > self.cooldown:
                rospy.loginfo("Detected ArUco marker ID: %d", marker_id)
                self.last_print_times[marker_id] = now


def main():
    rospy.init_node("aruco_marker_id_printer", anonymous=False)
    ArucoMarkerIDPrinter()
    rospy.spin()


if __name__ == "__main__":
    main()
