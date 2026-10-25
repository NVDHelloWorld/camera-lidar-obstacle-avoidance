#!/usr/bin/python3

import math
import cv2
import numpy as np
import rospy

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class CameraLidarFusion:

    def __init__(self):
        rospy.init_node("camera_lidar_fusion")

        # Controller settings
        self.stop_distance = 0.60
        self.forward_speed = 0.10
        self.turn_speed = 0.40

        # Camera horizontal field of view: approximate initial value.
        # We will verify this against the camera's calibration information.
        self.camera_fov = math.radians(60.0)

        # Latest sensor data
        self.latest_scan = None
        self.latest_detection = None
        self.last_scan_time = None
        self.last_camera_time = None

        self.bridge = CvBridge()

        # LiDAR subscriber
        self.scan_sub = rospy.Subscriber(
            "/scan",
            LaserScan,
            self.scan_callback,
            queue_size=1
        )

        # Camera subscriber
        self.camera_sub = rospy.Subscriber(
            "/camera/rgb/image_raw",
            Image,
            self.camera_callback,
            queue_size=1
        )

        # Velocity publisher
        self.cmd_pub = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=1
        )

        # Controller runs at 10 Hz
        self.timer = rospy.Timer(
            rospy.Duration(0.1),
            self.control_loop
        )

        rospy.on_shutdown(self.stop_robot)

        rospy.loginfo("Camera-LiDAR fusion controller started.")

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.last_scan_time = rospy.Time.now()

    def camera_callback(self, msg):
        """Detect a red region and estimate its horizontal image position."""

        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Red wraps around the HSV hue boundary, so use two masks.
            lower_red_1 = np.array([0, 100, 80])
            upper_red_1 = np.array([10, 255, 255])

            lower_red_2 = np.array([170, 100, 80])
            upper_red_2 = np.array([180, 255, 255])

            mask_1 = cv2.inRange(
                hsv, lower_red_1, upper_red_1
            )
            mask_2 = cv2.inRange(
                hsv, lower_red_2, upper_red_2
            )

            mask = cv2.bitwise_or(mask_1, mask_2)

            # Remove small isolated pixels.
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, kernel
            )

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            # Only accept reasonably large red regions.
            candidates = [
                contour for contour in contours
                if cv2.contourArea(contour) > 300
            ]

            self.latest_detection = None

            if candidates:
                # Use the largest detected red region.
                contour = max(
                    candidates,
                    key=cv2.contourArea
                )

                moments = cv2.moments(contour)

                if moments["m00"] > 0:
                    center_x = (
                        moments["m10"] / moments["m00"]
                    )

                    image_width = image.shape[1]

                    # Positive angle means the object is to the left.
                    normalized_x = (
                        center_x - image_width / 2.0
                    ) / (image_width / 2.0)

                    object_angle = (
                        -normalized_x * self.camera_fov / 2.0
                    )

                    self.latest_detection = {
                        "angle": object_angle,
                        "center_x": center_x,
                        "area": cv2.contourArea(contour)
                    }

            self.last_camera_time = rospy.Time.now()

        except Exception as error:
            rospy.logwarn_throttle(
                5.0,
                "Camera processing error: %s" % str(error)
            )

    def get_lidar_distance(self, scan, target_angle):
        """Find valid LiDAR measurements near the visual target angle."""

        distances = []

        # Use a small angular window around the target.
        angular_tolerance = math.radians(5.0)

        for index, distance in enumerate(scan.ranges):
            lidar_angle = (
                scan.angle_min +
                index * scan.angle_increment
            )

            # Normalize angular difference to [-pi, pi].
            angle_difference = math.atan2(
                math.sin(lidar_angle - target_angle),
                math.cos(lidar_angle - target_angle)
            )

            if abs(angle_difference) <= angular_tolerance:
                if (
                    math.isfinite(distance)
                    and scan.range_min <= distance <= scan.range_max
                ):
                    distances.append(distance)

        if not distances:
            return None

        # The nearest reading in the target direction.
        return min(distances)

    def control_loop(self, event):
        command = Twist()

        # Require recent LiDAR data.
        if (
            self.latest_scan is None
            or self.last_scan_time is None
            or (rospy.Time.now() - self.last_scan_time).to_sec() > 1.0
        ):
            rospy.logwarn_throttle(
                2.0, "Waiting for fresh LiDAR data."
            )
            self.cmd_pub.publish(command)
            return

        # Require recent camera data.
        if (
            self.last_camera_time is None
            or (rospy.Time.now() - self.last_camera_time).to_sec() > 1.0
        ):
            rospy.logwarn_throttle(
                2.0, "Waiting for fresh camera data."
            )
            self.cmd_pub.publish(command)
            return

        # Default behaviour: move forward slowly.
        command.linear.x = self.forward_speed
        command.angular.z = 0.0

        detection = self.latest_detection

        if detection is not None:
            target_angle = detection["angle"]

            distance = self.get_lidar_distance(
                self.latest_scan,
                target_angle
            )

            if distance is not None:
                rospy.loginfo_throttle(
                    1.0,
                    "Red target: angle=%.1f deg, "
                    "LiDAR distance=%.2f m",
                    math.degrees(target_angle),
                    distance
                )

                # Treat the detected red object as an obstacle.
                if distance < self.stop_distance:
                    command.linear.x = 0.0

                    # Turn away based on which side contains the target.
                    if target_angle >= 0:
                        command.angular.z = -self.turn_speed
                    else:
                        command.angular.z = self.turn_speed

                    rospy.loginfo_throttle(
                        1.0,
                        "Target too close. Avoiding obstacle."
                    )

            else:
                rospy.logwarn_throttle(
                    2.0,
                    "Red target detected, but no matching "
                    "LiDAR measurement was found."
                )

        self.cmd_pub.publish(command)

    def stop_robot(self):
        if hasattr(self, "cmd_pub"):
            self.cmd_pub.publish(Twist())


if __name__ == "__main__":
    try:
        CameraLidarFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
