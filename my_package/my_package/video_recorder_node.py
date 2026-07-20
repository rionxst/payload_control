#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
video_recorder : sensor_msgs/Image 토픽을 구독해 mp4 파일로 저장하는 노드.

시작/정지는 std_srvs/Trigger 서비스로 제어한다.
    ros2 service call /video_recorder/start_recording std_srvs/srv/Trigger
    ros2 service call /video_recorder/stop_recording  std_srvs/srv/Trigger

파일명은 start 시각 기준으로 자동 생성된다 (output_dir/prefix_YYYYmmdd_HHMMSS.mp4).

Entry point: video_recorder = my_package.video_recorder_node:main
"""

import os
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger


class VideoRecorderNode(Node):
    """Image 토픽 -> mp4 녹화. Trigger 서비스로 start/stop."""

    def __init__(self):
        super().__init__('video_recorder')

        # ---------------- Parameters ----------------
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('output_dir', os.path.expanduser('~/videos'))
        self.declare_parameter('filename_prefix', 'siyi')
        self.declare_parameter('fps', 30.0)          # VideoWriter 에 기록될 재생 fps
        self.declare_parameter('fourcc', 'mp4v')     # mp4v | avc1 (avc1 은 빌드에 따라 미지원)
        self.declare_parameter('auto_start', False)  # 노드 시작과 동시에 녹화

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.output_dir = os.path.expanduser(
            self.get_parameter('output_dir').get_parameter_value().string_value)
        self.filename_prefix = self.get_parameter('filename_prefix').get_parameter_value().string_value
        self.fps = float(self.get_parameter('fps').get_parameter_value().double_value)
        self.fourcc_str = self.get_parameter('fourcc').get_parameter_value().string_value
        auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value

        if self.fps <= 0.0:
            self.fps = 30.0

        # ---------------- Recording state ----------------
        self.bridge = CvBridge()
        self._lock = threading.Lock()       # writer/상태를 콜백과 서비스가 공유하므로 보호한다.
        self._recording = False             # start 요청됨 (writer 는 첫 프레임에서 열린다)
        self._writer = None
        self._current_path = None
        self._frame_count = 0
        self._size = None                   # (w, h) : 첫 프레임 해상도에 고정된다.

        # ---------------- Subscription / services ----------------
        self.sub = self.create_subscription(
            Image, self.image_topic, self._image_cb, qos_profile_sensor_data)
        self.create_service(Trigger, '~/start_recording', self._start_srv)
        self.create_service(Trigger, '~/stop_recording', self._stop_srv)

        self.get_logger().info(
            "video_recorder ready: topic=%s dir=%s fps=%.2f fourcc=%s" % (
                self.image_topic, self.output_dir, self.fps, self.fourcc_str))

        if auto_start:
            ok, msg = self._start_recording()
            self.get_logger().info("auto_start: %s" % msg) if ok else \
                self.get_logger().error("auto_start failed: %s" % msg)

    # --------------------------------------------------------------------- #
    def _image_cb(self, msg):
        with self._lock:
            if not self._recording:
                return
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    "imgmsg_to_cv2 failed: %s" % exc, throttle_duration_sec=5.0)
                return

            h, w = frame.shape[0], frame.shape[1]

            if self._writer is None:
                # 녹화 시작 후 첫 프레임: 이 해상도로 파일을 연다.
                if not self._open_writer((int(w), int(h))):
                    self._recording = False
                    return

            if (w, h) != self._size:
                # VideoWriter 는 해상도 변경을 허용하지 않으므로 맞춰서 리사이즈한다.
                self.get_logger().warning(
                    "Frame size %dx%d != recording size %dx%d; resizing." % (
                        w, h, self._size[0], self._size[1]),
                    throttle_duration_sec=5.0)
                frame = cv2.resize(frame, self._size)

            self._writer.write(frame)
            self._frame_count += 1

    # --------------------------------------------------------------------- #
    def _open_writer(self, size):
        """_lock 을 잡은 상태에서 호출. 첫 프레임 해상도로 mp4 파일을 연다."""
        try:
            os.makedirs(self.output_dir, exist_ok=True)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                "Cannot create output_dir '%s': %s" % (self.output_dir, exc))
            return False

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.output_dir, '%s_%s.mp4' % (self.filename_prefix, stamp))

        fourcc = cv2.VideoWriter_fourcc(*self.fourcc_str)
        writer = cv2.VideoWriter(path, fourcc, self.fps, size)
        if not writer.isOpened():
            self.get_logger().error(
                "VideoWriter failed to open '%s' (fourcc=%s)" % (path, self.fourcc_str))
            return False

        self._writer = writer
        self._current_path = path
        self._size = size
        self._frame_count = 0
        self.get_logger().info(
            "Recording to %s (%dx%d @ %.2f fps)" % (path, size[0], size[1], self.fps))
        return True

    def _start_recording(self):
        """(성공여부, 메시지) 반환. 실제 파일은 첫 프레임 수신 시 열린다."""
        with self._lock:
            if self._recording:
                return False, "Already recording: %s" % (self._current_path or '(pending first frame)')
            self._recording = True
            return True, "Recording armed; file opens on first frame from '%s'." % self.image_topic

    def _stop_recording(self):
        with self._lock:
            if not self._recording:
                return False, "Not recording."
            self._recording = False
            path, count = self._current_path, self._frame_count
            if self._writer is not None:
                try:
                    self._writer.release()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error("VideoWriter.release() failed: %s" % exc)
                self._writer = None
                self._current_path = None
                return True, "Recording stopped: %s (%d frames)" % (path, count)
            return True, "Recording cancelled (no frames were received)."

    # --------------------------------------------------------------------- #
    def _start_srv(self, request, response):
        ok, msg = self._start_recording()
        response.success = ok
        response.message = msg
        self.get_logger().info(msg) if ok else self.get_logger().warning(msg)
        return response

    def _stop_srv(self, request, response):
        ok, msg = self._stop_recording()
        response.success = ok
        response.message = msg
        self.get_logger().info(msg) if ok else self.get_logger().warning(msg)
        return response

    def stop(self):
        """노드 종료 시 파일이 깨지지 않도록 반드시 release 한다."""
        ok, msg = self._stop_recording()
        if ok:
            self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
