#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
siyi_camera : SIYI A8 mini RTSP -> ROS 2 image publisher (OpenCV FFMPEG backend).

이 노드는 SIYI A8 mini 카메라의 RTSP 스트림을 OpenCV의 FFMPEG 백엔드로 받아
sensor_msgs/Image (그리고 선택적으로 CompressedImage / CameraInfo) 토픽으로 발행한다.

중요 (This machine specifics):
- OpenCV 4.12.0 is built WITHOUT GStreamer, so cv2.CAP_GSTREAMER must NOT be used.
- The FFMPEG backend works: cv2.VideoCapture(url, cv2.CAP_FFMPEG).
- Low-latency / transport selection is done via the OPENCV_FFMPEG_CAPTURE_OPTIONS
  environment variable, which MUST be set BEFORE the VideoCapture object is created.

Entry point: siyi_camera = my_package.camera_node:main
"""

import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np

from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge


def _load_camera_info_from_url(url, logger=None):
    """
    camera_info_url(예: 'file:///path/to/calib.yaml' 또는 'package://pkg/...' 또는 평범한 경로)을
    읽어 sensor_msgs/CameraInfo 로 변환한다. 실패하면 None 을 반환한다.

    camera_info_manager 모듈이 항상 import 가능한 것은 아니므로(환경에 따라),
    표준 ROS calibration YAML 포맷을 직접 파싱한다.
    """
    if not url:
        return None

    path = url
    # 흔히 쓰이는 URL 스킴들을 일반 파일 경로로 변환.
    if path.startswith('file://'):
        path = path[len('file://'):]
    elif path.startswith('package://'):
        # package://<pkg>/<rel/path> -> <share_dir(pkg)>/<rel/path>
        try:
            from ament_index_python.packages import get_package_share_directory
            rest = path[len('package://'):]
            pkg, _, rel = rest.partition('/')
            path = os.path.join(get_package_share_directory(pkg), rel)
        except Exception as exc:  # noqa: BLE001
            if logger is not None:
                logger.error("Failed to resolve package:// camera_info_url '%s': %s" % (url, exc))
            return None

    if not os.path.isfile(path):
        if logger is not None:
            logger.error("camera_info_url file not found: %s" % path)
        return None

    try:
        import yaml
        with open(path, 'r') as f:
            calib = yaml.safe_load(f)
    except Exception as exc:  # noqa: BLE001
        if logger is not None:
            logger.error("Failed to parse camera_info YAML '%s': %s" % (path, exc))
        return None

    try:
        info = CameraInfo()
        info.width = int(calib.get('image_width', 0))
        info.height = int(calib.get('image_height', 0))
        info.distortion_model = str(calib.get('distortion_model', 'plumb_bob'))

        def _matrix_data(key):
            node = calib.get(key, None)
            if node is None:
                return None
            if isinstance(node, dict) and 'data' in node:
                return list(node['data'])
            if isinstance(node, (list, tuple)):
                return list(node)
            return None

        d = _matrix_data('distortion_coefficients')
        if d is not None:
            info.d = [float(x) for x in d]

        k = _matrix_data('camera_matrix')
        if k is not None and len(k) == 9:
            info.k = [float(x) for x in k]

        r = _matrix_data('rectification_matrix')
        if r is not None and len(r) == 9:
            info.r = [float(x) for x in r]

        p = _matrix_data('projection_matrix')
        if p is not None and len(p) == 12:
            info.p = [float(x) for x in p]

        return info
    except Exception as exc:  # noqa: BLE001
        if logger is not None:
            logger.error("Malformed camera_info YAML '%s': %s" % (path, exc))
        return None


class SiyiCameraNode(Node):
    """SIYI A8 mini RTSP -> ROS 2 Image publisher using OpenCV FFMPEG backend."""

    DEFAULT_RTSP_URL = 'rtsp://192.168.144.25:8554/main.264'
    SUB_STREAM_URL = 'rtsp://192.168.144.25:8554/sub.264'
    FALLBACK_WIDTH = 1920
    FALLBACK_HEIGHT = 1080

    def __init__(self):
        super().__init__('siyi_camera_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('rtsp_url', self.DEFAULT_RTSP_URL)
        self.declare_parameter('frame_id', 'siyi_a8_camera')
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('rtsp_transport', 'tcp')          # tcp | udp
        self.declare_parameter('reconnect_period_sec', 3.0)
        self.declare_parameter('publish_compressed', False)
        self.declare_parameter('fps_limit', 0.0)                 # 0.0 = no limit
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('use_sub_stream', False)          # convenience override
        self.declare_parameter('jpeg_quality', 80)               # for CompressedImage

        use_sub_stream = self.get_parameter('use_sub_stream').get_parameter_value().bool_value
        rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        # use_sub_stream 가 켜져 있고, 사용자가 기본 URL 을 그대로 두었다면 sub 스트림으로 바꿔준다.
        if use_sub_stream and rtsp_url == self.DEFAULT_RTSP_URL:
            rtsp_url = self.SUB_STREAM_URL

        self.rtsp_url = rtsp_url
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.rtsp_transport = self.get_parameter('rtsp_transport').get_parameter_value().string_value
        self.reconnect_period_sec = float(
            self.get_parameter('reconnect_period_sec').get_parameter_value().double_value)
        self.publish_compressed = self.get_parameter('publish_compressed').get_parameter_value().bool_value
        self.fps_limit = float(self.get_parameter('fps_limit').get_parameter_value().double_value)
        self.camera_info_url = self.get_parameter('camera_info_url').get_parameter_value().string_value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').get_parameter_value().integer_value)

        # ---------------- Publishers (SensorDataQoS: best-effort, depth ~10) -----------
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.image_topic, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', qos_profile_sensor_data)
        self.compressed_pub = None
        if self.publish_compressed:
            # image_transport 관례: <image_topic>/compressed
            self.compressed_pub = self.create_publisher(
                CompressedImage, self.image_topic + '/compressed', qos_profile_sensor_data)

        # ---------------- CameraInfo template ----------------
        self.camera_info_msg = _load_camera_info_from_url(self.camera_info_url, self.get_logger())
        if self.camera_info_msg is None:
            # 최소 CameraInfo: 해상도와 frame_id 만 채운다 (calibration 미제공시).
            self.camera_info_msg = CameraInfo()
            self.camera_info_msg.width = self.FALLBACK_WIDTH
            self.camera_info_msg.height = self.FALLBACK_HEIGHT
        self.camera_info_msg.header.frame_id = self.frame_id

        # ---------------- Reader thread state ----------------
        self._cap = None
        self._stop_event = threading.Event()
        self._min_period = (1.0 / self.fps_limit) if self.fps_limit > 0.0 else 0.0
        self._last_pub_monotonic = 0.0

        self.get_logger().info(
            "siyi_camera (FFMPEG backend) starting: url=%s transport=%s topic=%s "
            "compressed=%s fps_limit=%.2f" % (
                self.rtsp_url, self.rtsp_transport, self.image_topic,
                self.publish_compressed, self.fps_limit))

        self._reader_thread = threading.Thread(
            target=self._reader_loop, name='siyi_camera_reader', daemon=True)
        self._reader_thread.start()

    # --------------------------------------------------------------------- #
    def _open_capture(self):
        """
        VideoCapture 를 FFMPEG 백엔드로 연다.
        OPENCV_FFMPEG_CAPTURE_OPTIONS 는 VideoCapture 생성 '전에' 설정해야 적용된다.
        """
        transport = self.rtsp_transport if self.rtsp_transport in ('tcp', 'udp') else 'tcp'
        # max_delay 단위는 microseconds (0.5s). 저지연 + 안정적 전송(TCP) 목적.
        os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = (
            'rtsp_transport;%s|max_delay;500000' % transport)

        cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        # 내부 버퍼를 최소화하여 최신 프레임에 가깝게 유지 (백엔드에 따라 무시될 수 있음).
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:  # noqa: BLE001
            pass
        return cap

    def _reader_loop(self):
        """
        전용 백그라운드 스레드. read() 는 블로킹이므로 rclpy.spin() 을 막지 않도록
        반드시 별도 스레드에서 수행한다. open/read 실패 시 재접속 루프를 돈다.
        """
        while not self._stop_event.is_set() and rclpy.ok():
            cap = self._open_capture()
            self._cap = cap

            if not cap.isOpened():
                self.get_logger().warning(
                    "Failed to open RTSP stream '%s'. Retrying in %.1fs ..." % (
                        self.rtsp_url, self.reconnect_period_sec),
                    throttle_duration_sec=5.0)
                self._release_capture()
                self._sleep_reconnect()
                continue

            self.get_logger().info("RTSP stream opened: %s" % self.rtsp_url)

            # ----- 프레임 읽기 루프 -----
            while not self._stop_event.is_set() and rclpy.ok():
                try:
                    ret, frame = cap.read()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warning(
                        "VideoCapture.read() raised: %s" % exc,
                        throttle_duration_sec=5.0)
                    ret, frame = False, None

                if not ret or frame is None:
                    self.get_logger().warning(
                        "No frame from '%s' (stream stalled/ended). Reconnecting in %.1fs ..." % (
                            self.rtsp_url, self.reconnect_period_sec),
                        throttle_duration_sec=5.0)
                    break  # inner loop -> reconnect

                self._maybe_publish(frame)

            # 연결이 끊겼거나 종료 요청 -> 정리 후(필요시) 재접속.
            self._release_capture()
            if self._stop_event.is_set() or not rclpy.ok():
                break
            self._sleep_reconnect()

        self._release_capture()

    def _maybe_publish(self, frame):
        """fps_limit 을 적용하여(설정 시) 프레임을 발행한다."""
        if self._min_period > 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
            if (now - self._last_pub_monotonic) < self._min_period:
                return
            self._last_pub_monotonic = now
        self._publish_frame(frame)

    def _publish_frame(self, frame):
        """BGR numpy 프레임을 Image (+옵션: CompressedImage, CameraInfo) 로 발행."""
        stamp = self.get_clock().now().to_msg()

        # ----- raw Image -----
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                "cv2_to_imgmsg failed: %s" % exc, throttle_duration_sec=5.0)
            return
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        # ----- compressed Image (JPEG) -----
        if self.compressed_pub is not None:
            ok, enc = cv2.imencode(
                '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                comp = CompressedImage()
                comp.header.stamp = stamp
                comp.header.frame_id = self.frame_id
                comp.format = 'jpeg'
                comp.data = np.asarray(enc).tobytes()
                self.compressed_pub.publish(comp)
            else:
                self.get_logger().warning(
                    "cv2.imencode JPEG failed", throttle_duration_sec=5.0)

        # ----- CameraInfo (해상도를 실제 프레임에 맞춰 갱신) -----
        h, w = frame.shape[0], frame.shape[1]
        if self.camera_info_msg.width == 0:
            self.camera_info_msg.width = int(w)
        if self.camera_info_msg.height == 0:
            self.camera_info_msg.height = int(h)
        self.camera_info_msg.header.stamp = stamp
        self.camera_info_msg.header.frame_id = self.frame_id
        self.info_pub.publish(self.camera_info_msg)

    # --------------------------------------------------------------------- #
    def _sleep_reconnect(self):
        """재접속 대기. stop 이벤트가 오면 즉시 깨어난다."""
        self._stop_event.wait(timeout=self.reconnect_period_sec)

    def _release_capture(self):
        cap = self._cap
        self._cap = None
        if cap is not None:
            try:
                cap.release()
            except Exception:  # noqa: BLE001
                pass

    def stop(self):
        """리더 스레드 종료 및 capture 해제."""
        self._stop_event.set()
        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=5.0)
        self._release_capture()


def main(args=None):
    rclpy.init(args=args)
    node = SiyiCameraNode()
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
