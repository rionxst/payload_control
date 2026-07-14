#!/usr/bin/env python3

import time
import threading
import serial

import rclpy
from rclpy.node import Node
from mission_interface.srv import SetServoAngle
from std_srvs.srv import Trigger, SetBool

# grapper_control.ino 의 SET,<id>,<value> 프로토콜과 일치해야 함
SERVO_ID_GRIPPER = 0
SERVO_ID_YAW = 1


class ServoSerialBridge(Node):
    def __init__(self):
        super().__init__('servo_serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('timeout_sec', 4.0)
        # True 이면 그리퍼/요 명령 전에 ARM 이 안 되어 있을 때 자동으로 ARM 을 보냄
        self.declare_parameter('auto_arm', False)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)

        self.lock = threading.Lock()

        # 펌웨어 상태 미러 (STATUS?/응답으로 갱신)
        self.armed = False
        self.gripper_opened = True   # 펌웨어는 setup()에서 openGripper()로 시작
        self.yaw_angle = 90   # grapper_control.ino 홈(전원 인가 시) 기준각 = 90°(중립)

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=0.1
        )

        # ESP32 reset 대기
        time.sleep(2.0)
        self.ser.reset_input_buffer()

        # ---- 서비스 ----
        # 그리퍼 open/close: data=True → OPEN, data=False → CLOSE
        self.gripper_srv = self.create_service(
            SetBool, '/gripper/set', self.handle_gripper
        )
        # ARM/DISARM: data=True → ARM, data=False → DISARM
        self.arm_srv = self.create_service(
            SetBool, '/gripper/arm', self.handle_arm
        )
        # 요(yaw) 서보 각도 0~180
        self.yaw_srv = self.create_service(
            SetServoAngle, '/gripper/set_yaw', self.handle_set_yaw
        )
        # 상태 조회
        self.status_srv = self.create_service(
            Trigger, '/gripper/status', self.handle_status
        )
        # 연결 확인
        self.ping_srv = self.create_service(
            Trigger, '/gripper/ping', self.handle_ping
        )

        self.get_logger().info(
            f'Gripper serial bridge started: port={self.port}, '
            f'baud={self.baud}, auto_arm={self.auto_arm}'
        )

    # ------------------------------------------------------------------
    # 시리얼 트랜잭션
    # ------------------------------------------------------------------
    def transact(self, command: str):
        with self.lock:
            self.ser.reset_input_buffer()

            cmd = command.strip() + '\n'
            self.ser.write(cmd.encode('utf-8'))

            deadline = time.time() + self.timeout_sec
            received_lines = []

            while time.time() < deadline:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()

                if not line:
                    continue

                received_lines.append(line)

                if line.startswith('OK,') or line.startswith('ERR,'):
                    return line, received_lines

            return '', received_lines

    def ensure_armed(self):
        """auto_arm 이 켜져 있고 아직 ARM 안 됐으면 ARM 을 시도한다."""
        if self.armed or not self.auto_arm:
            return
        line, _ = self.transact('ARM')
        if line.startswith('OK,ARMED'):
            self.armed = True

    # ------------------------------------------------------------------
    # 핸들러
    # ------------------------------------------------------------------
    def handle_gripper(self, request, response):
        self.ensure_armed()

        # 펌웨어: value==0 → CLOSE, value!=0 → OPEN
        value = 1 if request.data else 0
        line, lines = self.transact(f'SET,{SERVO_ID_GRIPPER},{value}')

        if line.startswith('OK,GRIPPER,'):
            self.gripper_opened = line.endswith('OPEN')
            response.success = True
            response.message = line
            return response

        response.success = False
        response.message = line if line else f'timeout or invalid response: {lines}'
        return response

    def handle_arm(self, request, response):
        cmd = 'ARM' if request.data else 'DISARM'
        line, lines = self.transact(cmd)

        if line.startswith('OK,ARMED'):
            self.armed = True
            response.success = True
            response.message = line
        elif line.startswith('OK,DISARMED'):
            self.armed = False
            response.success = True
            response.message = line
        else:
            response.success = False
            response.message = line if line else f'timeout or invalid response: {lines}'

        return response

    def handle_set_yaw(self, request, response):
        angle = int(request.angle)

        # grapper_control.ino 의 setYaw(표준 위치제어 서보)와 동일하게 0~180 허용
        if angle < 0 or angle > 180:
            response.success = False
            response.message = 'angle must be 0~180'
            response.current_angle = self.yaw_angle
            return response

        self.ensure_armed()

        line, lines = self.transact(f'SET,{SERVO_ID_YAW},{angle}')

        if line.startswith('OK,YAW,'):
            try:
                self.yaw_angle = int(line.split(',')[-1])
            except ValueError:
                self.yaw_angle = angle
            response.success = True
            response.message = line
            response.current_angle = self.yaw_angle
            return response

        response.success = False
        response.message = line if line else f'timeout or invalid response: {lines}'
        response.current_angle = self.yaw_angle
        return response

    def handle_status(self, request, response):
        line, lines = self.transact('STATUS?')

        if line.startswith('OK,STATUS'):
            self._update_state_from_status(line)
            response.success = True
            response.message = line
        else:
            response.success = False
            response.message = line if line else f'timeout or invalid response: {lines}'

        return response

    def handle_ping(self, request, response):
        line, lines = self.transact('PING')

        if line.startswith('OK,PONG'):
            response.success = True
            response.message = line
        else:
            response.success = False
            response.message = line if line else f'timeout or invalid response: {lines}'

        return response

    # ------------------------------------------------------------------
    def _update_state_from_status(self, line: str):
        # OK,STATUS,ARMED=1,GRIPPER=OPEN,YAW=90
        for token in line.split(','):
            if token.startswith('ARMED='):
                self.armed = token.split('=', 1)[1] not in ('0', '')
            elif token.startswith('GRIPPER='):
                self.gripper_opened = token.split('=', 1)[1] == 'OPEN'
            elif token.startswith('YAW='):
                try:
                    self.yaw_angle = int(token.split('=', 1)[1])
                except ValueError:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = ServoSerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
