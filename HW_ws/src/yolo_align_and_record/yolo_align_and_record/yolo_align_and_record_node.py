#!/usr/bin/env python3                                       # 스크립트 실행 헤더
# yolo_align_and_record.py (ROS2 Foxy/Humble 호환)           # 파일 설명

import rclpy                                                  # ROS2 파이썬 클라이언트
from rclpy.node import Node                                   # ROS2 노드 베이스 클래스
from geometry_msgs.msg import Point, Twist                    # 픽셀 중심/속도 메시지 타입
from sensor_msgs.msg import LaserScan                         # 라이다 스캔 메시지 타입
from visualization_msgs.msg import Marker                     # RViz 마커 메시지 타입
from std_msgs.msg import Bool                                 # 일시정지/재개 신호용 Bool
from tf2_ros import Buffer, TransformListener                 # TF 버퍼/리스너
import numpy as np                                            # 수치 계산
import math                                                   # 수학 함수
from nav2_msgs.srv import SaveMap                             # Nav2 맵 저장 서비스
from rclpy.duration import Duration                           # 시간 비교용 Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy  # QoS

class YoloAlignAndRecord(Node):                               # 정렬→전진→근접→기록→재개 노드 클래스
    def __init__(self):                                       # 생성자
        """
        요약: 노드 초기화. 파라미터 선언·읽기, 퍼블리셔/구독자/서비스/TF/타이머 설정, 내부 상태 초기화.
        입력: 없음
        반환: 없음
        부작용: ROS2 노드, 통신 인터페이스, 타이머 생성
        """
        super().__init__('yolo_align_and_record')             # 노드 이름 설정

        # ───────── 파라미터 선언 ─────────
        self.declare_parameter('img_width', 640)              # 영상 가로 픽셀
        self.declare_parameter('img_height', 480)             # 영상 세로 픽셀
        self.declare_parameter('align_px_tol', 15)            # 정렬 허용 오차(px)
        self.declare_parameter('k_ang', 0.5)                  # yaw 제어 게인
        self.declare_parameter('fwd_speed', 0.25)             # 전진 속도(m/s)
        self.declare_parameter('approach_dist', 0.1)          # 근접 임계값(m)
        self.declare_parameter('front_fov_deg', 15.0)         # 전방 섹터 각도(deg)
        self.declare_parameter('save_map', True)              # 맵 저장 여부
        self.declare_parameter('map_saver_service', '/map_saver/save_map')  # 맵 저장 서비스명
        self.declare_parameter('save_to', 'auto_mapper_object')            # 저장 prefix
        self.declare_parameter('px_timeout_sec', 1.5)         # YOLO 픽셀 타임아웃(s)

        # ───────── 파라미터 읽기(안전 파서) ─────────
        def _get_num_param(name: str, as_float: bool = True):
            """숫자 파라미터 안전 추출: double 우선, 그 다음 integer"""
            pv = self.get_parameter(name).get_parameter_value()
            if hasattr(pv, 'double_value'):
                dv = pv.double_value
                # Foxy에서는 unset double이 0.0인 경우 없음, 그래도 방어
                if dv is not None and not (as_float and (hasattr(math, 'isnan') and math.isnan(dv))):
                    return float(dv) if as_float else dv
            if hasattr(pv, 'integer_value'):
                iv = pv.integer_value
                return float(iv) if as_float else int(iv)
            raise ValueError(f'Parameter "{name}" has no numeric value')

        gp = lambda n: self.get_parameter(n).get_parameter_value()                       # 원시 값 접근
        self.img_width     = int(gp('img_width').integer_value)
        self.img_height    = int(gp('img_height').integer_value)
        self.align_px_tol  = int(gp('align_px_tol').integer_value)
        self.k_ang         = _get_num_param('k_ang', True)
        self.fwd_speed     = _get_num_param('fwd_speed', True)
        self.approach_dist = _get_num_param('approach_dist', True)
        self.front_fov_rad = math.radians(_get_num_param('front_fov_deg', True))
        self.save_map      = bool(gp('save_map').bool_value)
        self.map_saver_name = gp('map_saver_service').string_value
        self.save_to        = gp('save_to').string_value
        self.px_timeout_sec = _get_num_param('px_timeout_sec', True)

        # ───────── 퍼블리셔/서브스크라이버 ─────────
        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pause_pub  = self.create_publisher(Bool, '/pause_flag', 10)
        self.marker_pub = self.create_publisher(Marker, '/object_area_marker', 1)

        # LaserScan은 SensorData QoS로 구독(베스트에포트 드라이버 호환)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.cb_scan, sensor_qos)
        self.sub_px   = self.create_subscription(Point, '/yolo/center_px', self.cb_px, 10)

        # ───────── TF 설정 ─────────
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        # ───────── 내부 상태 ─────────
        self.cx = self.img_width * 0.5
        self.last_px = None
        self.last_px_stamp = None
        self.last_scan = None
        # 상태: IDLE(자동 매핑 진행), ALIGN(정렬), FORWARD(전진)
        self.state = 'IDLE'
        self.align_stable_cnt = 0
        self.mapper_paused = False  # 맵퍼 일시정지 상태 플래그

        # ───────── 맵 저장 클라이언트 ─────────
        self.save_cli = self.create_client(SaveMap, self.map_saver_name)

        # ───────── 제어 루프 타이머 ─────────
        self.timer = self.create_timer(0.05, self._spin_once)  # 20Hz

        # ───────── 시작 로그 ─────────
        self.get_logger().info(
            f'yolo_align_and_record started ({self.img_width}x{self.img_height}, center={self.cx:.1f}px).'
        )

    def cb_px(self, msg: Point):
        """
        요약: YOLO 픽셀(u,v) 갱신. IDLE일 때만 ALIGN으로 전환하며, AutoMapper를 일시정지.
        입력: geometry_msgs/Point (x=u, y=v)
        반환: 없음
        상태: IDLE → ALIGN. ALIGN/FORWARD에서는 상태 유지(디바운스).
        """
        self.last_px = (int(msg.x), int(msg.y))
        self.last_px_stamp = self.get_clock().now()


        # IDLE에서만 정렬로 진입: AutoMapper를 일시정지하고 정지 명령
        if self.state == 'IDLE':
            self._pause_mapper(True)    # AutoMapper 일시정지
            self._stop_cmd()            # 남은 속도 제거
            self.state = 'ALIGN'
            self.align_stable_cnt = 0

    def cb_scan(self, msg: LaserScan):
        """
        요약: 최신 라이다 스캔을 저장하여 전진 단계에서 전방 최소거리 판정에 사용.
        입력: sensor_msgs/LaserScan
        반환: 없음
        """
        self.last_scan = msg

    def _spin_once(self):
        """20 Hz 메인 루프. 픽셀 타임아웃 감시 및 상태 처리."""
        # 현재 상태에 따라 flag 퍼블리시 (상시)
        flag = Bool()
        flag.data = (self.state in ['ALIGN', 'FORWARD'])  # 간섭 중이면 1, 아니면 0
        self.pause_pub.publish(flag)

        # 픽셀 타임아웃 처리
        if self.state == 'ALIGN' and self._px_timed_out():
            self.get_logger().warn('YOLO pixel timeout → resume AutoMapper and IDLE.')
            self._stop_cmd()
            self.state = 'IDLE'
            return

        # 상태별 동작
        if self.state == 'ALIGN':
            self._do_align()
        elif self.state == 'FORWARD':
            self._do_forward()

        

    def _px_timed_out(self) -> bool:
        """
        요약: 마지막 YOLO 픽셀 수신 시각과 현재 시각 차이를 비교해 타임아웃 여부를 반환.
        입력: 없음
        반환: bool (타임아웃이면 True)
        """
        if self.last_px_stamp is None:
            return True
        now = self.get_clock().now()
        return (now - self.last_px_stamp) > Duration(seconds=self.px_timeout_sec)

    def _do_align(self):
        """
        요약: YOLO 픽셀의 가로 오차로 yaw 비례제어. 허용 오차 이내 5프레임 연속이면 FORWARD로 전환.
        입력: last_px
        반환: 없음
        상태: ALIGN → FORWARD (정렬 안정 시)
        """
        if self.last_px is None:
            return
        u, _ = self.last_px
        err_px = (u - self.cx)

        tw = Twist()
        tw.linear.x = 0.0
        norm = max(self.cx, 1.0)
        tw.angular.z = - self.k_ang * (err_px / norm)

        # 안정 판정
        if abs(err_px) <= self.align_px_tol:
            self.align_stable_cnt += 1
        else:
            self.align_stable_cnt = 0

        # 각속도 제한
        tw.angular.z = float(np.clip(tw.angular.z, -0.8, 0.8))
        self.cmd_pub.publish(tw)

        if self.align_stable_cnt >= 5:
            self.state = 'FORWARD'
            self.get_logger().info('Aligned. Switching to FORWARD.')

    def _do_forward(self):
        """
        직진하며 전방 섹터 최소거리 감시. 임계 이하면 정지→마커→(옵션)맵 저장→AutoMapper 재개→IDLE.
        """
        if self.last_scan is None:
            return

        min_front = self._min_front_range(self.last_scan, self.front_fov_rad)

        # 정지 임계값: 센서 최소거리보다 살짝 크게
        stop_thresh = max(self.approach_dist, self.last_scan.range_min + 0.02)

        # 디버그: 최소거리 로깅(필요시 주석 해제)
        # self.get_logger().info(f'min_front={min_front:.3f} m, stop_thresh={stop_thresh:.3f} m')

        if (min_front is not None) and (min_front <= stop_thresh):
            self._stop_cmd()
            pos = self._current_pose_in_map()
            if pos is not None:
                self._publish_circle_marker(pos, radius=2.0)
                if self.save_map:
                    self._try_save_map()
            self._pause_mapper(False)  # AutoMapper 재개
            self.state = 'IDLE'
            self.get_logger().info('Reached object. Marked, saved (if set), resumed AutoMapper → IDLE.')
            return

        # 계속 전진
        tw = Twist()
        tw.linear.x = self.fwd_speed
        tw.angular.z = 0.0
        self.cmd_pub.publish(tw)


    def _min_front_range(self, scan: LaserScan, fov_rad: float):
        """
        요약: 라이다 스캔에서 유효 거리만 추려 전방 섹터(±fov_rad) 안의 최소 거리 계산.
        입력: LaserScan, fov_rad(라디안)
        반환: float 또는 None
        """
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        N = ranges.shape[0]
        if N == 0:
            return None
        angles = scan.angle_min + np.arange(N, dtype=np.float32) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        if not np.any(valid):
            return None
        angles = angles[valid]
        ranges = ranges[valid]
        front_mask = (angles >= -fov_rad) & (angles <= fov_rad)
        if not np.any(front_mask):
            return None
        return float(np.min(ranges[front_mask]))

    def _current_pose_in_map(self):
        """
        요약: TF에서 map ← base_link 변환을 조회하여 현재 위치를 얻어 반환.
        입력: 없음
        반환: (x, y, z) 또는 None
        """
        try:
            tf = self.tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z
            return (x, y, z)
        except Exception as e:
            self.get_logger().warn(f'TF(map<-base_link) failed: {e}')
            return None

    def _publish_circle_marker(self, pos_xyz, radius=2.0):
        """
        요약: map 좌표계 기준 중심 pos_xyz, 반지름 radius의 원기둥 마커 발행.
        입력: pos_xyz (x,y,z), radius
        반환: 없음
        """
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'object_area'
        m.id = 1  # 고정 ID: 최신 마커로 갱신
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = pos_xyz[0]
        m.pose.position.y = pos_xyz[1]
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = radius * 2.0
        m.scale.y = radius * 2.0
        m.scale.z = 0.05
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 0.35
        self.marker_pub.publish(m)

    def _pause_mapper(self, yes: bool):
        """
        요약: AutoMapper에 일시정지(True)/재개(False) 신호를 퍼블리시. 중복 전송 방지.
        입력: yes (bool)
        반환: 없음
        """
        if yes and self.mapper_paused:
            return
        if (not yes) and (not self.mapper_paused):
            return
        self.pause_pub.publish(Bool(data=yes))
        self.mapper_paused = yes
        self.get_logger().info('Sent pause to AutoMapper.' if yes else 'Sent resume to AutoMapper.')

    def _try_save_map(self):
        """
        요약: Nav2 SaveMap 서비스를 비동기로 호출해 현재 맵 저장 시도.
        입력: 없음
        반환: 없음
        """
        if not self.save_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('SaveMap service not available (2s).')
            return
        req = SaveMap.Request()
        req.map_topic = 'map'
        req.map_url = self.save_to
        req.image_format = 'pgm'
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65
        try:
            self.save_cli.call_async(req)
        except Exception as e:
            self.get_logger().warn(f'SaveMap call failed: {e}')

    def _stop_cmd(self):
        """
        요약: 즉시 정지 명령을 퍼블리시하기 위해 영 벡터 Twist를 발행.
        입력: 없음
        반환: 없음
        """
        self.cmd_pub.publish(Twist())

def main():
    """
    요약: ROS2 초기화, 노드 생성·스핀·종료.
    입력: 없음
    반환: 없음
    """
    rclpy.init()
    node = YoloAlignAndRecord()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
