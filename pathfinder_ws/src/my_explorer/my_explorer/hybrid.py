import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
import numpy as np
from scipy.ndimage import label

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        # 초기화 변수
        self.map = None
        self.map_received = False
        self.goal_in_progress = False
        self.robot_pose = None
        self.exploration_complete = False

        # 반복 감지용 변수 (사용자 코드에서 가져옴)
        self.last_goal = None
        self.repeated_goal_count = 0
        self.max_repeats = 3
        self.min_repeat_distance = 0.3

        # 속도 최적화용 샘플링 간격 (내 코드에서 가져옴)
        self.sample_interval = 5

        self.get_logger().info('🚀 Frontier Explorer 노드 시작됨')

        # Nav2 액션 클라이언트
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # 맵 및 로봇 위치 구독
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def map_callback(self, msg):
        if self.exploration_complete:
            return

        self.map = msg
        self.map_received = True

        if not self.goal_in_progress:
            self.send_next_frontier()

    def send_next_frontier(self):
        self.get_logger().info('🧭 send_next_frontier() 호출됨')

        if not self.map_received:
            self.get_logger().warn('❗ 맵이 아직 없음')
            return

        if not self.robot_pose:
            self.get_logger().warn('❗ 로봇 위치 정보 대기 중...')
            return

        # 프론티어 리스트 가져오기
        frontiers = self.get_all_frontiers()
        if not frontiers:
            self.get_logger().info('🏁 탐색 완료: 더 이상 프론티어 없음')
            self.exploration_complete = True
            return

        rx = self.robot_pose.position.x
        ry = self.robot_pose.position.y

        # 프론티어를 거리순으로 정렬 (가까운 → 먼)
        frontiers.sort(key=lambda pt: (pt[0] - rx)**2 + (pt[1] - ry)**2)

        # 기본 선택: 가장 가까운 프론티어
        selected = frontiers[0]

        # 반복 감지 로직
        if self.last_goal is not None:
            gx, gy = self.last_goal
            dist = np.hypot(selected[0] - gx, selected[1] - gy)
            if dist < self.min_repeat_distance:
                self.repeated_goal_count += 1
                self.get_logger().warn(f'⚠️ 가까운 목표 반복됨 ({self.repeated_goal_count}회)')
            else:
                self.repeated_goal_count = 0
        else:
            self.repeated_goal_count = 0

        # 반복 횟수 초과 시 가장 먼 프론티어 선택
        if self.repeated_goal_count >= self.max_repeats:
            selected = frontiers[-1]
            self.repeated_goal_count = 0
            self.get_logger().info(f'🔁 반복으로 인해 먼 프론티어로 전환: {selected}')

        # 목표 전송
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(selected[0])
        goal_msg.pose.pose.position.y = float(selected[1])
        goal_msg.pose.pose.orientation.w = 1.0

        self.last_goal = selected
        self.goal_in_progress = True

        self.get_logger().info(f'📍 목표 전송: {selected}')
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('❌ Nav2 액션 서버 연결 실패')
            self.goal_in_progress = False
            return

        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb
        )
        send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ 목표 거부됨')
            self.goal_in_progress = False
            return

        self.get_logger().info('✅ 목표 수락됨')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        status = future.result().status
        if status == 4:  # Goal succeeded
            self.get_logger().info('🎯 목표 도달 완료')
        else:
            self.get_logger().warn(f'⚠️ 목표 실패 or 도달 불가 (status={status})')

        self.goal_in_progress = False
        self.send_next_frontier()

    def feedback_cb(self, feedback_msg):
        pass

    def get_all_frontiers(self):
        """모든 프론티어 후보를 반환 (샘플링 적용)"""
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin = self.map.info.origin

        data = np.array(self.map.data).reshape((height, width))
        free = (data == 0)
        unknown = (data == -1)

        # 샘플링 적용
        sampled_free = free[::self.sample_interval, ::self.sample_interval]
        sampled_unknown = unknown[::self.sample_interval, ::self.sample_interval]
        labeled_free, _ = label(sampled_free)

        frontiers = []
        for y in range(1, height - 1, self.sample_interval):
            for x in range(1, width - 1, self.sample_interval):
                if free[y, x]:
                    # 주변에 미탐색 영역 확인
                    for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        ni, nj = y + di * self.sample_interval, x + dj * self.sample_interval
                        if 0 <= ni < height and 0 <= nj < width and unknown[ni, nj]:
                            wx = origin.position.x + x * resolution
                            wy = origin.position.y + y * resolution
                            frontiers.append((wx, wy))
                            break

        return frontiers

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
