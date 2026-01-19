import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.map = None
        self.map_received = False
        self.goal_in_progress = False
        self.robot_position = (0.0, 0.0)

        # 반복 감지용 변수
        self.last_goal = None
        self.repeated_goal_count = 0
        self.max_repeats = 3         # 반복 허용 횟수
        self.min_repeat_distance = 0.3  # 너무 가까운 목표 반복 기준

        self.get_logger().info('🚀 Frontier Explorer 노드 시작됨')

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def map_callback(self, msg):
        self.map = msg
        self.map_received = True

        if not self.goal_in_progress:
            self.send_next_frontier()

    def send_next_frontier(self):
        self.get_logger().info('🧭 send_next_frontier() 호출됨')

        if not self.map_received:
            self.get_logger().warn('❗ 맵이 아직 없음')
            return

        # 프론티어 리스트 가져오기
        frontiers = self.get_all_frontiers()
        if not frontiers:
            self.get_logger().warn('❗ 프론티어 없음')
            return

        rx, ry = self.robot_position

        # 프론티어를 거리순으로 정렬 (가까운 → 먼)
        frontiers.sort(key=lambda pt: (pt[0] - rx)**2 + (pt[1] - ry)**2)

        # 기본 선택: 가장 가까운 프론티어
        selected = frontiers[0]

        # 이전 목표와 너무 가까운 경우 → 반복 횟수 증가
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

        # 반복 횟수 초과 시 → 가장 먼 프론티어 선택
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
        self.nav_to_pose_client.wait_for_server()
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
        result = future.result().result
        status = future.result().status

        if status == 4:
            self.get_logger().info('🎯 목표 도달 완료')
        else:
            self.get_logger().warn(f'⚠️ 목표 실패 or 도달 불가 (status={status})')

        self.goal_in_progress = False
        self.send_next_frontier()

    def feedback_cb(self, feedback_msg):
        pass

    def get_all_frontiers(self):
        """모든 프론티어 후보를 반환"""
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin = self.map.info.origin

        data = np.array(self.map.data).reshape((height, width))
        frontiers = []

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y, x] == 0:
                    neighbors = data[y-1:y+2, x-1:x+2].flatten()
                    if -1 in neighbors:
                        wx = origin.position.x + x * resolution
                        wy = origin.position.y + y * resolution
                        frontiers.append((wx, wy))

        return frontiers

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
