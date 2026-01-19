import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # 액션 클라이언트 생성
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 맵 구독
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.map = None
        self.map_received = False

        # 최초 한 번만 시작
        self.goal_in_progress = False

        self.get_logger().info('🚀 FrontierExplorer 노드가 시작되었습니다.')

    def map_callback(self, msg):
        if not self.map_received:
            self.get_logger().info('🗺️ 맵 수신 완료')
        self.map = msg
        self.map_received = True

        # 최초 목표 전송
        if not self.goal_in_progress:
            self.send_next_frontier()

    def send_next_frontier(self):
        if not self.map_received:
            self.get_logger().warn('❗ 맵이 아직 없습니다.')
            return

        frontier_point = self.find_frontier()
        if frontier_point is None:
            self.get_logger().warn('❗ 프론티어를 찾을 수 없습니다.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(frontier_point[0])
        goal_msg.pose.pose.position.y = float(frontier_point[1])
        goal_msg.pose.pose.orientation.w = 1.0  # 단순 정면

        self.get_logger().info(f'📍 새로운 프론티어 목표: {frontier_point}')
        self.goal_in_progress = True

        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb
        )
        send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ 목표가 거부되었습니다.')
            self.goal_in_progress = False
            return

        self.get_logger().info('✅ 목표가 수락되었습니다.')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        self.get_logger().info('🎯 목표에 도달했습니다.')
        self.goal_in_progress = False
        self.send_next_frontier()  # 다음 프론티어로 재시작

    def feedback_cb(self, feedback):
        # 선택적으로 중간 경로 확인 가능
        pass

    def find_frontier(self):
        """Occupancy Grid에서 프론티어를 찾는 매우 간단한 구현"""
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin = self.map.info.origin

        data = np.array(self.map.data).reshape((height, width))
        frontiers = []

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y, x] == 0:  # free
                    neighbors = data[y-1:y+2, x-1:x+2].flatten()
                    if -1 in neighbors:
                        wx = origin.position.x + x * resolution
                        wy = origin.position.y + y * resolution
                        frontiers.append((wx, wy))

        if not frontiers:
            return None

        # 가장 가까운 프론티어 반환
        return frontiers[0]

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
