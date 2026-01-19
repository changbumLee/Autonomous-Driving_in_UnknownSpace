import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
import numpy as np

class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # 1️⃣ Nav2 액션 클라이언트 생성
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # 2️⃣ 맵 데이터 구독
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.map_data = None  # 맵 데이터 저장
        self.timer = self.create_timer(10.0, self.send_new_waypoint)  # 10초마다 웨이포인트 업데이트

    def map_callback(self, msg):
        """ 맵 데이터를 받아서 프론티어 탐색에 활용 """
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)

    def find_frontiers(self):
        """ 탐색되지 않은 영역과 탐색된 영역의 경계를 찾아 프론티어 생성 """
        frontier_points = []
        
        if self.map_data is None:
            return frontier_points

        for i in range(1, self.map_data.shape[0] - 1):
            for j in range(1, self.map_data.shape[1] - 1):
                if self.map_data[i, j] == -1:  # 미지 영역 (unknown)
                    neighbors = [
                        self.map_data[i + 1, j], self.map_data[i - 1, j],
                        self.map_data[i, j + 1], self.map_data[i, j - 1]
                    ]
                    if any(n == 0 for n in neighbors):  # 주변에 탐색된 영역(known)이 있으면 프론티어
                        frontier_points.append((float(i), float(j)))  # float으로 변환

        return frontier_points

    def send_new_waypoint(self):
        """ 가장 가까운 프론티어를 새로운 웨이포인트로 설정 """
        if self.map_data is None:
            self.get_logger().warning("맵 데이터 없음!")
            return

        frontier_points = self.find_frontiers()
        if not frontier_points:
            self.get_logger().info("탐색할 프론티어가 없습니다!")
            return

        # 가장 가까운 프론티어 찾기
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(frontier_points[0][0])  # float 변환
        goal_msg.pose.pose.position.y = float(frontier_points[0][1])  # float 변환
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'새로운 웨이포인트: x={goal_msg.pose.pose.position.x}, y={goal_msg.pose.pose.position.y}')
        self.nav_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

