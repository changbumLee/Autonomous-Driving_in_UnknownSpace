import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformListener, Buffer, TransformException  # 수정: TransformException을 tf2_ros에서 임포트
import numpy as np

class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # 1️⃣ Nav2 액션 클라이언트 생성
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # 2️⃣ 맵 데이터 구독
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # 3️⃣ TF를 통해 로봇 위치 얻기
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 4️⃣ 프론티어 시각화를 위한 퍼블리셔
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontiers', 10)

        # 변수 초기화
        self.map_data = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.robot_pose = None  # 로봇 현재 위치
        self.visited_frontiers = set()  # 방문한 프론티어 기록
        self.timer = self.create_timer(10.0, self.send_new_waypoint)

    def map_callback(self, msg):
        """ 맵 데이터를 받아서 프론티어 탐색에 활용 """
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.get_logger().info("맵 데이터 업데이트됨")

    def get_robot_pose(self):
        """ TF를 통해 로봇의 현재 위치를 얻음 """
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.robot_pose = (trans.transform.translation.x, trans.transform.translation.y)
            self.get_logger().info(f"로봇 위치: x={self.robot_pose[0]}, y={self.robot_pose[1]}")
        except TransformException as e:
            self.get_logger().warn(f"TF 조회 실패: {e}")
            self.robot_pose = None

    def find_frontiers(self):
        """ 탐색되지 않은 영역과 탐색된 영역의 경계를 찾아 프론티어 생성 """
        frontier_points = []
        
        if self.map_data is None:
            self.get_logger().warn("맵 데이터가 없습니다!")
            return frontier_points

        for i in range(1, self.map_data.shape[0] - 1):
            for j in range(1, self.map_data.shape[1] - 1):
                if self.map_data[i, j] == -1:  # 미지 영역 (unknown)
                    neighbors = [
                        self.map_data[i + 1, j], self.map_data[i - 1, j],
                        self.map_data[i, j + 1], self.map_data[i, j - 1]
                    ]
                    if any(n == 0 for n in neighbors):  # 주변에 탐색된 영역(known)이 있으면 프론티어
                        world_x = self.map_origin_x + (j * self.map_resolution)
                        world_y = self.map_origin_y + (i * self.map_resolution)
                        if (0 <= i < self.map_height) and (0 <= j < self.map_width):
                            frontier_points.append((world_x, world_y))
                        else:
                            self.get_logger().warn(f"프론티어 ({world_x}, {world_y})가 맵 범위를 벗어났습니다!")

        # 방문한 프론티어 제외 (좌표를 소수점 2자리로 반올림하여 비교)
        frontier_points = [
            p for p in frontier_points
            if (round(p[0], 2), round(p[1], 2)) not in self.visited_frontiers
        ]

        self.get_logger().info(f"탐지된 프론티어 수: {len(frontier_points)}")
        return frontier_points

    def publish_frontiers(self, frontier_points):
        """ 프론티어를 RViz에서 시각화 """
        marker_array = MarkerArray()
        for idx, (x, y) in enumerate(frontier_points):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.frontier_pub.publish(marker_array)

    def send_new_waypoint(self):
        """ 가장 가까운 프론티어를 새로운 웨이포인트로 설정 """
        if self.map_data is None or self.map_resolution is None:
            self.get_logger().warning("맵 데이터 또는 해상도가 없습니다!")
            return

        # 로봇 위치 업데이트
        self.get_robot_pose()
        if self.robot_pose is None:
            self.get_logger().warning("로봇 위치를 아직 수신하지 못했습니다!")
            return

        frontier_points = self.find_frontiers()
        if not frontier_points:
            self.get_logger().info("탐색할 프론티어가 없습니다!")
            return

        # 프론티어 시각화
        self.publish_frontiers(frontier_points)

        # 현재 로봇 위치에서 가장 가까운 프론티어 찾기
        robot_pos = np.array([self.robot_pose[0], self.robot_pose[1]])
        distances = [np.linalg.norm(np.array(p) - robot_pos) for p in frontier_points]
        best_frontier_idx = np.argmin(distances)
        best_frontier = frontier_points[best_frontier_idx]

        # 방문한 프론티어로 기록 (좌표를 소수점 2자리로 반올림하여 비교)
        rounded_frontier = (round(best_frontier[0], 2), round(best_frontier[1], 2))
        if rounded_frontier in self.visited_frontiers:
            self.get_logger().warn(f"이미 방문한 프론티어입니다: {rounded_frontier}")
            return
        self.visited_frontiers.add(rounded_frontier)

        # 목표 설정
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = best_frontier[0]
        goal_msg.pose.pose.position.y = best_frontier[1]
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
