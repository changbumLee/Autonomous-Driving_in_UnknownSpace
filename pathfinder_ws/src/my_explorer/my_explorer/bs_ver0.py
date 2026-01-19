import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformListener, Buffer, TransformException
import numpy as np

class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontiers', 10)

        self.map_data = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.robot_pose = None
        self.visited_frontiers = set()

        self.goal_in_progress = False
        self.last_goal = None
        self.repeated_goal_count = 0
        self.max_repeats = 3
        self.min_repeat_distance = 0.5
        self.timer = self.create_timer(10.0, self.send_new_waypoint)

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.get_logger().info("맵 데이터 업데이트됨")

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.robot_pose = (trans.transform.translation.x, trans.transform.translation.y)
            self.get_logger().info(f"로봇 위치: x={self.robot_pose[0]}, y={self.robot_pose[1]}")
        except TransformException as e:
            self.get_logger().warn(f"TF 조회 실패: {e}")
            self.robot_pose = None

    def find_frontiers(self):
        frontier_points = []
        if self.map_data is None:
            self.get_logger().warn("맵 데이터가 없습니다!")
            return frontier_points

        for i in range(1, self.map_data.shape[0] - 1):
            for j in range(1, self.map_data.shape[1] - 1):
                if self.map_data[i, j] == -1:
                    neighbors = [
                        self.map_data[i + 1, j], self.map_data[i - 1, j],
                        self.map_data[i, j + 1], self.map_data[i, j - 1]
                    ]
                    if any(n == 0 for n in neighbors):
                        world_x = self.map_origin_x + (j * self.map_resolution)
                        world_y = self.map_origin_y + (i * self.map_resolution)
                        if (0 <= i < self.map_height) and (0 <= j < self.map_width):
                            frontier_points.append((world_x, world_y))
                        else:
                            self.get_logger().warn(f"프론티어 ({world_x}, {world_y})가 맵 범위를 벗어났습니다!")

        frontier_points = [
            p for p in frontier_points
            if (round(p[0], 2), round(p[1], 2)) not in self.visited_frontiers
        ]

        self.get_logger().info(f"탐지된 프론티어 수: {len(frontier_points)}")
        return frontier_points

    def publish_frontiers(self, frontier_points):
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
        if self.map_data is None or self.map_resolution is None:
            self.get_logger().warning("맵 데이터 또는 해상도가 없습니다!")
            return

        self.get_robot_pose()
        if self.robot_pose is None:
            self.get_logger().warning("로봇 위치를 아직 수신하지 못했습니다!")
            return

        frontier_points = self.find_frontiers()
        if not frontier_points:
            self.get_logger().info("탐색할 프론티어가 없습니다!")
            return

        self.publish_frontiers(frontier_points)

        rx, ry = self.robot_pose
        frontier_points.sort(key=lambda pt: (pt[0] - rx) ** 2 + (pt[1] - ry) ** 2)
        selected = frontier_points[0]

        if self.last_goal:
            gx, gy = self.last_goal
            dist = np.hypot(selected[0] - gx, selected[1] - gy)
            if dist < self.min_repeat_distance:
                self.repeated_goal_count += 1
                self.get_logger().warn(f"⚠️ 가까운 목표 반복됨 ({self.repeated_goal_count}회)")
            else:
                self.repeated_goal_count = 0

        if self.repeated_goal_count >= self.max_repeats:
            selected = frontier_points[-1]  # 가장 먼 프론티어 선택
            self.get_logger().info(f"🔁 반복 방지: 가장 먼 프론티어 선택 → {selected}")
            self.repeated_goal_count = 0

        self.last_goal = selected
        rounded_frontier = (round(selected[0], 2), round(selected[1], 2))
        self.visited_frontiers.add(rounded_frontier)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = selected[0]
        goal_msg.pose.pose.position.y = selected[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'📍 새로운 웨이포인트 전송: {selected}')
        self.nav_client.send_goal_async(goal_msg)

    def goal_result_cb(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info("🎯 목표 도달 완료!")
        else:
            self.get_logger().warn(f"⚠️ 목표 실패 또는 도달 불가 (status={status})")
            if self.last_goal:
                rounded = (round(self.last_goal[0], 2), round(self.last_goal[1], 2))
                self.visited_frontiers.add(rounded)
        self.goal_in_progress = False
        self.send_new_waypoint()

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

