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

        # Nav2 액션 클라이언트 생성
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # 맵 데이터 구독
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        # TF를 통해 로봇 위치 얻기
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 프론티어 시각화를 위한 퍼블리셔
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontiers', 10)

        # 변수 초기화
        self.map_data = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.robot_pose = None
        self.visited_frontiers = set()
        self.last_goal = None
        self.repeated_goal_count = 0
        self.max_repeats = 3
        self.min_repeat_distance = 1.0
        self.goal_in_progress = False
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

    def calculate_frontier_score(self, frontier, rx, ry):
        # 거리 계산
        dist = np.hypot(frontier[0] - rx, frontier[1] - ry)

        # 프론티어 주변 미탐석 영역 크기 계산
        fx, fy = frontier
        grid_x = int((fx - self.map_origin_x) / self.map_resolution)
        grid_y = int((fy - self.map_origin_y) / self.map_resolution)
        window_size = 5
        unknown_count = 0

        for i in range(max(0, grid_y - window_size), min(self.map_height, grid_y + window_size + 1)):
            for j in range(max(0, grid_x - window_size), min(self.map_width, grid_x + window_size + 1)):
                if self.map_data[i, j] == -1:
                    unknown_count += 1

        # 맵 구역별 탐색 상태 고려
        zone = 0
        if fx > 0 and fy > 0:
            zone = 1  # 상단우측
        elif fx <= 0 and fy > 0:
            zone = 2  # 상단좌측
        elif fx <= 0 and fy <= 0:
            zone = 3  # 하단좌측
        else:
            zone = 4  # 하단우측

        # 각 구역의 미탐석 영역 비율 계산
        zone_unknown = sum(1 for f in self.find_frontiers() if (f[0] > 0) == (fx > 0) and (f[1] > 0) == (fy > 0))
        zone_score = zone_unknown * 0.3

        # 점수 계산
        score = unknown_count * 0.4 + dist * 0.3 + zone_score
        return score


    def find_frontiers(self):
        frontiers = []
        if self.map_data is None:
            self.get_logger().warn("맵 데이터가 없습니다!")
            return frontiers

        # 초기 프론티어 탐지
        for i in range(1, self.map_data.shape[0] - 1):
            for j in range(1, self.map_data.shape[1] - 1):
                if self.map_data[i, j] == -1:
                    neighbors = [self.map_data[i + 1, j], self.map_data[i - 1, j],
                                 self.map_data[i, j + 1], self.map_data[i, j - 1]]
                    if any(n == 0 for n in neighbors):
                        wx = self.map_origin_x + j * self.map_resolution
                        wy = self.map_origin_y + i * self.map_resolution
                        frontiers.append((wx, wy))

        # 방문한 프론티어 제외
        frontiers = [
            p for p in frontiers
            if (round(p[0], 2), round(p[1], 2)) not in self.visited_frontiers
        ]

        # 클러스터링
        if not frontiers:
            return frontiers

        cluster_distance = 1.0
        clustered_frontiers = []
        current_cluster = [frontiers[0]]

        for frontier in frontiers[1:]:
            last_frontier = current_cluster[-1]
            dist = np.hypot(frontier[0] - last_frontier[0], frontier[1] - last_frontier[1])
            if dist < cluster_distance:
                current_cluster.append(frontier)
            else:
                avg_x = sum(f[0] for f in current_cluster) / len(current_cluster)
                avg_y = sum(f[1] for f in current_cluster) / len(current_cluster)
                clustered_frontiers.append((avg_x, avg_y))
                current_cluster = [frontier]

        if current_cluster:
            avg_x = sum(f[0] for f in current_cluster) / len(current_cluster)
            avg_y = sum(f[1] for f in current_cluster) / len(current_cluster)
            clustered_frontiers.append((avg_x, avg_y))

        #self.get_logger().info(f"탐지된 프론티어 수: {len(frontiers)}, 클러스터링 후: {len(clustered_frontiers)}")
        return clustered_frontiers

    def publish_frontiers(self, frontiers):
        marker_array = MarkerArray()
        for idx, (x, y) in enumerate(frontiers):
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
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.frontier_pub.publish(marker_array)

    def send_new_waypoint(self):
        if self.goal_in_progress:
            return
        self.get_robot_pose()
        if self.robot_pose is None or self.map_data is None:
            return

        # 미탐석 영역 비율 계산
        total_cells = self.map_data.shape[0] * self.map_data.shape[1]
        unknown_cells = np.sum(self.map_data == -1)
        unknown_ratio = unknown_cells / total_cells
        self.get_logger().info(f"현재 미탐석 영역 비율: {unknown_ratio * 100:.2f}%")  # 추가
        if unknown_ratio < 0.1:
            self.get_logger().info(f"탐색 완료! 미탐석 영역 비율: {unknown_ratio * 100:.2f}%")
            rclpy.shutdown()
            return

        frontiers = self.find_frontiers()
        if not frontiers:
            self.get_logger().info("탐색할 프론티어가 없습니다! 탐색 완료로 간주.")
            rclpy.shutdown()
            return

        self.publish_frontiers(frontiers)
        rx, ry = self.robot_pose
        self.get_logger().info(f"현재 로봇 위치: ({rx}, {ry})")

        # 최소 거리 필터링
        min_distance_threshold = 0.5
        filtered_frontiers = []
        for frontier in frontiers:
            dist = np.hypot(frontier[0] - rx, frontier[1] - ry)
            if dist >= min_distance_threshold:
                filtered_frontiers.append(frontier)

        if not filtered_frontiers:
            self.get_logger().info("적절한 거리의 프론티어가 없습니다! 탐색 완료로 간주.")
            rclpy.shutdown()
            return

        # 점수 기반 프론티어 선택
        scores = []
        for frontier in filtered_frontiers:
            score = self.calculate_frontier_score(frontier, rx, ry)
            scores.append((frontier, score))
            #self.get_logger().info(f"프론티어 {frontier}: 점수 {score:.2f}")

        # 점수가 높은 프론티어 선택
        scores.sort(key=lambda x: x[1], reverse=True)
        selected = scores[0][0]
        selected_score = scores[0][1]
        self.get_logger().info(f"선택된 프론티어: {selected}, 점수: {selected_score:.2f}")

        # 반복 목표 방지
        if self.last_goal:
            gx, gy = self.last_goal
            dist = np.hypot(selected[0] - gx, selected[1] - gy)
            if dist < self.min_repeat_distance:
                self.repeated_goal_count += 1
                self.get_logger().warn(f"⚠️ 반복된 목표 탐지됨: {self.repeated_goal_count}회")
            else:
                self.repeated_goal_count = 0

        if self.repeated_goal_count >= self.max_repeats:
            # 가장 점수가 낮은(보통 더 먼) 프론티어 선택
            selected = scores[-1][0]
            selected_score = scores[-1][1]
            self.get_logger().info(f"🔁 반복 방지로 다른 프론티어 선택: {selected}, 점수: {selected_score:.2f}")
            self.repeated_goal_count = 0

        # 방문한 프론티어로 기록
        rounded_frontier = (round(selected[0], 2), round(selected[1], 2))
        self.visited_frontiers.add(rounded_frontier)

        self.last_goal = selected
        self.goal_in_progress = True

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = selected[0]
        goal.pose.pose.position.y = selected[1]
        goal.pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("❗ navigate_to_pose 서버를 찾을 수 없음")
            self.goal_in_progress = False
            return

        self.get_logger().info(f"📍 목표 전송: {selected}")
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ 목표가 거부됨")
            self.goal_in_progress = False
            return
        self.get_logger().info("✅ 목표가 수락됨")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info("🎯 목표 도달 완료!")
        else:
            self.get_logger().warn(f"⚠️ 목표 실패 (status={status})")
        self.goal_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
