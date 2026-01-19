
/**********************************************************************************************
 * 프로젝트: AutoMapper (ROS 2 / Nav2 / SLAM Toolbox)
 * 목적:    미탐사 경계(Frontier) 탐색을 자동 수행하는 노드
 *          - /map(OccupancyGrid)을 Costmap2D로 변환/동기화
 *          - 맵에서 프런티어(알려진 자유공간과 미탐사 공간의 경계)를 BFS로 군집화
 *          - 가장 우선순위 높은 프런티어 중심으로 Nav2 NavigateToPose 목표 전송
 *          - 목표 도달/종료 시 SLAM Toolbox로 맵 저장, 다음 프런티어 재탐색
 *          - RViz Marker로 프런티어 시각화
 *
 * ───────────────────────────────────────────────────────────────────────────────────────────
 * 1) 시스템 구성/의존
 *    - ROS 2 rclcpp / rclcpp_action
 *    - nav2_costmap_2d::Costmap2D: 내부 맵 버퍼(OccupancyGrid → Costmap 표현)
 *    - nav2_msgs/action/NavigateToPose: Nav2 목표 이동 액션 클라이언트
 *    - slam_toolbox 서비스:
 *        • /slam_toolbox/serialize_map (SerializePoseGraph): 포즈 그래프 직렬화(파일)
 *        • /slam_toolbox/save_map (SaveMap): OccupancyGrid를 파일로 저장
 *    - RViz: /frontiers (MarkerArray)로 프런티어 시각화
 *
 * ───────────────────────────────────────────────────────────────────────────────────────────
 * 2) 통신 인터페이스(입력/출력)
 *    [Subscribe]
 *      • /pose : geometry_msgs::PoseWithCovarianceStamped
 *           - SLAM이 추정한 로봇 현재 위치/자세. 프런티어 거리 필터 및 맵 좌표계 정합에 사용.
 *      • /map  : nav_msgs::OccupancyGrid
 *           - 최신 점유 그리드 지도. 수신 시 내부 Costmap2D로 리사이즈/복사하고 탐색 트리거.
 *
 *    [Publish]
 *      • /frontiers : visualization_msgs::MarkerArray
 *           - 프런티어 중심점(구체 마커) 시각화. RViz로 확인 가능.
 *
 *    [Action Client]
 *      • /navigate_to_pose : nav2_msgs::action::NavigateToPose
 *           - 프런티어 중심으로 이동 목표 전송(비동기). 응답/피드백/결과 콜백 관리.
 *
 *    [Service Client]
 *      • /slam_toolbox/serialize_map : slam_toolbox::srv::SerializePoseGraph
 *      • /slam_toolbox/save_map      : slam_toolbox::srv::SaveMap
 *           - 목표 완료/중단 시점 등에서 맵과 그래프를 파일로 저장.
 *
 *    [Parameter]
 *      • map_path(string): 맵/그래프 저장 파일 경로(접두어). 예) "/home/user/maps/exp01"
 *
 * ───────────────────────────────────────────────────────────────────────────────────────────
 * 3) 주요 데이터 구조/상수
 *    - Costmap2D costmap_:
 *         OccupancyGrid를 내부 코스트맵 표현으로 변환/보관. Mutex로 보호됨(락 필수).
 *    - costTranslationTable_[256]:
 *         OccupancyGrid의 data[-1,0..100] → Costmap2D 비용(FREE_SPACE/NO_INFORMATION/LETHAL 등)
 *         맵 수신 시 선형 매핑 + 특수값 재설정으로 costmap 버퍼를 채움.
 *    - Frontier { centroid, points }:
 *         BFS로 인접한 프런티어 셀들을 하나의 군집으로 묶어 중심점(월드 좌표)와 포인트 배열을 기록.
 *    - 파라미터/상수:
 *         MIN_FRONTIER_DENSITY: 프런티어 군집 최소 “길이/밀도” 기준(해상도 곱으로 근사)
 *         MIN_DISTANCE_TO_FRONTIER: 로봇과 프런티어 중심 간 최소 거리(너무 가까운 후보 제거)
 *         MIN_FREE_THRESHOLD: 후보 셀 주변 8이웃 중 자유셀 최소 개수(접근 가능성 보장)
 *
 * ───────────────────────────────────────────────────────────────────────────────────────────
 * 4) 전체 흐름(수명주기/상태)
 *    [초기화]
 *      (a) 노드 이름 "auto_mapper"로 시작.
 *      (b) /pose, /map 구독자 생성. /frontiers 퍼블리셔 생성.
 *      (c) /navigate_to_pose 액션 클라이언트 생성 및 서버 대기.
 *      (d) map_path 파라미터 선언/획득.
 *
 *    [런타임 루프]
 *      (1) /pose 콜백: 최신 PoseWithCovarianceStamped를 보관(캐시).
 *      (2) /map  콜백:
 *            • 맵 메타(크기/해상도/원점)로 costmap_ 리사이즈
 *            • costTranslationTable로 OccupancyGrid.data → costmap_.getCharMap()에 복사
 *            • explore() 호출(탐사 시작/갱신 트리거)
 *
 *      (3) explore():
 *            • isExploring_이면(이미 Nav2 목표 진행 중) 재진입 방지로 즉시 반환.
 *            • findFrontiers()로 프런티어 후보 목록을 산출.
 *            • 후보가 없으면: 경고 로그 → stop()(구독 해제/목표 취소/맵 저장/마커 정리) → 종료.
 *            • 후보가 있으면: 첫 번째 프런티어 선택(기본: 정렬 없음) → RViz 마커로 전체 표시
 *                            → 목표 메시지(centroid, frame "map") 구성
 *                            → async_send_goal()로 Nav2에 전송:
 *                                  goal_response_callback: 수락 시 isExploring_=true
 *                                  feedback_callback     : 남은 거리 등을 로그
 *                                  result_callback       : isExploring_=false, saveMap(), clearMarkers(), explore() 재호출
 *                                                         (성공/중단/취소 코드별 로그)
 *
 *      (4) saveMap():
 *            • /slam_toolbox/serialize_map: pose graph를 map_path로 직렬화
 *            • /slam_toolbox/save_map    : OccupancyGrid를 map_path로 저장
 *            (두 서비스 모두 비동기 요청)
 *
 *      (5) stop():
 *            • /pose, /map 구독 해제(더 이상 입력 처리 안 함)
 *            • 진행 중 액션 목표 모두 취소
 *            • saveMap() 호출
 *            • RViz 마커 삭제/정리
 *
 * ───────────────────────────────────────────────────────────────────────────────────────────
 * 5) 프런티어 탐색 알고리즘(핵심 로직)
 *    [핵심 개념]
 *      - “프런티어 셀”: 맵에서 “미확인(NO_INFORMATION)”이면서, 그 8이웃 중 자유(FREE_SPACE)가
 *        일정 개수(MIN_FREE_THRESHOLD) 이상인 셀(= 접근 가능한 미탐사 경계로 간주).
 *
 *    [findFrontiers() 절차]
 *      1) 현재 로봇 위치(world)를 맵 셀 좌표로 변환(worldToMap). 실패 시 맵 바깥 → 탐색 불가.
 *      2) costmap_.getMutex()로 맵을 잠금(탐색 동안 일관성 보장).
 *      3) visited_flag, frontier_flag(둘 다 size_x*size_y) 초기화.
 *      4) BFS 시작점: 로봇 위치 셀 인덱스 → 큐에 push, visited=true.
 *      5) BFS 루프:
 *          - 이웃 8방향 순회:
 *             a) 자유(FREE_SPACE) && 미방문 → 방문 표시 후 큐에 push(자유영역을 따라 파급)
 *             b) 그 외에, isAchievableFrontierCell(“미확인 && frontier 미표시 && 주변 자유셀 ≥ 임계”)
 *                → frontier_flag=true, buildNewFrontier() 호출로 연결된 프런티어 군집을 확장/수집
 *                → 군집 중심과 크기(=points.size()*해상도)로 필터(최소 거리/밀도) 통과 시 후보 목록에 추가
 *
 *    [buildNewFrontier() 절차]
 *      - 입력 이웃 셀을 시작점으로 BFS 확장(조건은 isAchievableFrontierCell와 동일)
 *      - 각 군집 포인트의 월드 좌표를 누적 → 최종적으로 군집 크기로 나눠 centroid 계산
 *
 *    [isAchievableFrontierCell(idx)]
 *      - 맵 값이 NO_INFORMATION(미확인)인가?
 *      - 이미 frontier_flag[idx] 표시되어 있지 않은가?
 *      - 8이웃 중 FREE_SPACE가 MIN_FREE_THRESHOLD개 이상인가?
 *
 *    [마커 시각화]
 *      - drawMarkers(): 각 프런티어 중심 위치에 구체(SPHERE) 마커를 추가/퍼블리시
 *      - clearMarkers(): 기존 마커 action=DELETE로 퍼블리시하여 RViz에서 제거
 *
 * ───────────────────────────────────────────────────────────────────────────────────────────
 * 6) 동시성/락/콜백
 *    - /map 콜백에서 costmap_ 버퍼(raw char map)를 갱신할 때, costmap_.getMutex()를 잡고 복사.
 *    - findFrontiers()에서도 탐색 전/중에 같은 뮤텍스를 잠가 일관성 유지.
 *    - ROS 콜백 스레드/액션 콜백은 서로 비동기이므로, isExploring_ 플래그로 목표 중복 전송 방지.
 *
 * ───────────────────────────────────────────────────────────────────────────────────────────
 * 7) 좌표계/프레임
 *    - /map(OccupancyGrid)의 origin/resolution 기반으로 world↔map 셀좌표 변환.
 *    - 목표(Goal)와 마커 모두 frame_id="map" 기준으로 발행.
 *    - /pose 역시 map 프레임 정합이 맞아야 함(SLAM Toolbox가 보통 map 프레임을 퍼블리시).
 *
 * ───────────────────────────────────────────────────────────────────────────────────────────
 * 8) 예외/종료 처리
 *    - /pose 미수신 상태에서 /map이 와도 탐색 실행 안 함(로봇 위치 불명).
 *    - worldToMap 실패(로봇이 costmap 바운드 밖) 시 에러 로그 후 빈 결과 반환.
 *    - 프런티어가 하나도 없으면 stop(): 구독 해제 → 목표 취소 → 맵 저장 → 마커 정리.
 *    - Nav2 목표 결과는 성공/중단/취소 모두 처리하고 다음 explore() 재호출(최신 맵 기준 재탐색).
 *
 * ───────────────────────────────────────────────────────────────────────────────────────────
 * 9) 계산복잡도/성능
 *    - findFrontiers(): 최악 O(N) (N=맵 셀 수) BFS 1회. 맵 수신 빈도에 따라 반복.
 *    - buildNewFrontier(): 프런티어 군집별 BFS 확장. 모든 셀은 frontier_flag/visited_flag로 1회 내외 방문.
 *    - 마커 퍼블리시는 프런티어 후보 수에 선형. 필요시 마커 배치/발행 빈도를 제한해 부하 완화 가능.
 *
 * ───────────────────────────────────────────────────────────────────────────────────────────
 * 10) 구성 체크리스트(실행 전)
 *    - Nav2 bringup 완료: /navigate_to_pose 액션 서버 활성
 *    - SLAM Toolbox 실행: /pose, /map 퍼블리시, /slam_toolbox/* 서비스 활성
 *    - 파라미터 map_path 설정(저장 경로 접근 권한 확인)
 *    - TF/프레임 일관성: /pose의 frame_id가 map 기준으로 합리적(예: map)인지 확인
 *
 * ───────────────────────────────────────────────────────────────────────────────────────────
 * 11) 확장 포인트/개선 아이디어
 *    - 프런티어 우선순위: “첫 원소” 대신 (로봇거리, 군집크기, 정보가치, heading 비용 등) 가중치 정렬.
 *    - 안전도 고려: global/local costmap의 inflation/오큘전 반영, 최소 통과폭/로봇 footprint 검사.
 *    - 중복 탐사 방지: 방문한 프런티어 키/영역 캐시, 히스테리시스 도입.
 *    - 액션 대기/재시도: wait_for_action_server() 타임아웃/백오프 전략.
 *    - 저장 정책: 주기/이벤트 기반 저장(예: 이동거리 누적, frontier 소진 시점 등)으로 I/O 최소화.
 *    - 마커 최적화: MarkerArray 크기 제한, Rate 제한, 색상/크기로 우선순위 표현.
 *
 * ───────────────────────────────────────────────────────────────────────────────────────────
 * 12) 의사코드(요약)
 *    on_init():
 *      sub /pose, /map
 *      pub  /frontiers
 *      action_client /navigate_to_pose (wait server)
 *      get param map_path
 *
 *    on_pose(msg):
 *      cache current pose
 *
 *    on_map(occupancy):
 *      lock(costmap)
 *      resize costmap by occupancy.info
 *      translate occupancy.data → costmap char map
 *      unlock(costmap)
 *      explore()
 *
 *    explore():
 *      if isExploring: return
 *      F = findFrontiers()
 *      if F empty: stop(); return
 *      drawMarkers(F)
 *      G = goal(F[0].centroid in map frame)
 *      send_goal(G,
 *         on_accept: isExploring=true
 *         on_feedback: log(distance_remaining)
 *         on_result:
 *            isExploring=false
 *            saveMap()
 *            clearMarkers()
 *            explore()   // loop to next frontier
 *      )
 *
 *    findFrontiers():
 *      if !worldToMap(robot): error; return []
 *      lock(costmap)
 *      BFS over FREE_SPACE from robot cell:
 *        - enqueue unvisited FREE neighbors
 *        - when neighbor is NO_INFORMATION with FREE around (threshold):
 *            buildNewFrontier() to cluster
 *            filter by distance & density
 *            push to list
 *      unlock(costmap)
 *      return frontier_list
 *
 *    stop():
 *      unsubscribe pose/map
 *      cancel all goals
 *      saveMap()
 *      clearMarkers()
 *
 **********************************************************************************************/




//
// Created by omar on 2/5/24. // 파일 메타 정보
//
// // 빈 줄
// Created by omar on 1/30/24. // 파일 메타 정보
//
#include <chrono> // 시간 관련 유틸
#include <functional> // std::bind 등 함수 객체
#include <memory> // 스마트 포인터
#include <string> // std::string
#include <array> // std::array
#include <filesystem> // 파일 경로 처리(맵 저장 등에서 사용 가능)
#include <slam_toolbox/srv/detail/save_map__struct.hpp> // slam_toolbox SaveMap 메시지(비권장 detail 경로)
#include <fstream> // 파일 입출력

#include "rclcpp_action/rclcpp_action.hpp" // 액션 클라이언트
#include "rclcpp/rclcpp.hpp" // ROS2 rclcpp 코어
#include "sensor_msgs/msg/range.hpp" // Range 메시지(현재 코드에서는 미사용)
#include "geometry_msgs/msg/pose_stamped.hpp" // PoseStamped 메시지
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" // PoseWithCovarianceStamped 메시지
#include "geometry_msgs/msg/point.hpp" // Point 메시지
#include "nav2_msgs/action/navigate_to_pose.hpp" // Nav2 NavigateToPose 액션
#include "nav_msgs/msg/occupancy_grid.hpp" // OccupancyGrid 맵
#include "map_msgs/msg/occupancy_grid_update.hpp" // 증분 맵 업데이트(여기선 미사용)
#include "nav2_costmap_2d/costmap_2d.hpp" // Nav2 Costmap2D
#include "nav2_costmap_2d/cost_values.hpp" // 비용 상수(FREE/LETHAL 등)
#include "nav2_util/occ_grid_values.hpp" // 점유그리드 유틸(여기선 미사용)
#include "visualization_msgs/msg/marker_array.hpp" // RViz MarkerArray
#include "visualization_msgs/msg/marker.hpp" // RViz Marker
#include "std_msgs/msg/color_rgba.hpp" // ColorRGBA(경로 이상: 일반적으로 std_msgs/msg/color_rgba.hpp) 
#include "nav2_map_server/map_mode.hpp" // 맵 저장 모드(여기선 미사용)
#include "nav2_map_server/map_saver.hpp" // 맵 세이버(여기선 직접 사용 안 함)
#include "slam_toolbox/srv/serialize_pose_graph.hpp" // Pose graph 직렬화 서비스
#include "geometry_msgs/msg/twist.hpp" // Twist(미사용)
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // TF2 변환 유틸(미사용)
#include "std_msgs/msg/bool.hpp"       // pause_flag 토픽용 Bool

#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"





using std::placeholders::_1; // bind 자리표시자
using sensor_msgs::msg::Range; // Range 별칭
using geometry_msgs::msg::PoseWithCovarianceStamped; // 별칭
using geometry_msgs::msg::PoseStamped; // 별칭
using geometry_msgs::msg::Point; // 별칭
using nav_msgs::msg::OccupancyGrid; // 별칭
using nav2_msgs::action::NavigateToPose; // 별칭
using map_msgs::msg::OccupancyGridUpdate; // 별칭(미사용)
using visualization_msgs::msg::MarkerArray; // 별칭
using visualization_msgs::msg::Marker; // 별칭
using std_msgs::msg::ColorRGBA; // 별칭
using nav2_costmap_2d::Costmap2D; // 별칭
using nav2_costmap_2d::LETHAL_OBSTACLE; // 치명 장애물 값
using nav2_costmap_2d::NO_INFORMATION; // 미확인 영역 값
using nav2_costmap_2d::FREE_SPACE; // 자유 공간 값
using std::to_string; // 문자열 변환
using std::abs; // 절대값
using std::chrono::milliseconds; // 밀리초 타입
using namespace std::chrono_literals; // 100ms 같은 리터럴
using namespace std; // std 네임스페이스 전역 사용
using namespace rclcpp; // rclcpp 네임스페이스 전역 사용
using namespace rclcpp_action; // 액션 네임스페이스
using namespace nav2_map_server; // 맵서버 네임스페이스(여기선 직접 미사용)
using namespace slam_toolbox; // 슬램툴박스 네임스페이스
using std::chrono::steady_clock; // 단조 증가 시계

using NavigateToPose = nav2_msgs::action::NavigateToPose; // 타입 별칭(중복이지만 명확화)
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>; // 액션 Goal 핸들 타입

using geometry_msgs::msg::PoseStamped;

class AutoMapper : public Node { // AutoMapper 노드 클래스 선언
public:
    AutoMapper()
            : Node("auto_mapper") { // 노드 이름 설정
        RCLCPP_INFO(get_logger(), "AutoMapper started..."); // 시작 로그

        pause_flag_.store(false);  

        
        pauseSub_ = create_subscription<std_msgs::msg::Bool>("/pause_flag", 1, std::bind(&AutoMapper::pauseCallback, this, _1));



        // poseSubscription_ = create_subscription<PoseWithCovarianceStamped>(
        //         "/pose", 10, bind(&AutoMapper::poseTopicCallback, this, _1)); // 슬램 포즈 구독

        std::string pose_topic = this->declare_parameter<std::string>("pose_topic", "/tracked_pose");
        poseSubscription_ = create_subscription<PoseStamped>(
            pose_topic, 10, bind(&AutoMapper::poseTopicCallback, this, _1));

        mapSubscription_ = create_subscription<OccupancyGrid>(
                "/map", 10, bind(&AutoMapper::updateFullMap, this, _1)); // 맵 전체 갱신 구독

        markerArrayPublisher_ = create_publisher<MarkerArray>("/frontiers", 10); // 프런티어 마커 퍼블리셔
        poseNavigator_ = rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose"); // Nav2 NavigateToPose 액션 클라이언트 생성

        poseNavigator_->wait_for_action_server(); // 액션 서버 대기(블로킹)
        RCLCPP_INFO(get_logger(), "AutoMapper poseNavigator_"); // 액션 준비 로그
        declare_parameter("map_path", rclcpp::PARAMETER_STRING); // 맵 저장 경로 파라미터 선언
        get_parameter("map_path", mapPath_); // 파라미터 값 읽기
    }

private:
    const double MIN_FRONTIER_DENSITY = 0.1; // 프런티어 최소 밀도(길이/면적 기준 근사)
    const double MIN_DISTANCE_TO_FRONTIER = 1.0; // 로봇과 프런티어 최소 거리
    const int MIN_FREE_THRESHOLD = 2; // 주변 자유셀 최소 개수
    Costmap2D costmap_; // 내부 코스트맵 버퍼
    rclcpp_action::Client<NavigateToPose>::SharedPtr poseNavigator_; // Nav2 액션 클라이언트
    Publisher<MarkerArray>::SharedPtr markerArrayPublisher_; // 마커 퍼블리셔 핸들
    MarkerArray markersMsg_; // 누적 마커 메시지
    Subscription<OccupancyGrid>::SharedPtr mapSubscription_; // 맵 구독 핸들
    bool isExploring_ = false; // 탐사 중 상태 플래그
    int markerId_; // RViz 마커 ID 증가용
    string mapPath_; // 맵 저장 경로

    std::atomic<bool> pause_flag_{false}; // ▶ 일시정지 플래그(토픽 반영)
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pauseSub_; // ▶ 토픽 구독 핸들

    

    void pauseCallback(const std_msgs::msg::Bool::SharedPtr msg) { 
        flag_ = msg->data;
     } // 콜백 래퍼
 

    // Subscription<PoseWithCovarianceStamped>::SharedPtr poseSubscription_; // 포즈 구독 핸들
    // PoseWithCovarianceStamped::UniquePtr pose_; // 최신 포즈(고유 포인터로 보관)

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSubscription_;
    std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_;
    std::unique_ptr<std_msgs::msg::Bool>  flag;

    array<unsigned char, 256> costTranslationTable_ = initTranslationTable(); // 점유→코스트 변환 테이블

    static array<unsigned char, 256> initTranslationTable() {
        array<unsigned char, 256> t{};

        // 기본값은 모두 미확인(UNKNOWN)로 채워 둔다.
        for (size_t i = 0; i < 256; ++i) t[i] = NO_INFORMATION;

        // 카토그래퍼 OccupancyGrid 규칙:
        // -1(=255 캐스팅됨) : NO_INFORMATION
        // 0..40             : FREE_SPACE
        // 41..64            : NO_INFORMATION  ← 애매한 확률대는 미지로 남겨 경계를 만들기 위함
        // 65..100           : LETHAL_OBSTACLE
        t[static_cast<unsigned char>(-1)] = NO_INFORMATION; // -1

        for (int v = 0; v <= 40; ++v)   t[v] = FREE_SPACE;         // 자유
        for (int v = 65; v <= 100; ++v) t[v] = LETHAL_OBSTACLE;    // 장애물

        return t;
    }


    struct Frontier { // 프런티어 구조체
        Point centroid; // 중심점(월드 좌표)
        vector<Point> points; // 소속 포인트들
        string getKey() const{to_string(centroid.x) + "," + to_string(centroid.y);} // ⚠ 문자열 반환 누락(버그)
    };


    void poseTopicCallback(PoseStamped::UniquePtr pose) {
        pose_ = std::move(pose);
    }


    // void poseTopicCallback(PoseWithCovarianceStamped::UniquePtr pose) { // 포즈 콜백
    //     pose_ = move(pose); // 최신 포즈 저장
    //     RCLCPP_INFO(get_logger(), "poseTopicCallback..."); // 디버그 로그
    // }

    void updateFullMap(OccupancyGrid::UniquePtr occupancyGrid) { // 맵 전체 갱신 콜백
        if (pose_ == nullptr) { return; } // 포즈 없으면 처리 중단
        RCLCPP_INFO(get_logger(), "updateFullMap..."); // 로그
        const auto occupancyGridInfo = occupancyGrid->info; // 메타정보 참조
        unsigned int size_in_cells_x = occupancyGridInfo.width; // 맵 가로 셀 수
        unsigned int size_in_cells_y = occupancyGridInfo.height; // 맵 세로 셀 수
        double resolution = occupancyGridInfo.resolution; // 해상도(m/셀)
        double origin_x = occupancyGridInfo.origin.position.x; // 원점 x
        double origin_y = occupancyGridInfo.origin.position.y; // 원점 y

        RCLCPP_INFO(get_logger(), "received full new map, resizing to: %d, %d", size_in_cells_x,
                    size_in_cells_y); // 크기 로그
        costmap_.resizeMap(size_in_cells_x,
                           size_in_cells_y,
                           resolution,
                           origin_x,
                           origin_y); // 내부 코스트맵 리사이즈

        // lock as we are accessing raw underlying map // 데이터 보호 주석
        auto *mutex = costmap_.getMutex(); // 코스트맵 뮤텍스
        lock_guard<Costmap2D::mutex_t> lock(*mutex); // 락 가드

        // fill map with data // 맵 데이터 채우기
        unsigned char *costmap_data = costmap_.getCharMap(); // 내부 버퍼 포인터
        size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY(); // 총 셀 수
        RCLCPP_INFO(get_logger(), "full map update, %lu values", costmap_size); // 로그
        for (size_t i = 0; i < costmap_size && i < occupancyGrid->data.size(); ++i) { // 모든 셀 순회
            auto cell_cost = static_cast<unsigned char>(occupancyGrid->data[i]); // 점유율값 추출
            costmap_data[i] = costTranslationTable_[cell_cost]; // 변환 테이블로 코스트로 변환
        }

        explore(); // 프런티어 탐색 및 목표 전송 로직 호출
    }

    void drawMarkers(const vector<Frontier> &frontiers) { // 프런티어 시각화
        for (const auto &frontier: frontiers) { // 각 프런티어 순회
            RCLCPP_INFO(get_logger(), "visualising %f,%f ", frontier.centroid.x, frontier.centroid.y); // 로그
            ColorRGBA green; // 색상 정의
            green.r = 0; // R=0
            green.g = 1.0; // G=1
            green.b = 0; // B=0
            green.a = 1.0; // 불투명

            vector<Marker> &markers = markersMsg_.markers; // 마커 배열 참조
            Marker m; // 마커 인스턴스

            m.header.frame_id = "map"; // 맵 좌표계
            m.header.stamp = now(); // 타임스탬프
            m.frame_locked = true; // 프레임 고정

            m.action = Marker::ADD; // 추가 액션
            m.ns = "frontiers"; // 네임스페이스
            m.id = ++markerId_; // 고유 ID 증가
            m.type = Marker::SPHERE; // 구 형태 마커
            m.pose.position = frontier.centroid; // 위치는 중심점
            m.scale.x = 0.3; // 지름 x
            m.scale.y = 0.3; // 지름 y
            m.scale.z = 0.3; // 지름 z
            m.color = green; // 녹색
            markers.push_back(m); // 배열에 추가
            markerArrayPublisher_->publish(markersMsg_); // 퍼블리시
        }
    }

    void clearMarkers() { // 기존 마커 삭제
        for (auto &m: markersMsg_.markers) { // 모든 마커 순회
            m.action = Marker::DELETE; // 삭제 액션 지정
        }
        markerArrayPublisher_->publish(markersMsg_); // 퍼블리시로 삭제 반영
    }

    void stop() { // 탐사 종료 처리
        RCLCPP_INFO(get_logger(), "Stopped..."); // 로그
        poseSubscription_.reset(); // 포즈 구독 해제
        mapSubscription_.reset(); // 맵 구독 해제
        poseNavigator_->async_cancel_all_goals(); // 모든 목표 취소
        saveMap(); // 맵 저장 요청
        clearMarkers(); // 마커 정리
    }

    void explore() { // 프런티어 탐사 및 이동 명령 루프
        if (isExploring_) { return; } // 이미 탐사 중이면 반환
        auto frontiers = findFrontiers(); // 프런티어 찾기
        if (frontiers.empty()) { // 프런티어 없으면
            RCLCPP_WARN(get_logger(), "NO BOUNDARIES FOUND!!"); // 경고
            return; // 종료
        }
        const auto frontier = frontiers[0]; // 첫 후보(가까운 순 등 정렬은 없음)
        drawMarkers(frontiers); // 마커로 표시
        auto goal = NavigateToPose::Goal(); // 액션 목표 생성
        goal.pose.pose.position = frontier.centroid; // 목표 위치 = 프런티어 중심
        goal.pose.pose.orientation.w = 1.; // 방향은 기본(0 yaw)
        goal.pose.header.frame_id = "map"; // 좌표계는 맵

        RCLCPP_INFO(get_logger(), "Sending goal %f,%f", frontier.centroid.x, frontier.centroid.y); // 로그

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions(); // 콜백 옵션
        send_goal_options.goal_response_callback = [this, &frontier](
                const GoalHandleNavigateToPose::SharedPtr &goal_handle) { // 응답 콜백
            if (goal_handle) { // 수락됨
                RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result"); // 로그
                isExploring_ = true; // 탐사 중으로 표시
            } else { // 거절됨
                RCLCPP_ERROR(get_logger(), "Goal was rejected by server"); // 오류 로그
            }
        };

        send_goal_options.feedback_callback = [this](
                const GoalHandleNavigateToPose::SharedPtr &,
                const std::shared_ptr<const NavigateToPose::Feedback> &feedback) { // 진행 피드백
            RCLCPP_INFO(get_logger(), "Distance remaining: %f", feedback->distance_remaining); // 남은 거리 로그
        };

        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult &result) { // 결과 콜백
            isExploring_ = false; // 탐사 상태 해제
            saveMap(); // 매 목적지 후 맵 저장 요청
            clearMarkers(); // 마커 정리
            explore(); // 다음 프런티어 탐색(재귀적 흐름)
            switch (result.code) { // 결과 코드 분기
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "Goal reached"); // 성공 로그
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(get_logger(), "Goal was aborted"); // 중단 로그
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(get_logger(), "Goal was canceled"); // 취소 로그
                    break;
                default:
                    RCLCPP_ERROR(get_logger(), "Unknown result code"); // 미정의 상태
                    break;
            }
        };
        poseNavigator_->async_send_goal(goal, send_goal_options); // 비동기 목표 전송
    }

    void saveMap() { // 맵 저장/직렬화 호출
        auto mapSerializer = create_client<slam_toolbox::srv::SerializePoseGraph>(
                "/slam_toolbox/serialize_map"); // pose graph 직렬화 서비스 클라이언트
        auto serializePoseGraphRequest =
                std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>(); // 요청 생성
        serializePoseGraphRequest->filename = mapPath_; // 저장 경로 지정
        auto serializePoseResult = mapSerializer->async_send_request(serializePoseGraphRequest); // 비동기 호출

        auto map_saver = create_client<slam_toolbox::srv::SaveMap>(
                "/slam_toolbox/save_map"); // SaveMap 서비스 클라이언트
        auto saveMapRequest = std::make_shared<slam_toolbox::srv::SaveMap::Request>(); // 요청 생성
        saveMapRequest->name.data = mapPath_; // 파일 이름 설정
        auto saveMapResult = map_saver->async_send_request(saveMapRequest); // 비동기 호출
    }

    vector<unsigned int> nhood8(unsigned int idx) { // 8-이웃 인덱스 취득
        unsigned int mx, my; // 셀 좌표
        vector<unsigned int> out; // 결과 이웃 목록
        costmap_.indexToCells(idx, mx, my); // 선형 인덱스→셀 좌표
        const int x = mx; // int 캐스팅
        const int y = my; // int 캐스팅
        const pair<int, int> directions[] = { // 8방향 오프셋
                pair(-1, -1),
                pair(-1, 1),
                pair(1, -1),
                pair(1, 1),
                pair(1, 0),
                pair(-1, 0),
                pair(0, 1),
                pair(0, -1)
        };
        for (const auto &d: directions) { // 각 방향 검사
            int newX = x + d.first; // 이웃 x
            int newY = y + d.second; // 이웃 y
            if (newX > -1 && newX < costmap_.getSizeInCellsX() &&
                newY > -1 && newY < costmap_.getSizeInCellsY()) { // 경계 체크
                out.push_back(costmap_.getIndex(newX, newY)); // 유효하면 인덱스 추가
            }
        }
        return out; // 결과 반환
    }

    bool isAchievableFrontierCell(unsigned int idx,
                                  const vector<bool> &frontier_flag) { // 프런티어 후보 셀 판정
        auto map = costmap_.getCharMap(); // 내부 코스트맵 버퍼
        // check that cell is unknown and not already marked as frontier // 조건 설명
        if (map[idx] != NO_INFORMATION || frontier_flag[idx]) { // 미확인 셀이 아니거나 이미 표시됨
            return false; // 프런티어 아님
        }

        //check there's enough free space for robot to move to frontier // 이동 가능한 주변 자유공간 확인
        int freeCount = 0; // 자유셀 카운트
        for (unsigned int nbr: nhood8(idx)) { // 8이웃 순회
            if (map[nbr] == FREE_SPACE) { // 자유공간이면
                if (++freeCount >= MIN_FREE_THRESHOLD) { // 임계치 도달 시
                    return true; // 유효 프런티어 후보
                }
            }
        }

        return false; // 주변 자유공간 부족
    }

    Frontier buildNewFrontier(unsigned int neighborCell, vector<bool> &frontier_flag) { // BFS로 프런티어 확장
        Frontier output; // 결과 프런티어
        output.centroid.x = 0; // 중심 x 초기화
        output.centroid.y = 0; // 중심 y 초기화

        queue<unsigned int> bfs; // 큐 준비
        bfs.push(neighborCell); // 시작 셀 삽입

        while (!bfs.empty()) { // BFS 루프
            unsigned int idx = bfs.front(); // 큐 프런트
            bfs.pop(); // 팝

            // try adding cells in 8-connected neighborhood to frontier // 8이웃 확장
            for (unsigned int nbr: nhood8(idx)) { // 이웃 순회
                // check if neighbour is a potential frontier cell // 후보 판정
                if (isAchievableFrontierCell(nbr, frontier_flag)) { // 유효 후보면
                    // mark cell as frontier // 프런티어 표시
                    frontier_flag[nbr] = true; // 방문 처리
                    unsigned int mx, my; // 셀 좌표
                    double wx, wy; // 월드 좌표
                    costmap_.indexToCells(nbr, mx, my); // 인덱스→셀
                    costmap_.mapToWorld(mx, my, wx, wy); // 셀→월드 좌표

                    Point point; // 점 생성
                    point.x = wx; // x 설정
                    point.y = wy; // y 설정
                    output.points.push_back(point); // 목록 추가

                    // update centroid of frontier // 중심 누적
                    output.centroid.x += wx; // x 합산
                    output.centroid.y += wy; // y 합산

                    bfs.push(nbr); // 큐에 추가 확장
                }
            }
        }

        // average out frontier centroid // 중심 평균화
        output.centroid.x /= output.points.size(); // 평균 x
        output.centroid.y /= output.points.size(); // 평균 y
        return output; // 프런티어 반환
    }

    vector<Frontier> findFrontiers() { // 프런티어 전체 탐색
        vector<Frontier> frontier_list; // 결과 목록
        const auto position = pose_->pose.position; // 로봇 현재 위치
        unsigned int mx, my; // 맵 셀 좌표
        if (!costmap_.worldToMap(position.x, position.y, mx, my)) { // 좌표 변환 실패 시
            RCLCPP_ERROR(get_logger(), "Robot out of costmap bounds, cannot search for frontiers"); // 에러
            return frontier_list; // 빈 목록 반환
        }

        // make sure map is consistent and locked for duration of search // 락 유지
        lock_guard<Costmap2D::mutex_t> lock(*(costmap_.getMutex())); // 맵 잠금

        auto map_ = costmap_.getCharMap(); // 코스트맵 버퍼
        auto size_x_ = costmap_.getSizeInCellsX(); // 가로 크기
        auto size_y_ = costmap_.getSizeInCellsY(); // 세로 크기

        // initialize flag arrays to keep track of visited and frontier cells // 방문/프런티어 플래그
        vector<bool> frontier_flag(size_x_ * size_y_,
                                   false); // 프런티어 여부
        vector<bool> visited_flag(size_x_ * size_y_,
                                  false); // 방문 여부

        // initialize breadth first search // BFS 준비
        queue<unsigned int> bfs; // 큐
        unsigned int pos = costmap_.getIndex(mx, my); // 시작 인덱스
        bfs.push(pos); // 푸시
        visited_flag[bfs.front()] = true; // 시작 지점 방문 표시

        while (!bfs.empty()) { // BFS 본 루프
            unsigned int idx = bfs.front(); // 현재 인덱스
            bfs.pop(); // 팝

            for (unsigned nbr: nhood8(idx)) { // 8이웃 순회
                // add to queue all free, unvisited cells, use descending search in case
                // initialized on non-free cell // 자유이면서 미방문은 큐 삽입
                if (map_[nbr] == FREE_SPACE && !visited_flag[nbr]) { // 자유 + 미방문
                    visited_flag[nbr] = true; // 방문 표시
                    bfs.push(nbr); // 큐 삽입
                    // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
                    // neighbour) // 프런티어인지 검사
                } else if (isAchievableFrontierCell(nbr, frontier_flag)) { // 프런티어 후보
                    frontier_flag[nbr] = true; // 표시
                    const Frontier frontier = buildNewFrontier(nbr, frontier_flag); // BFS 확장으로 하나 생성

                    double distance = sqrt(pow((double(frontier.centroid.x) - double(position.x)), 2.0) +
                                           pow((double(frontier.centroid.y) - double(position.y)), 2.0)); // 로봇과 거리
                    if (distance < MIN_DISTANCE_TO_FRONTIER) { continue; } // 너무 가까우면 제외
                    if (frontier.points.size() * costmap_.getResolution() >=
                        MIN_FRONTIER_DENSITY) { // 밀도 기준 통과 시
                        frontier_list.push_back(frontier); // 후보 추가
                    }
                }
            }
        }

        return frontier_list; // 전체 프런티어 반환
    }

};

int main(int argc, char *argv[]) { // 진입점
    init(argc, argv); // rclcpp 초기화
    spin(make_shared<AutoMapper>()); // 노드 실행(스핀)
    shutdown(); // 종료 처리
    return 0; // 정상 종료
}
