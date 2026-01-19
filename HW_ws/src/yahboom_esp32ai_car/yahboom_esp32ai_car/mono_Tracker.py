#!/usr/bin/env python3                                 # 파이썬3 인터프리터로 실행하도록 지정
# encoding: utf-8                                      # 소스 파일 인코딩을 UTF-8로 지정
import getpass                                         # (사용 여부에 따라) 현재 사용자명 등 얻는 모듈
import threading                                       # 서보 제어를 별도 스레드로 수행하기 위한 스레딩 모듈
from yahboom_esp32ai_car.astra_common import *         # 프로젝트 공용 유틸/클래스(Tracker, color_follow, cv 등)를 임포트
from sensor_msgs.msg import CompressedImage,Image      # esp32 카메라 이미지(Compressed)와 일반 Image 메시지 타입
from std_msgs.msg import Int32, Bool,UInt16            # 서보 각도 등 단순 정수/불리언 메시지 타입
import rclpy                                           # ROS2 파이썬 클라이언트 라이브러리
from rclpy.node import Node                            # ROS2 노드 기본 클래스
from cv_bridge import CvBridge                         # ROS 이미지 메시지를 OpenCV 이미지로 변환하는 브리지
from rclpy.time import Time                            # ROS2 Time 타입(Stamp 변환에 사용)
import datetime                                        # 시간 차이 계산 및 포매팅용 표준 라이브러리
# >>> 여기에 필요: import time                        # 코드 내에서 time.time(), time.perf_counter() 사용하므로 추가 필요


class mono_Tracker(Node):                              # 단안(모노) 트래커 노드 정의
    def __init__(self,name):                           # 생성자(노드명 name을 받아 초기화)
        super().__init__(name)                         # 부모 클래스(Node) 초기화
        self.pub_Servo1 = self.create_publisher(Int32,"servo_s1" , 10)  # 서보1 각도 퍼블리셔 생성
        self.pub_Servo2 = self.create_publisher(Int32,"servo_s2" , 10)  # 서보2 각도 퍼블리셔 생성
        self.declare_param()                           # PID 관련 파라미터 선언 및 초기화
        self.target_servox = 0                         # 서보 X축 목표 각도 초기값
        self.target_servoy = 10                        # 서보 Y축 목표 각도 초기값
        self.point_pose = (0, 0, 0)                    # 바운딩박스 중심/크기 저장용 (x,y,size)
        self.circle = (0, 0, 0)                        # 컬러 추적 시 원(중심x,y,반지름) 정보
        self.hsv_range = ()                            # 컬러 추적용 HSV 범위(튜플)
        self.dyn_update = True                         # 동적 파라미터 업데이트 플래그
        self.select_flags = False                      # ROI 선택 중인지 여부 플래그
        self.gTracker_state = False                    # 트래커 초기화 필요 상태 플래그
        self.windows_name = 'frame'                    # OpenCV 윈도우 이름
        self.cols, self.rows = 0, 0                    # ROI 좌상/우하 좌표 저장용
        self.Mouse_XY = (0, 0)                         # 마우스 클릭 좌표 저장
        self.index = 2                                 # 초기 트래커 타입 인덱스(KCF를 가리킴)
        self.end = 0                                   # FPS 계산용 이전 프레임 시간
        self.color = color_follow()                    # 컬러 추적 객체 생성(astra_common 제공)

        self.tracker_types = ['BOOSTING', 'MIL', 'KCF']# 사용 가능한 트래커 타입 리스트
        self.tracker_type = ['KCF']                    # 현재 선택된 트래커 타입(여기선 KCF)
        self.VideoSwitch = True                        # 비디오 처리 스위치(사용자 로직용)
        self.img_flip = False                          # 이미지 좌우 반전 여부

        self.last_stamp = None                         # 최신 이미지 타임스탬프 저장
        self.new_seconds = 0                           # FPS 계산용 이전 시각(초 단위)
        self.fps_seconds = 1                           # 지연 보정값(기본 1)

        ser1_angle = Int32()                           # 서보1 초기 메시지 생성
        ser1_angle.data = int(self.target_servox)      # 서보1 각도 데이터 설정
        ser2_angle = Int32()                           # 서보2 초기 메시지 생성
        ser2_angle.data = int(self.target_servoy)      # 서보2 각도 데이터 설정

        #确保角度正常处于中间 (서보를 중앙 근처로 맞추기 위해 여러 번 퍼블리시)
        for i in range(10):                            # 10회 반복 퍼블리시
            self.pub_Servo1.publish(ser1_angle)        # 서보1 각도 퍼블리시
            self.pub_Servo2.publish(ser2_angle)        # 서보2 각도 퍼블리시
            time.sleep(0.1)                            # 0.1초 대기(물리 서보 반영 시간 확보)
        
        self.hsv_text ="/home/yahboom/yahboomcar_ws/src/yahboom_esp32ai_car/yahboom_esp32ai_car/colorHSV.text"  # HSV 파라미터 파일 경로
        self.mono_PID = (12, 0, 0.9)                   # 기본 PID 계수(Kp, Ki, Kd)
        self.scale = 1000                              # PID 스케일(정규화용)
        self.PID_init()                                # PID 컨트롤러 초기화

        print("OpenCV Version: ",cv.__version__)       # OpenCV 버전 출력(astra_common에서 cv 제공 가정)
        self.gTracker = Tracker(tracker_type=self.tracker_type)  # 트래커 객체 생성
        self.tracker_type = self.tracker_types[self.index]       # 인덱스로 현재 트래커 타입 문자열 설정(KCF)
        self.Track_state = 'init'                      # 트래킹 상태 초기화(init/identify/tracking)

        #USB                                             
        #self.capture = cv.VideoCapture(0)              # (주석) USB 카메라 캡처 초기화
        #self.timer = self.create_timer(0.001, self.on_timer)  # (주석) 주기 타이머 등록

        #ESP32_wifi
        self.bridge = CvBridge()                       # CvBridge 생성(CompressedImage → OpenCV 이미지)
        self.sub_img = self.create_subscription(       # ESP32 카메라 이미지 구독
            CompressedImage, '/espRos/esp32camera', self.handleTopic, 1) # 콜백 등록/큐크기 1

    def declare_param(self):                           # 파라미터 선언 및 초기 획득 함수
        #PID
        self.declare_parameter("Kp",12)                # Kp 기본값 12 선언
        self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value  # Kp 값 읽기
        self.declare_parameter("Ki",0)                 # Ki 기본값 0 선언
        self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value  # Ki 값 읽기
        self.declare_parameter("Kd",0.9)               # Kd 기본값 0.9 선언
        self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value  # Kd 값 읽기
    
    def get_param(self):                               # 동작 중 최신 파라미터 재획득 함수
        self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value  # Kd 갱신
        self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value  # Ki 갱신
        self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value  # Kp 갱신
        self.mono_PID = (self.Kp,self.Ki,self.Kd)      # 내부 PID 튜플 갱신
        
    def cancel(self):                                  # 취소/정리 동작 수행 함수
        self.Reset()                                   # 내부 상태 리셋
        if self.VideoSwitch==False: self.__sub_img.unregister()  # (주의) __sub_img는 정의 안 됨(오타 가능)
        cv.destroyAllWindows()                         # OpenCV 윈도우 모두 닫기

    # USB
    # def on_timer(self):                              # (주석) USB 카메라 처리용 주기 콜백
    #     self.get_param()                             # PID 파라미터 갱신
    #     ret, frame = self.capture.read()             # 프레임 읽기
    #     action = cv.waitKey(10) & 0xFF               # 키 입력 처리
    #     frame, binary =self.process(frame, action)   # 프레임 처리(트래킹 등)
    #     start = time.time()                          # 시간 기록
    #     fps = 1 / (start - self.end)                 # FPS 계산
    #     text = "FPS : " + str(int(fps))              # FPS 텍스트
    #     self.end = start                             # 기준 시간 갱신
    #     cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)  # FPS 표시
    #     if len(binary) != 0: cv.imshow('frame', ManyImgs(1, ([frame, binary])))  # 결과 렌더링
    #     else:cv.imshow('frame', frame)               # 단일 프레임 표시
    #     if action == ord('q') or action == 113:      # q 누르면 종료
    #         self.capture.release()                   # 카메라 해제
    #         cv.destroyAllWindows()                   # 윈도우 닫기

    #ESP32_wifi
    def handleTopic(self, msg):                        # ESP32 카메라 이미지 수신 콜백
        self.last_stamp = msg.header.stamp             # 메시지 헤더 타임스탬프 저장
        if self.last_stamp:                            # 타임스탬프가 있으면
            total_secs = Time(nanoseconds=self.last_stamp.nanosec, seconds=self.last_stamp.sec).nanoseconds  # ns 단위 환산(사용 주의)
            delta = datetime.timedelta(seconds=total_secs * 1e-9)  # ns → s 변환하여 timedelta 생성
            seconds = delta.total_seconds()*100        # 100배 스케일의 초 값(커스텀 보정용으로 보임)
            if self.new_seconds != 0:                  # 이전 값이 있으면
                self.fps_seconds = seconds - self.new_seconds  # 지연 보정값 업데이트
            self.new_seconds = seconds                 # 현재 값을 저장(다음 회차 비교용)

        self.get_param()                               # 최신 PID 파라미터 갱신
        start = time.time()                            # 시작 시간 기록
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)  # CompressedImage → OpenCV 이미지로 변환
        frame = cv.resize(frame, (640, 480))          # 프레임 크기 표준화

        action = cv.waitKey(10) & 0xFF                # 키 입력 처리(10ms 대기)
        frame, binary =self.process(frame, action)    # 프레임 처리(트래킹/표시/ROI 등)
        
        end = time.time()                              # 종료 시간 기록
        fps = 1 / ((end - start)+self.fps_seconds)     # 처리시간+보정값으로 FPS 계산
        text = "FPS : " + str(int(fps))                # FPS 텍스트 생성
        cv.putText(frame, text, (10,20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)  # 프레임에 FPS 표시
        cv.imshow('frame', frame)                      # 프레임 화면 출력

        if action == ord('q') or action == 113:        # q 키를 누르면
            cv.destroyAllWindows()                     # 윈도우 닫기

    def Reset(self):                                   # 추적 상태/변수 초기화
        self.hsv_range = ()                            # HSV 범위 초기화
        self.circle = (0, 0, 0)                        # 원(컬러 추적 결과) 초기화
        self.Mouse_XY = (0, 0)                         # 마우스 좌표 초기화
        self.Track_state = 'init'                      # 상태를 init으로
        self.target_servox = 0                         # 서보X 목표 초기화
        self.target_servoy = 10                        # 서보Y 목표 초기화
    
    def execute(self, point_x, point_y):               # 추적 대상 좌표를 받아 서보를 구동하는 함수
        # rospy.loginfo("point_x: {}, point_y: {}".format(point_x, point_y))  # (주석) ROS1 스타일 로그
        [x_Pid, y_Pid] = self.PID_controller.update([point_x - 320, point_y - 240])  # 화면 중심(320,240) 대비 오차로 PID 계산
        if self.img_flip == True:                      # 이미지 반전 시 보정(현재 분기 동일)
            self.target_servox -= x_Pid                # X축 서보 목표 갱신(좌우 반영)
            self.target_servoy += y_Pid                # Y축 서보 목표 갱신(상하 반영)
        else:
            self.target_servox -= x_Pid                # 반전 아님: 동일 계산
            self.target_servoy += y_Pid
        if self.target_servox >= 45:                   # X축 상한 제한
            self.target_servox = 45
        elif self.target_servox <= -45:                # X축 하한 제한
            self.target_servox = -45
        if self.target_servoy >= 40:                   # Y축 상한 제한
            self.target_servoy = 40
        elif self.target_servoy <= -90:                # Y축 하한 제한
            self.target_servoy = -90
        print("servo1",self.target_servox)             # 현재 서보X 목표 각도 출력(디버그)
        servo1_angle = Int32()                         # 서보1 메시지 생성
        servo1_angle.data = int(self.target_servox)    # 서보1 각도 설정
        servo2_angle = Int32()                         # 서보2 메시지 생성
        servo2_angle.data = int(self.target_servoy)    # 서보2 각도 설정
        self.pub_Servo1.publish(servo1_angle)          # 서보1 퍼블리시
        self.pub_Servo2.publish(servo2_angle)          # 서보2 퍼블리시

    def dynamic_reconfigure_callback(self, config, level):  # 동적 파라미터(슬라이더 등) 갱신 콜백(ROS1 스타일 흔적)
        self.scale = config['scale']                  # 스케일 업데이트
        self.mono_PID = (config['Kp'], config['Ki'], config['Kd'])  # PID 계수 업데이트
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),  # HSV 최소
                          (config['Hmax'], config['Smax'], config['Vmax']))  # HSV 최대
        self.PID_init()                               # PID 재초기화
        return config                                 # 변경된 설정 반환

    def PID_init(self):                               # PID 컨트롤러 초기화 함수
        self.PID_controller = simplePID(              # 간단 PID 객체 생성
            [0, 0],                                   # 타깃(오차 0,0)
            [self.mono_PID[0] / float(self.scale), self.mono_PID[0] / float(self.scale)],  # Kp (x,y)
            [self.mono_PID[1] / float(self.scale), self.mono_PID[1] / float(self.scale)],  # Ki (x,y)
            [self.mono_PID[2] / float(self.scale), self.mono_PID[2] / float(self.scale)])  # Kd (x,y)

    def onMouse(self, event, x, y, flags, param):     # 마우스 콜백(ROI 선택)
        if event == 1:                                # 버튼 다운 시
            self.Track_state = 'init'                 # 상태를 init으로
            self.select_flags = True                  # 선택 모드 시작
            self.Mouse_XY = (x,y)                     # 시작 좌표 기록
        if event == 4:                                # 버튼 업 시
            self.select_flags = False                 # 선택 모드 해제
            self.Track_state = 'identify'             # 상태를 identify로 전환
        if self.select_flags == True:                 # 드래그 중이라면
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)  # 좌상단 좌표 계산
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)  # 우하단 좌표 계산
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])  # ROI 튜플 저장

    def process(self, rgb_img, action):               # 한 프레임 처리(트래커/컬러/ROI/표시/실행)
        # param action: [113 or 'q':退出]，[114 or 'r':重置]，[105 or 'i'：识别]，[32：开始追踪]
        rgb_img = cv.resize(rgb_img, (640, 480))      # 크기 표준화
        binary = []                                   # 이진화 결과(컬러 모드에서 사용)
        if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)  # 필요한 경우 좌우 반전
        if action == 32: self.Track_state = 'tracking'            # 스페이스: 추적 시작
        elif action == ord('i') or action == 105: self.Track_state = "identify"  # i: 식별(컬러/ROI)
        elif action == ord('r') or action == 114: self.Reset()    # r: 상태 리셋
        elif action == ord('q') or action == 113: self.cancel()   # q: 취소/종료 처리
        if self.Track_state == 'init':               # 초기 상태
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE) # 윈도우 생성
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)  # 마우스 콜백 설정
            if self.select_flags == True:            # ROI 선택 중이면 가이드 렌더링
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)       # 대각선 라인
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)  # 사각형 박스
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:  # 유효 ROI 확인
                    if self.tracker_type == "color": rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)  # ROI 기반 HSV 범위 추출
                    self.gTracker_state = True       # 트래커 초기화 필요 플래그 ON
                    self.dyn_update = True           # 동적 파라미터 업데이트 필요
                else: self.Track_state = 'init'      # 잘못된 ROI면 상태 유지
        if self.Track_state != 'init':               # init 외 상태(identify/tracking)
            if self.tracker_type == "color" and len(self.hsv_range) != 0:  # 컬러 트래커 + HSV 유효
                rgb_img, binary, self.circle = self.color.object_follow(rgb_img, self.hsv_range)  # 컬러 기반 객체 추적
                if self.dyn_update == True:          # 동적 파라미터 갱신 필요 시
                    params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                              'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                              'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}  # 현재 HSV 범위 정리
                    self.dyn_client.update_configuration(params)  # (ROS1식) dyn_reconf 업데이트(환경에 따라 미동작 가능)
                    self.dyn_update = False         # 업데이트 완료
            if self.tracker_type != "color":        # 일반 트래커(KCF 등)
                if self.gTracker_state == True:     # 트래커 초기화 필요 시
                    Roi = (self.Roi_init[0], self.Roi_init[1], self.Roi_init[2] - self.Roi_init[0], self.Roi_init[3] - self.Roi_init[1])  # (x,y,w,h)
                    self.gTracker = Tracker(tracker_type=self.tracker_type)  # 선택 타입으로 트래커 재생성
                    self.gTracker.initWorking(rgb_img, Roi)  # 현재 프레임과 ROI로 트래커 초기화
                    self.gTracker_state = False     # 초기화 완료
                rgb_img, (targBegin_x, targBegin_y), (targEnd_x, targEnd_y) = self.gTracker.track(rgb_img)  # 트래킹 업데이트
                center_x = targEnd_x / 2 + targBegin_x / 2  # 중심 x 계산
                center_y = targEnd_y / 2 + targBegin_y / 2  # 중심 y 계산
                width = targEnd_x - targBegin_x            # 폭 계산
                high = targEnd_y - targBegin_y             # 높이 계산
                self.point_pose = (center_x, center_y, min(width, high))  # 중심과 최소 변 길이 저장
        if self.Track_state == 'tracking':           # 추적 상태이면
            if self.circle[2] != 0: threading.Thread(target=self.execute, args=(self.circle[0], self.circle[1])).start()  # 컬러 추적 결과가 유효하면 실행
            if self.point_pose[0] != 0 and self.point_pose[1] != 0: threading.Thread(target=self.execute, args=(self.point_pose[0], self.point_pose[1])).start()  # KCF 결과도 실행
        if self.tracker_type != "color": cv.putText(rgb_img, " Tracker", (260, 20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)  # 트래커 라벨 표시
        return rgb_img, binary                        # 렌더링 프레임과 바이너리 결과 반환

class simplePID:                                      # 간단 이산 PID 컨트롤러 클래스
    '''very simple discrete PID controller'''

    def __init__(self, target, P, I, D):              # 생성자: 목표값/게인 설정
        '''Create a discrete PID controller
        each of the parameters may be a vector if they have the same length
        Args:
        target (double) -- the target value(s)
        P, I, D (double)-- the PID parameter
        '''
        # check if parameter shapes are compatabile.
        if (not (np.size(P) == np.size(I) == np.size(D)) or ((np.size(target) == 1) and np.size(P) != 1) or (
                np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')  # 입력 차원 호환성 검사
       # rospy.loginfo('P:{}, I:{}, D:{}'.format(P, I, D))               # (주석) ROS1 로그
        self.Kp = np.array(P)                           # Kp 벡터 저장
        self.Ki = np.array(I)                           # Ki 벡터 저장
        self.Kd = np.array(D)                           # Kd 벡터 저장
        self.last_error = 0                             # 이전 오차 저장
        self.integrator = 0                             # 적분값 누적 변수
        self.timeOfLastCall = None                      # 마지막 호출 시간
        self.setPoint = np.array(target)                # 목표값 벡터
        self.integrator_max = float('inf')              # 적분 상한(무한대)

    def update(self, current_value):                    # PID 업데이트(현재값 → 제어량)
        '''Updates the PID controller.
        Args:
            current_value (double): vector/number of same legth as the target given in the constructor
        Returns:
            controll signal (double): vector of same length as the target
        '''
        current_value = np.array(current_value)         # 현재값을 배열화
        if (np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')  # 차원 불일치 검사
        if (self.timeOfLastCall is None):               # 첫 호출이면
            # the PID was called for the first time. we don't know the deltaT yet
            # no controll signal is applied
            self.timeOfLastCall = time.perf_counter()   # 기준 시간 저장
            return np.zeros(np.size(current_value))     # 제어량 0 반환
        error = self.setPoint - current_value           # 오차 계산
        P = error                                       # 비례항
        currentTime = time.perf_counter()               # 현재 시간
        deltaT = (currentTime - self.timeOfLastCall)    # 시간 차
        # integral of the error is current error * time since last update
        self.integrator = self.integrator + (error * deltaT)  # 적분항 누적
        I = self.integrator                              # 적분항
        # derivative is difference in error / time since last update
        D = (error - self.last_error) / deltaT          # 미분항(오차 변화율)
        self.last_error = error                          # 이전 오차 갱신
        self.timeOfLastCall = currentTime                # 호출 시간 갱신
        # return controll signal
        return self.Kp * P + self.Ki * I + self.Kd * D  # PID 합성 출력 반환


def main():                                             # 노드 실행 엔트리 포인트
    rclpy.init()                                        # rclpy 초기화
    mono_tracker = mono_Tracker("monoIdentify")         # mono_Tracker 노드 인스턴스 생성(노드명 monoIdentify)
    try:                                                # 예외 처리 블록
        rclpy.spin(mono_tracker)                        # 콜백 루프 시작(이미지 구독/처리 수행)
    except KeyboardInterrupt:                           # Ctrl+C 등으로 종료 시
        pass                                            # 추가 처리 없음
    finally:                                            # 항상 실행되는 정리 블록
        mono_tracker.destroy_node()                     # 노드 파괴/정리
        rclpy.shutdown()                                # rclpy 종료
