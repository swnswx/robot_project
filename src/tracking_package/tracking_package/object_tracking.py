import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cvzone
import math
import numpy as np
from ultralytics import YOLO
from tracking_package.sort.sort import Sort
import pyrealsense2 as rs
from skimage import io

class TrashDetectionNode(Node):
    def __init__(self):
        super().__init__('trash_detection_node')
        
        # SORT 추적기 초기화
        self.tracker = Sort(max_age=20, min_hits=3, iou_threshold=0.3)
        
        # YOLO 모델 로드
        self.model = YOLO("/home/sw/robot_project/src/tracking_package/tracking_package/best.pt")
        
        # 클래스 이름 정의
        self.classNames = ["trash"]
        
        # 리얼센스 파이프라인 초기화
        self.pipeline = rs.pipeline()
        config = rs.config()
        # 깊이 스트림 활성화
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # 컬러 스트림 활성화
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        
        # ROS 이미지와 OpenCV 이미지 간 변환을 위한 CvBridge 생성
        self.bridge = CvBridge()
        
        # 처리된 이미지를 퍼블리시할 퍼블리셔 생성
        self.image_publisher_ = self.create_publisher(Image, 'detection_image', 10)
        
        # 방향 및 거리 정보를 퍼블리시할 퍼블리셔 생성
        self.direction_distance_publisher_ = self.create_publisher(String, 'object_direction_distance', 10)
        
        # 주기적으로 탐지 함수를 호출하는 타이머 생성 (0.1초마다)
        self.timer = self.create_timer(0.1, self.detect)
        
    def detect(self):
        # 리얼센스 카메라에서 프레임 가져오기
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # 깊이 또는 컬러 프레임이 없을 경우 반환
        if not depth_frame or not color_frame:
            return
        
        # 컬러 프레임을 numpy 배열로 변환
        img = np.asanyarray(color_frame.get_data())
        img_height, img_width, _ = img.shape
        # 카메라의 중심 x좌표 계산
        center_x = img_width // 2  
        
        # YOLO 모델을 사용해 물체 탐지 수행
        results = self.model(img, stream=True)
        detections = np.empty((0, 5))
        
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                
                # 클래스 이름 가져오기
                cls = int(box.cls[0])
                
                # 신뢰도 점수 계산
                conf = math.ceil(box.conf[0] * 100) / 100
                if conf > 0.5:
                    # 이미지에 클래스 이름 표시
                    cvzone.putTextRect(img, f'{self.classNames[cls]}', (x2, y2), scale=1, thickness=1, colorR=(0, 0, 255))
                    currentArray = np.array([x1, y1, x2, y2, conf])
                    detections = np.vstack((detections, currentArray))
        
        # SORT 추적기를 사용해 물체 추적
        resultTracker = self.tracker.update(detections)
        
        for res in resultTracker:
            x1, y1, x2, y2, id = res
            x1, y1, x2, y2, id = int(x1), int(y1), int(x2), int(y2), int(id)
            w, h = x2 - x1, y2 - y1
            
            # 물체의 중심 x, y 좌표 계산
            object_center_x = (x1 + x2) // 2
            object_center_y = (y1 + y2) // 2
            
            # 물체의 방향 결정 (카메라 중심 기준)
            if object_center_x < center_x - 30:  # 중심 정렬을 위한 임계값 30 픽셀
                direction = 'left'
            elif object_center_x > center_x + 30:
                direction = 'right'
            else:
                direction = 'straight'
            
            # 물체와의 거리 계산 (미터 단위)
            distance = depth_frame.get_distance(object_center_x, object_center_y)
            
            # 방향 및 거리 정보를 퍼블리시
            direction_distance_msg = String()
            direction_distance_msg.data = f'ID: {id}, Direction: {direction}, Distance: {distance:.2f} meters'
            self.direction_distance_publisher_.publish(direction_distance_msg)
            
            # 로그 출력
            self.get_logger().info(f'Published: {direction_distance_msg.data}')
            
            # 이미지에 ID, 방향, 거리 정보 표시
            cvzone.putTextRect(img, f'ID: {id} {direction} {distance:.2f}m', (x1, y1), scale=1, thickness=1, colorR=(255, 0, 255))
        
        # OpenCV 창에 이미지 표시
        cv2.imshow("Detection", img)

        # OpenCV 이미지를 ROS 이미지 메시지로 변환
        image_message = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        
        # 이미지를 퍼블리시
        self.image_publisher_.publish(image_message)

        # 로그 출력
        self.get_logger().info('Published: Detection image')
        
        # OpenCV 창 업데이트
        cv2.waitKey(1)
        
    def destroy_node(self):
        # 리얼센스 파이프라인 종료
        self.pipeline.stop()
        super().destroy_node()
        # OpenCV 창 닫기
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = TrashDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
