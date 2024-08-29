import rclpy
from rclpy.node import Node
from lane_detection_msgs.msg import LaneDetection  # 기존 차선 인식 메시지 타입
from std_msgs.msg import String  # 기존 객체 트래킹 메시지 타입
from sensor_msgs.msg import Image  # ROS 이미지 메시지 타입

class MicroRosBridgeNode(Node):
    def __init__(self):
        super().__init__('micro_ros_bridge_node')

        # 구독자 생성: 두 개의 토픽을 구독합니다.
        self.lane_subscriber = self.create_subscription(
            LaneDetection,
            'lane_detection_info',
            self.lane_callback,
            10)
        self.object_subscriber = self.create_subscription(
            String,
            'object_direction_distance',
            self.object_callback,
            10)
        self.image_subscriber = self.create_subscription(
            Image,
            'detection_image',
            self.image_callback,
            10)

        # 퍼블리셔 생성
        self.micro_ros_lane_publisher = self.create_publisher(
            LaneDetection,
            'micro_ros_lane_detection_info',
            10)
        self.micro_ros_object_publisher = self.create_publisher(
            String,
            'micro_ros_object_direction_distance',
            10)
        self.micro_ros_image_publisher = self.create_publisher(
            Image,
            'micro_ros_detection_image',
            10)

    def lane_callback(self, msg):
        # 차선 인식 정보를 수신했을 때 바로 전송
        self.micro_ros_lane_publisher.publish(msg)
        self.get_logger().info(f'Published Lane Info to Micro ROS: {msg}')

    def object_callback(self, msg):
        # 객체 방향 및 거리 정보를 수신했을 때 바로 전송
        self.micro_ros_object_publisher.publish(msg)
        self.get_logger().info(f'Published Object Info to Micro ROS: {msg}')

    def image_callback(self, msg):
        # 객체 탐지 이미지를 수신했을 때 바로 전송
        self.micro_ros_image_publisher.publish(msg)
        self.get_logger().info(f'Published Detection Image to Micro ROS')

def main(args=None):
    rclpy.init(args=args)
    node = MicroRosBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
