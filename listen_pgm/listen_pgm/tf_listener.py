import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from pyproj import Proj, transform

class TfListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.check_transform)  # 0.1초 주기로 실행

    def check_transform(self):
        try:
            # UTM 프레임에서 GPS 프레임으로의 변환 조회
            transform = self.tf_buffer.lookup_transform('gps_frame', 'utm_k_frame', rclpy.time.Time())

            # 변환된 좌표 및 회전 값 출력
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.get_logger().info(f"변환: x={translation.x}, y={translation.y}, z={translation.z}")
            self.get_logger().info(f"회전: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")

        except tf2_ros.LookupException:
            self.get_logger().warn("변환을 찾을 수 없습니다.")
        except tf2_ros.ConnectivityException:
            self.get_logger().warn("변환 연결 문제 발생.")
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn("변환 시간이 너무 오래되었습니다.")


def main(args=None):
    rclpy.init(args=args)
    tf_listener = TfListener()
    rclpy.spin(tf_listener)
    tf_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
