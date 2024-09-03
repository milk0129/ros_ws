import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import cv2
import numpy as np
from pyproj import Proj, transform

class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')

        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # UTM-K 좌표계 정의
        self.wgs84 = Proj(init='epsg:4326')  # WGS84 좌표계 (위도, 경도)
        self.utm_k = Proj(init='epsg:5179')  # UTM-K 좌표계

        # 주기적으로 변환 방송
        self.timer = self.create_timer(0.1, self.broadcast_transform)

        # PGM 파일 불러오기
        self.load_pgm_image('/home/ksj/nav2_ws/src/empty_map.pgm')

    def load_pgm_image(self, file_path):
        # PGM 파일 읽기
        try:
            image = cv2.imread(file_path, cv2.IMREAD_UNCHANGED)
            if image is None:
                self.get_logger().error("이미지를 불러오는 데 실패했습니다.")
                return
            
            # 헤더가 'P2'인지 확인
            header_line = None
            with open(file_path, 'r') as f:
                header_line = f.readline().strip()

            if header_line == 'P2':
                self.get_logger().info("헤더가 P2인 PGM 파일을 성공적으로 불러왔습니다.")
                # PGM 파일 처리 (예: 이미지 출력)
                cv2.imshow("PGM Image", image)
                self.create_timer(0.1, self.display_image)  # 0.1초 주기로 display_image를 호출
            else:
                self.get_logger().error("헤더가 P2가 아닙니다.")
        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")

    def display_image(self):
        cv2.waitKey(1)  # 비동기적으로 키 입력을 기다림

    def broadcast_transform(self):
        # GPS 좌표계 (위도, 경도, 높이)
        gps_lat = 37.727721394462755 # 예시 위도
        gps_lon = 127.06123221242713  # 예시 경도
        gps_alt = 0.0  # 높이

        # WGS84 좌표계에서 UTM-K 좌표계로 변환
        utm_x, utm_y = transform(self.wgs84, self.utm_k, gps_lon, gps_lat)
        utm_z = gps_alt  # 고도는 그대로 사용

        # 변환 정의
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'utm_k_frame'
        t.child_frame_id = 'gps_frame'

        # translation 초기화
        t.transform.translation.x = float(utm_x)
        t.transform.translation.y = float(utm_y)
        t.transform.translation.z = float(utm_z)

        # rotation 초기화
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # 변환 방송
        self.broadcaster.sendTransform(t)
        self.get_logger().info(f"보내는 변환: {utm_x}, {utm_y}, {utm_z}")

def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = TfBroadcaster()
    rclpy.spin(tf_broadcaster)
    tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
