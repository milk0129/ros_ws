import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import cv2
import numpy as np
from pyproj import Proj, transform
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import yaml

class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')


        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # UTM-K 좌표계 정의
        self.wgs84 = Proj(init='epsg:4326')  # WGS84 좌표계 (위도, 경도)
        self.utm_k = Proj(proj="utm", zone=52, ellps="WGS84", south=False)  # UTM-K 좌표계, 북반구 지정

        # /gps/fix 토픽 발행 설정
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 발행

        # OccupancyGrid 메시지 발행 설정
        self.map_publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)

        # PGM 및 YAML 파일 경로 설정
        self.pgm_file_path = ('/home/ksj/nav2_ws/src/local_pgm/map/empty_map.pgm')
        self.yaml_file_path = ('/home/ksj/nav2_ws/src/local_pgm/map/empty_map.yaml')

        # PGM 및 YAML 파일 불러오기
        self.load_map(self.pgm_file_path, self.yaml_file_path)

    def load_map(self, pgm_file_path, yaml_file_path):
        # PGM 파일 읽기
        try:

            with open(yaml_file_path, 'r') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)

            image = cv2.imread(pgm_file_path, cv2.IMREAD_UNCHANGED)
            if image is None:
                self.get_logger().error("이미지를 불러오는 데 실패했습니다.")
                return
            
            # YAML 데이터에서 맵 정보 추출
            self.resolution = yaml_data.get('resolution', 0.05)
            self.origin = yaml_data.get('origin', [0.0, 0.0, 0.0])
            negate = yaml_data.get('negate', 0)
            occupied_thresh = yaml_data.get('occupied_thresh', 50)
            free_thresh = yaml_data.get('free_thresh', 50)

            # 이미지 데이터 처리
            if negate == 1:
                image = cv2.bitwise_not(image)

            # 픽셀 값을 OccupancyGrid 데이터로 매핑
            occupancy_data = []
            for pixel in image.flatten():
                if pixel >= occupied_thresh:
                    occupancy_data.append(100)  # 장애물 있음
                elif pixel <= free_thresh:
                    occupancy_data.append(0)    # 장애물 없음
                else:
                    occupancy_data.append(-1)   # 미확인 영역
            
            """# 헤더가 'P2'인지 확인
            #header_line = None
            with open(pgm_file_path, 'r') as f:
                header_line = f.readline().strip()

            if header_line == 'P2':
                self.get_logger().info("헤더가 P2인 PGM 파일을 성공적으로 불러왔습니다.")
                # PGM 파일 처리 (예: 이미지 출력)
                #cv2.imshow("PGM Image", image)
                #self.create_timer(0.1, self.display_image)  # 0.1초 주기로 display_image를 호출
                
            else:
                self.get_logger().error("헤더가 P2가 아닙니다.")"""

            # OccupancyGrid 메시지 발행
            self.publish_occupancy_grid(image, occupancy_data)

        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")

    def publish_occupancy_grid(self, image, occupancy_data):
        # OccupancyGrid 메시지 생성
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"

        grid_msg.info.height, grid_msg.info.width = image.shape
        grid_msg.info.resolution = self.resolution
        grid_msg.info.origin.position.x = self.origin[0]
        grid_msg.info.origin.position.y = self.origin[1]
        grid_msg.info.origin.position.z = self.origin[2]
        grid_msg.info.origin.orientation.w = 1.0

        # 이미지를 OccupancyGrid 데이터로 변환
        # OpenCV로 읽은 PGM 파일의 픽셀 값 (0~255)을 -1, 0, 100으로 매핑
        grid_msg.data = occupancy_data

        """for pixel in image.flatten():
            if pixel == 255:
                occupancy_data.append(0)  # 장애물 없음
            elif pixel == 0:
                occupancy_data.append(100)  # 장애물 있음
            else:
                occupancy_data.append(-1)  # 미확인 영역

        grid_msg.data = occupancy_data"""

        # OccupancyGrid 메시지 퍼블리시
        self.map_publisher_.publish(grid_msg)
        self.get_logger().info("OccupancyGrid 메시지를 성공적으로 발행했습니다.")

    """def display_image(self):
        cv2.waitKey(1)  # 비동기적으로 키 입력을 기다림"""

    def timer_callback(self):
            # 예시로 사용할 위도, 경도, 고도 값
            latitude = 37.72770336620859 
            longitude = 127.06124921282776
            altitude = 0.0

            # NavSatFix 메시지 생성 및 발행
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps_link'  # GPS 장치가 장착된 프레임 설정
            msg.latitude = latitude
            msg.longitude = longitude
            msg.altitude = altitude

            self.publisher_.publish(msg)
            self.get_logger().info(f"발행한 GPS 데이터: 위도: {msg.latitude}, 경도: {msg.longitude}, 고도: {msg.altitude}")

            # 발행한 데이터를 기반으로 변환 방송
            self.broadcast_transform(msg.latitude, msg.longitude, msg.altitude)

    def broadcast_transform(self, gps_lat, gps_lon, gps_alt):
        # GPS 좌표계 (위도, 경도, 높이)

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
        self.get_logger().info(f"위도, 경도, 고도: {gps_lat}, {gps_lon}, {gps_alt}")
        self.get_logger().info(f"보내는 변환: {utm_x}, {utm_y}, {utm_z}")
        
def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = TfBroadcaster()
    rclpy.spin(tf_broadcaster)
    tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()