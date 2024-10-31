import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import cv2
import yaml
import numpy as np
from firebase_admin import db, credentials
import firebase_admin 
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point

def parse_coordinates(data):
    if data is None:
        return [], []

    data = data.strip("[]").split(", ")
    
    e_coordinates = []
    c_coordinates = []
    
    for i, item in enumerate(data):
        parts = item.split(" ")
        lat = float(parts[1])
        lon = float(parts[3])

        if i == 0:  # 첫 번째 좌표
           c_coordinates.append((lat, lon))
           print(e_coordinates)
        elif i == 1:  # 두 번째 좌표
           e_coordinates.append((lat, lon))
           print(c_coordinates)

    return e_coordinates, c_coordinates

class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.subscription = self.create_subscription(
        NavSatFix,
        '/gps/fix',
        self.gps_callback,
        10)

        self.latitude = 37.631013 #37.632778 #37.631125 # 보드에서 gps센서가 보내주는 위경도 값을 latitude, longitude에 저장
        self.longitude = 127.054042 #127.055389 #127.054211

        # Firebase 초기화
        if not firebase_admin._apps:
            cred = credentials.Certificate("/home/ksj/Downloads/dbtest-c6461-firebase-adminsdk-rfe4b-24bf021e56.json")
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://dbtest-admin.firebaseio.com'
            })
        self.ref = db.reference('admin')

        # TF 브로드캐스터 초기화
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # map 메시지 발행 설정
        self.map_publisher_ =  self.create_publisher(OccupancyGrid, 'map', 10)
        # self.map_timer = self.create_timer(1.0,self.publish_map_callback)

        # MarkerArray 메시지 발행 설정
        self.e_marker_publisher_ = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.e_marker_timer = self.create_timer(1.1, self.publish_e_marker)

        self.c_marker_publisher_ = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.c_marker_timer = self.create_timer(1.3, self.publish_c_marker)

        self.sen_marker_publisher_ = self.create_publisher(MarkerArray, '/visualization_marker_array',10)
        self.sen_marker_timer = self.create_timer(1.5,self.sensor_marker)

        self.e_marker_array = MarkerArray() # db에서 받아온 위치 좌표 마커위함(녹색)
        self.c_marker_array = MarkerArray() 
        self.sen_marker_array = MarkerArray() #현 gps의 위치 좌표 마커위함(적색)

        # PGM 및 YAML 파일 경로 설정
        self.pgm_file_path = '/home/ksj/locali_ws/src/local_pgm/map/youfuck2.pgm'
        self.yaml_file_path = '/home/ksj/locali_ws/src/local_pgm/map/youfuck.yaml'

# PGM 및 YAML 파일 불러오기
        self.load_map(self.pgm_file_path, self.yaml_file_path)
        self.publish_map_callback()

    def load_map(self, pgm_file_path, yaml_file_path):
        """PGM 파일과 YAML 파일을 불러와서 맵 정보를 설정합니다."""
        try:
            with open(yaml_file_path, 'r') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)

            self.image = cv2.imread(pgm_file_path, cv2.IMREAD_UNCHANGED)
            if self.image is None:
                self.get_logger().error("이미지를 불러오는 데 실패했습니다.")
                return
            
            self.image = cv2.flip(self.image,1)

            self.resolution = yaml_data.get('resolution')
            self.origin = yaml_data.get('origin')
            negate = yaml_data.get('negate')
            occupied_thresh = yaml_data.get('occupied_thresh')
            free_thresh = yaml_data.get('free_thresh')
            self.get_logger().info(f"resolution: {self.resolution}, origin: {self.origin}")

            if negate == 1:
                self.image = cv2.bitwise_not(self.image)

            self.image_shape = self.image.shape
            self.occupancy_data = []
            for pixel in self.image.flatten():
                if pixel >= occupied_thresh:
                    self.occupancy_data.append(0)  # 장애물 없음
                elif pixel <= free_thresh:
                    self.occupancy_data.append(100)  # 장애물 있음
                else:
                    self.occupancy_data.append(-1)  # 알 수 없음

        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")

    def publish_map_callback(self): # 전체 pgm 맵을 발행
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"

        grid_msg.info.height, grid_msg.info.width = self.image_shape
        grid_msg.info.resolution = self.resolution
        grid_msg.info.origin.position.x = self.origin[0]
        grid_msg.info.origin.position.y = self.origin[1]
        grid_msg.info.origin.position.z = self.origin[2]
        grid_msg.info.origin.orientation.w = 0.0

        grid_msg.data = self.occupancy_data
        self.map_publisher_.publish(grid_msg)
        self.get_logger().info("map을 성공적으로 발행했습니다19.")

    def sensor_marker(self): # gps 센서의 위치 마커를 찍는 함수(red)
        self.sen_marker_array.markers.clear()

        sen_visual_marker = Marker()
        sen_visual_marker.header.frame_id = "map"
        sen_visual_marker.header.stamp = self.get_clock().now().to_msg()
        sen_visual_marker.ns = "gps"
        sen_visual_marker.id = 10000
        sen_visual_marker.type = Marker.SPHERE
        sen_visual_marker.action = Marker.ADD

        sen_visual_marker.pose.position = Point(x=self.longitude, y=self.latitude, z=1.0)
        sen_visual_marker.scale.x = 0.00005
        sen_visual_marker.scale.y = 0.00005
        sen_visual_marker.scale.z = 0.00005

        sen_visual_marker.color.r = 1.0
        sen_visual_marker.color.g = 0.0
        sen_visual_marker.color.b = 0.0
        sen_visual_marker.color.a = 1.0

        sen_visual_marker.lifetime = Duration(sec=0, nanosec=0)
        self.sen_marker_array.markers.append(sen_visual_marker)

        self.sen_marker_publisher_.publish(self.sen_marker_array)
        self.get_logger().info(f"{len(self.sen_marker_array.markers)}개의 now_gps마커가 발행되었습니다.")

        # 마커의 중심계산 (x=0 y=0)
        center_pixel_x = int((self.origin[0]-self.longitude)/self.resolution)
        center_pixel_y = int((self.origin[1]-self.latitude)/self.resolution)

        self.get_logger().info(f"origin x={self.origin[0]}, origin y={self.origin[1]}")
        self.get_logger().info(f"x center pixel: x pixel={center_pixel_x}, y pixel={center_pixel_y} ")

        # 잘라낼 영역의 픽셀 좌표 계산 (50픽셀
        crop_size = 20
        x_min = max(center_pixel_x - crop_size // 2, 0) # >> 0
        x_max = (min(center_pixel_x + crop_size // 2, self.image_shape[1])) # >>25
        y_min = max(center_pixel_y - crop_size // 2, 0) # >>0
        y_max = (min(center_pixel_y + crop_size // 2, self.image_shape[0])) # >> 25
        self.get_logger().info(f"xmin={x_min}, ymin={y_min}")

        # PGM 이미지 잘라내기
        self.cropped_image = self.image[y_min:y_max, x_min:x_max]
        self.cropped_image_shape = self.cropped_image.shape[:2]  # (height, width) 형태로 저장

        print(self.latitude, self.longitude)
        self.get_logger().info(f"잘라낸 이미지 크기: {self.cropped_image_shape}")
        self.get_logger().info(f"just map: {self.image_shape}")
        # self.publish_map_callback() #big map
        self.broadcast_transform()

        #self.publish_cropped_map(x_min, y_min)

    def publish_e_marker(self):
        self.get_logger().info("publish_e_marker 호출됨")
        e_marker_data = self.ref.child('id/da1218/information/ApptoRobot').get()
        
        if e_marker_data:
            e_coordinates, _ = parse_coordinates(e_marker_data) 
            self.update_e_markers(e_coordinates)
            self.get_logger().info(f"e_marker_data: {e_coordinates}")  # 로그 추가

    def update_e_markers(self, e_coordinates):
        """마커 좌표를 기반으로 RViz에 마커를 추가합니다."""
        self.e_marker_array.markers.clear()

        for i, (self.lat, self.lon) in enumerate(e_coordinates):
            self.get_logger().info(f"eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee마커 좌표: lat={self.lat}, lon={self.lon}")
            e_marker = Marker()
            e_marker.header.frame_id = "map"
            e_marker.header.stamp = self.get_clock().now().to_msg()
            e_marker.ns = "e_coordinates"
            e_marker.id = i
            e_marker.type = Marker.SPHERE                                    

            e_marker.pose.position = Point(x=self.lon, y=self.lat, z=0.0)
            e_marker.scale.x = 1.0
            e_marker.scale.y = 1.0
            e_marker.scale.z = 0.0001

            e_marker.color.r = 0.0
            e_marker.color.g = 1.0
            e_marker.color.b = 0.0
            e_marker.color.a = 1.0

            e_marker.lifetime = Duration(sec=0, nanosec=0)
            self.e_marker_array.markers.append(e_marker)

        self.e_marker_publisher_.publish(self.e_marker_array)

    def publish_c_marker(self):
        c_marker_data = self.ref.child('id/da1218/information/ApptoRobot').get()

        if c_marker_data:  
           c_coordinates,_ = parse_coordinates(c_marker_data)
           self.update_c_markers(c_coordinates) 
           self.get_logger().info(f"c_marker_data: {c_coordinates}")  # 로그 추가  

    def update_c_markers(self, c_coordinates):
        """마커 좌표를 기반으로 RViz에 마커를 추가합니다."""
        self.c_marker_array.markers.clear()

        for j, (self.lat, self.lon) in enumerate(c_coordinates):
            self.get_logger().info(f"ccccccccccccccccccccccccccccccccc마커 좌표: lat={self.lat}, lon={self.lon}")
            c_marker = Marker()
            c_marker.header.frame_id = "map"
            c_marker.header.stamp = self.get_clock().now().to_msg()
            c_marker.ns = "c_coordinates"
            c_marker.id = j
            c_marker.type = Marker.SPHERE
            c_marker.action = Marker.ADD

            c_marker.pose.position = Point(x=self.lon, y=self.lat, z=0.0)
            c_marker.scale.x = 1.0
            c_marker.scale.y = 1.0 #0.00005
            c_marker.scale.z = 0.0001

            c_marker.color.r = 0.0
            c_marker.color.g = 0.0
            c_marker.color.b = 1.0
            c_marker.color.a = 1.0

            c_marker.lifetime = Duration(sec=0, nanosec=0)
            self.c_marker_array.markers.append(c_marker)

        self.c_marker_publisher_.publish(self.c_marker_array)     

    def publish_cropped_map(self, x_offset, y_offset):
        cropped_occupancy_data = []
        # 잘라낸 이미지 데이터를 occupancy_data로 변환
        for pixel in self.cropped_image.flatten():
            if pixel >= 255:  # 장애물
                cropped_occupancy_data.append(0)
            elif pixel < 255:  # 비어 있음
                cropped_occupancy_data.append(100)
            else:  # 알 수 없음
                cropped_occupancy_data.append(-1)

        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"
        grid_msg.info.height, grid_msg.info.width = self.cropped_image_shape
        grid_msg.info.resolution = self.resolution

        # 원점을 마커의 위치로 설정
        grid_msg.info.origin.position.x = self.longitude + 0.001 #자른 pgm중간에 마커 찍기 
        grid_msg.info.origin.position.y = self.latitude + 0.001 #자른 pgm중간에 마커 찍기 
        grid_msg.info.origin.position.z = 0.009
        grid_msg.info.origin.orientation.w = 0.0
        grid_msg.data = cropped_occupancy_data
        self.map_publisher_.publish(grid_msg)
        self.get_logger().info("잘라낸 이미지를 성공적으로 발행했습니다.")
    # 마커의 위치를 픽셀 좌표로 변환
        marker_pixel_x = ((self.origin[0] + (x_offset * self.resolution) - self.origin[0]) / self.resolution)
        marker_pixel_y = ((self.origin[1] + (y_offset * self.resolution) - self.origin[1]) / self.resolution)

        self.get_logger().info(f"offX={x_offset}, offY ={y_offset}")
        self.get_logger().info(f"positonX = {grid_msg.info.origin.position.x}, positionY = {grid_msg.info.origin.position.y}")
        self.get_logger().info(f"Mx={marker_pixel_x}, My={marker_pixel_y}")

    def timer_callback(self):
        gps_msg = NavSatFix()
        gps_msg.latitude = self.latitude
        gps_msg.longitude = self.longitude
        self.gps_callback(gps_msg)

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.get_logger().info(f"수신한 GPS 데이터: 위도: {self.latitude}, 경도: {self.longitude}")
        # 변환 방송
        # self.broadcast_transform()

    def broadcast_transform(self):
        # map과 gps 프레임 간의 변환도 설정
        t_map_to_gps = geometry_msgs.msg.TransformStamped()
        t_map_to_gps.header.stamp = self.get_clock().now().to_msg()
        t_map_to_gps.header.frame_id = 'map'
        t_map_to_gps.child_frame_id = 'gps_frame'

        t_map_to_gps.transform.translation.x = self.longitude
        t_map_to_gps.transform.translation.y = self.latitude
        t_map_to_gps.transform.translation.z = 0.0

        t_map_to_gps.transform.rotation.x = 0.0
        t_map_to_gps.transform.rotation.y = 0.0
        t_map_to_gps.transform.rotation.z = 0.0
        t_map_to_gps.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t_map_to_gps)
        #self.get_logger().info("map과 gps 프레임 변환을 발행했습니다.")
        self.get_logger().info(f"위도: {self.latitude}, 경도: {self.longitude}")
        self.get_logger().info(f"위도: {t_map_to_gps.transform.translation.x}, 경도: {t_map_to_gps.transform.translation.y}")

        # gps -> base_link 변환
        t_gps_to_base = geometry_msgs.msg.TransformStamped()
        t_gps_to_base.header.stamp = self.get_clock().now().to_msg()
        t_gps_to_base.header.frame_id = 'gps_frame'
        t_gps_to_base.child_frame_id = 'base_link'

        t_gps_to_base.transform.translation.x = 0.0
        t_gps_to_base.transform.translation.y = 0.0
        t_gps_to_base.transform.translation.z = 0.0

        t_gps_to_base.transform.rotation.x = 0.0
        t_gps_to_base.transform.rotation.y = 0.0
        t_gps_to_base.transform.rotation.z = 0.0
        t_gps_to_base.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t_gps_to_base)
        self.get_logger().info("gps와 base_link 프레임 변환을 발행했습니다.")

def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = TfBroadcaster()
    rclpy.spin(tf_broadcaster)
    tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
