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
           print("c=",c_coordinates)
        
        elif i == 1:  # 두 번째 좌표
           e_coordinates.append((lat, lon))
           print("e=",e_coordinates)

    return c_coordinates, e_coordinates    

class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
         
        self.latitude = 50.0 #37.632778 #37.631125 # 보드에서 gps센서가 보내주는 위경도 값을 latitude, longitude에 저장
        self.longitude = 50.0 #127.055389 #127.054211

        # Firebase 초기화
        if not firebase_admin._apps:
            cred = credentials.Certificate("/home/ksj/Downloads/dbtest-c6461-firebase-adminsdk-rfe4b-24bf021e56.json")
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://dbtest-admin.firebaseio.com'
            })
        self.ref = db.reference('admin')

        # TF 브로드캐스터 초기화
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # MarkerArray 메시지 발행 설정
        self.e_marker_publisher_ = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.e_marker_timer = self.create_timer(1.1,self.publish_e_marker)

        self.c_marker_publisher_ = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.c_marker_timer = self.create_timer(1.2,self.publish_c_marker)

        self.sen_marker_publisher_ = self.create_publisher(MarkerArray, '/visualization_marker_array',10)
        self.sen_marker_timer = self.create_timer(1.3,self.sensor_marker)

        self.e_marker_array = MarkerArray() # db에서 받아온 위치 좌표 마커위함(녹색)
        self.c_marker_array = MarkerArray() 
        self.sen_marker_array = MarkerArray() #현 gps의 위치 좌표 마커위함(적색)

        # PGM 및 YAML 파일 경로 설정
        self.pgm_file_path = '/home/ksj/locali_ws/src/local_pgm/map/youfuck2_550.pgm'
        self.yaml_file_path = '/home/ksj/locali_ws/src/local_pgm/map/youfuck.yaml'

        self.subscription = self.create_subscription(NavSatFix,'/gps/fix', self.gps_callback,10)  
        # PGM 및 YAML 파일 불러오기
        self.load_map(self.pgm_file_path, self.yaml_file_path)
        # map 메시지 발행 설정
        self.map_publisher_ =  self.create_publisher(OccupancyGrid, 'map', 10)
        self.map_timer = self.create_timer(1.0,self.publish_map_callback)

    def load_map(self, pgm_file_path, yaml_file_path):
        """PGM 파일과 YAML 파일을 불러와서 맵 정보를 설정합니다."""
        try:
            with open(yaml_file_path, 'r') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)

            # 이미지 불러오기
            self.image = cv2.imread(pgm_file_path, cv2.IMREAD_UNCHANGED)
            
            if self.image is None:
                self.get_logger().error("이미지를 불러오는 데 실패했습니다.")
                return

            # 이미지 회전: 20도
            (h, w) = self.image.shape[:2]
            center = (w // 2, h // 2)
            rotation_matrix = cv2.getRotationMatrix2D(center, 90, 1.0)
            self.image = cv2.warpAffine(self.image, rotation_matrix, (w, h))

            # 해상도 및 원점 정보 설정
            self.resolution = yaml_data.get('resolution')
            self.origin = yaml_data.get('origin')
            negate = yaml_data.get('negate')
            occupied_thresh = yaml_data.get('occupied_thresh')
            free_thresh = yaml_data.get('free_thresh')
            # self.get_logger().info(f"resolution: {self.resolution}, origin: {self.origin}")

            # 네거티브 이미지 설정
            if negate == 1:
                self.image = cv2.bitwise_not(self.image)

            # 맵 데이터를 OccupancyGrid로 변환
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
            self.get_logger().error(f"맵을 불러오는 중 오류 발생: {e}")

    def publish_map_callback(self): # 전체 pgm 맵을 발행
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"

        grid_msg.info.height, grid_msg.info.width = self.image_shape
        grid_msg.info.resolution = self.resolution
        grid_msg.info.origin.position.x = self.origin[0] + 0.00340# value low -> tf move right #0.0314
        grid_msg.info.origin.position.y = self.origin[1] + 0.00355# value low -> tf move up
        grid_msg.info.origin.position.z = self.origin[2]
        grid_msg.info.origin.orientation.w = 0.0

        grid_msg.data = self.occupancy_data
        self.map_publisher_.publish(grid_msg)
        self.get_logger().info("map을 성공적으로 발행했습니다.")

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
        self.get_logger().info(f"gps마커가 발행되었습니다.")

        self.broadcast_transform()

    def publish_e_marker(self):
        self.get_logger().info("publish_e_marker 호출됨")
        e_marker_data = self.ref.child('id/da1218/information/ApptoRobot').get()
        
        if e_marker_data:
            c_coordinates, e_coordinates = parse_coordinates(e_marker_data) 
            self.get_logger().info(f"e_corordinates: {e_coordinates}")  # 로그 추가
            self.update_e_markers(e_coordinates)

    def update_e_markers(self, e_coordinates):
        """마커 좌표를 기반으로 RViz에 마커를 추가합니다."""
        self.e_marker_array.markers.clear()

        for i, (lat, lon) in enumerate(e_coordinates):
            self.get_logger().info(f"e마커 좌표: lat={lat}, lon={lon}")
            e_marker = Marker()
            e_marker.header.frame_id = "map"
            e_marker.header.stamp = self.get_clock().now().to_msg()
            e_marker.ns = "e_coordinates"
            e_marker.id = i
            e_marker.type = Marker.SPHERE                                    

            e_marker.pose.position = Point(x=lat, y=lon, z=0.0)

            e_marker.scale.x = 0.00005
            e_marker.scale.y = 0.00005                                                        
            e_marker.scale.z = 0.00001

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
           self.get_logger().info(f"c_coordinates: {c_coordinates}")  # 로그 추가  
           self.update_c_markers(c_coordinates) 

    def update_c_markers(self, c_coordinates):
        """마커 좌표를 기반으로 RViz에 마커를 추가합니다."""
        self.c_marker_array.markers.clear()

        for j, (lat, lon) in enumerate(c_coordinates):
            self.get_logger().info(f"c마커 좌표: lat={lat}, lon={lon}")
            c_marker = Marker()
            c_marker.header.frame_id = "map"
            c_marker.header.stamp = self.get_clock().now().to_msg()
            c_marker.ns = "c_coordinates"
            c_marker.id = j
            c_marker.type = Marker.SPHERE
            c_marker.action = Marker.ADD

            c_marker.pose.position = Point(x=lat, y=lon, z=0.0)  
            c_marker.scale.x = 0.00005
            c_marker.scale.y = 0.00005 #0.00005
            c_marker.scale.z = 0.00001

            c_marker.color.r = 0.0
            c_marker.color.g = 0.0
            c_marker.color.b = 1.0
            c_marker.color.a = 1.0

            c_marker.lifetime = Duration(sec=0, nanosec=0)
            self.c_marker_array.markers.append(c_marker)

        self.c_marker_publisher_.publish(self.c_marker_array)     

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.get_logger().info(f"수신한 GPS 데이터: 위도: {self.latitude}, 경도: {self.longitude}")
    
    def broadcast_transform(self):
        # map과 gps 프레임 간의 변환도 설정
        map_to_gps = geometry_msgs.msg.TransformStamped()
        map_to_gps.header.stamp = self.get_clock().now().to_msg()
        map_to_gps.header.frame_id = 'map'
        map_to_gps.child_frame_id = 'gps_frame'

        map_to_gps.transform.translation.x = self.latitude 
        map_to_gps.transform.translation.y = self.longitude
        map_to_gps.transform.translation.z = 0.0

        map_to_gps.transform.rotation.x = 0.0
        map_to_gps.transform.rotation.y = 0.0
        map_to_gps.transform.rotation.z = 0.0
        map_to_gps.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(map_to_gps)
        self.get_logger().info(f"위도: {map_to_gps.transform.translation.x}, 경도: {map_to_gps.transform.translation.y}")

        # # gps -> base_link 변환
        # t_gps_to_cafe = geometry_msgs.msg.TransformStamped()
        # t_gps_to_cafe.header.stamp = self.get_clock().now().to_msg()
        # t_gps_to_cafe.header.frame_id = 'map'
        # t_gps_to_cafe.child_frame_id = 'cafe_frame'

        # t_gps_to_cafe.transform.translation.x = 37.632142823831416 
        # t_gps_to_cafe.transform.translation.y = 127.05484673090155
        # t_gps_to_cafe.transform.translation.z = 0.0

        # t_gps_to_cafe.transform.rotation.x = 0.0
        # t_gps_to_cafe.transform.rotation.y = 0.0
        # t_gps_to_cafe.transform.rotation.z = 0.0
        # t_gps_to_cafe.transform.rotation.w = 1.0 
        # self.broadcaster.sendTransform(t_gps_to_cafe)

        # t_gps_to_stop3 = geometry_msgs.msg.TransformStamped()
        # t_gps_to_stop3.header.stamp = self.get_clock().now().to_msg()
        # t_gps_to_stop3.header.frame_id = 'map'
        # t_gps_to_stop3.child_frame_id = 'stop3_frame'

        # t_gps_to_stop3.transform.translation.x = 37.631255
        # t_gps_to_stop3.transform.translation.y = 127.054262
        # t_gps_to_stop3.transform.translation.z = 0.0

        # t_gps_to_stop3.transform.rotation.x = 0.0
        # t_gps_to_stop3.transform.rotation.y = 0.0
        # t_gps_to_stop3.transform.rotation.z = 0.0
        # t_gps_to_stop3.transform.rotation.w = 1.0
        # self.broadcaster.sendTransform(t_gps_to_stop3)

        # t_gps_to_stop2 = geometry_msgs.msg.TransformStamped()
        # t_gps_to_stop2.header.stamp = self.get_clock().now().to_msg()
        # t_gps_to_stop2.header.frame_id = 'map'
        # t_gps_to_stop2.child_frame_id = 'stop2_frame'

        # t_gps_to_stop2.transform.translation.x = 37.631587 
        # t_gps_to_stop2.transform.translation.y = 127.054429
        # t_gps_to_stop2.transform.translation.z = 0.0

        # t_gps_to_stop2.transform.rotation.x = 0.0
        # t_gps_to_stop2.transform.rotation.y = 0.0
        # t_gps_to_stop2.transform.rotation.z = 0.0
        # t_gps_to_stop2.transform.rotation.w = 1.0
        # self.broadcaster.sendTransform(t_gps_to_stop2)

        # t_gps_to_stop1 = geometry_msgs.msg.TransformStamped()
        # t_gps_to_stop1.header.stamp = self.get_clock().now().to_msg()
        # t_gps_to_stop1.header.frame_id = 'map'
        # t_gps_to_stop1.child_frame_id = 'stop1_frame'

        # t_gps_to_stop1.transform.translation.x = 37.631889 
        # t_gps_to_stop1.transform.translation.y = 127.054611
        # t_gps_to_stop1.transform.translation.z = 0.0

        # t_gps_to_stop1.transform.rotation.x = 0.0
        # t_gps_to_stop1.transform.rotation.y = 0.0
        # t_gps_to_stop1.transform.rotation.z = 0.0
        # t_gps_to_stop1.transform.rotation.w = 1.0
        # self.broadcaster.sendTransform(t_gps_to_stop1)

        # t_gps_to_end = geometry_msgs.msg.TransformStamped()
        # t_gps_to_end.header.stamp = self.get_clock().now().to_msg()
        # t_gps_to_end.header.frame_id = 'map'
        # t_gps_to_end.child_frame_id = 'cafe_end_frame'

        # t_gps_to_end.transform.translation.x = 37.632217 
        # t_gps_to_end.transform.translation.y = 127.054733
        # t_gps_to_end.transform.translation.z = 0.0

        # t_gps_to_end.transform.rotation.x = 0.0
        # t_gps_to_end.transform.rotation.y = 0.0
        # t_gps_to_end.transform.rotation.z = 0.0
        # t_gps_to_end.transform.rotation.w = 1.0
        # self.broadcaster.sendTransform(t_gps_to_end)

        # t_gps_to_stop3 = geometry_msgs.msg.TransformStamped()
        # t_gps_to_stop3.header.stamp = self.get_clock().now().to_msg()
        # t_gps_to_stop3.header.frame_id = 'map'
        # t_gps_to_stop3.child_frame_id = 'stop3_frame'

        # t_gps_to_stop3.transform.translation.x = 37.631255 
        # t_gps_to_stop3.transform.translation.y = 127.054262
        # t_gps_to_stop3.transform.translation.z = 0.0

        # t_gps_to_stop3.transform.rotation.x = 0.0
        # t_gps_to_stop3.transform.rotation.y = 0.0
        # t_gps_to_stop3.transform.rotation.z = 0.0
        # t_gps_to_stop3.transform.rotation.w = 1.0
        # self.broadcaster.sendTransform(t_gps_to_stop3) 

        # t_gps_to_parking = geometry_msgs.msg.TransformStamped()
        # t_gps_to_parking.header.stamp = self.get_clock().now().to_msg()
        # t_gps_to_parking.header.frame_id = 'map'
        # t_gps_to_parking.child_frame_id = 'parking_frame'

        # t_gps_to_parking.transform.translation.x = 37.631000
        # t_gps_to_parking.transform.translation.y = 127.054139
        # t_gps_to_parking.transform.translation.z = 0.0

        # t_gps_to_parking.transform.rotation.x = 0.0
        # t_gps_to_parking.transform.rotation.y = 0.0
        # t_gps_to_parking.transform.rotation.z = 0.0
        # t_gps_to_parking.transform.rotation.w = 1.0
        # self.broadcaster.sendTransform(t_gps_to_parking)

def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = TfBroadcaster()
    rclpy.spin(tf_broadcaster)
    tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()