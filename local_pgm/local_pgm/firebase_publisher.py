import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
from sensor_msgs.msg import NavSatFix, Imu  # IMU 센서와 GPS 센서 메시지 타입

class FirebasePublisher(Node):
    def __init__(self):
        super().__init__('firebase_publisher')

        # Firebase 초기화
        if not firebase_admin._apps:
            cred = credentials.Certificate("/home/ksj/Downloads/dbtest-c6461-firebase-adminsdk-rfe4b-24bf021e56.json")
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://dbtest-admin.firebaseio.com'
            })                                                                                                                                                   
        # GPS 및 IMU 데이터 구독자 생성
        # self.gps_subscription = self.create_subscription(
        #     NavSatFix,
        #     '/gps/fix',
        #     self.gps_callback,
        #     10)
        
        # self.imu_subscription = self.create_subscription(
        #     Imu,
        #     '/imu',
        #     self.imu_callback,
        #     10)
        
        # Firebase에 주기적으로 데이터를 업로드하는 타이머
        # self.timer = self.create_timer(0.5, self.publish_to_firebase)
        self.timer = self.create_timer(0.5, self.gps_callback)
        self.timer = self.create_timer(0.7, self.imu_callback)
        self.timer = self.create_timer(0.6, self.publish_to_firebase)
        
        # 데이터를 저장할 변수 초기화
        self.lat = None
        self.lon = None
        self.yaw = None

    # GPS 콜백 함수
    def gps_callback(self):
        self.lat = 37.631085838975299  #msg.latitude  37.631085838975004, 127.05416703835846,60
        self.lon = 127.05416703835846 #msg.longitude
        self.get_logger().info(f"Received GPS data12: latitude={self.lat}, longitude={self.lon}")

    # IMU 콜백 함수
    def imu_callback(self):
        # Quaternion을 변환하지 않고 yaw 값만 직접 사용
        self.yaw =  30  # 필요에 따라 수정 가능
        self.get_logger().info(f"Received IMU data: yaw={self.yaw}")      

    # Firebase에 데이터 업로드
    def publish_to_firebase(self):
        if self.lat is not None and self.lon is not None and self.yaw is not None:
            data_string = f"{self.lat},{self.lon},{self.yaw}"
            ref = db.reference('/admin/id/da1218/information/RobottoApp')
            ref.set(data_string)
            self.get_logger().info("Data added to Firebase!")

def main(args=None):
    rclpy.init(args=args)
    firebase_publisher = FirebasePublisher()
    rclpy.spin(firebase_publisher)
    firebase_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

