import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import time
import tf_broadcaster

def parse_gps_data(nmea_sentence):
    parts = nmea_sentence.split(',')
    if len(parts) > 10:
        latitude = parts[9]
        longitude = parts[11]
        print(nmea_sentence)
        return float(latitude), float(longitude)
    else:
        return None, None

class GpsReceiver(Node):
    def __init__(self):
        super().__init__('gps_receiver')

        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        serial_port = '/dev/ttyUSB0'
        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=1)
            time.sleep(2)  # 시리얼 포트 안정화 대기
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 포트 오류: {e}")
            self.ser = None  # 초기화 실패 시 None으로 설정

    def timer_callback(self):
        if self.ser is not None and self.ser.in_waiting > 0:  # self.ser이 None이 아닐 때만 접근
            nmea_sentence = self.ser.readline().decode('ascii', errors='replace').strip()
            latitude, longitude = parse_gps_data(nmea_sentence)

            if latitude is not None and longitude is not None:
                msg = NavSatFix()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'gps_link'
                msg.latitude = latitude
                msg.longitude = longitude

                self.publisher_.publish(msg)
                self.get_logger().info(f"GPS 데이터 발행: 위도: {latitude}, 경도: {longitude}")

def main(args=None):
    rclpy.init(args=args)
    gps_receiver = GpsReceiver()
    rclpy.spin(gps_receiver)
    gps_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
